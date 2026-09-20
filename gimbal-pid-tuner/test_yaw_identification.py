import io
import math
import re
from pathlib import Path
import struct
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import patch

from yaw_capture_protocol import crc8, frame
from yaw_identification_protocol import (
    ARM, BUILD, CANCEL, DOWNLOAD, KEEPALIVE, MAGIC, META, PROBE, RECORD, RELEASE, REMOTE,
    Decoder, request,
)
from yaw_identification_cli import run


def group(command, sequence, data, version=1):
    return b''.join(frame(command, struct.pack('<HBB', sequence, part, version) + data[part*8:part*8+8])
                    for part in range(len(data) // 8))


def capture(count=1001, phase=5, trial=1, start=100, gap=False, build=BUILD):
    metadata = META.pack(trial, start, build, count, 4, phase, 0, 1, 1, 1, *([0.] * 42))
    rows = [RECORD.pack((start+i*4+(1 if gap and i else 0)) & 0xffffffff,
                        720., 0., 0., 0., 1., 2., 8191, 6816, 0, 0, 0, 0, 0,
                        1, 0, 0, 0, 5 if i == count-1 else 3) for i in range(count)]
    return group(0x39, 0, metadata) + b''.join(group(0x3a, i+1, r) for i, r in enumerate(rows))


class ProtocolTests(unittest.TestCase):
    def test_layout_and_requests(self):
        self.assertEqual(RECORD.size, 48)
        self.assertEqual(META.size, 192)
        data = request(ARM, 123, 2, 1.25)
        self.assertEqual(struct.unpack('<IHBBI', data[2:14]), (123, 125, 1, 2, MAGIC))
        self.assertEqual(data[14], crc8(data[:14]))
        for value in (math.nan, math.inf, -1, 0, 5.01):
            with self.assertRaises(ValueError):
                request(ARM, 1, 1, value)
        for op in (CANCEL, DOWNLOAD, RELEASE):
            with self.assertRaises(ValueError):
                request(op)
        self.assertEqual(request()[3:10], bytes(7))

    def test_bytewise_complete_and_wrap(self):
        d = Decoder(1)
        for byte in capture(start=0xfffffffc):
            d.feed(bytes([byte]))
        self.assertTrue(d.complete)
        self.assertEqual(d.report()['quality_issues'], [])
        self.assertEqual(d.rows[1]['tick_ms'], 0)

    def test_trial_binding(self):
        with self.assertRaises(ValueError):
            Decoder(2).feed(capture(count=1))
        with self.assertRaises(ValueError):
            Decoder().feed(capture(count=1))

    def test_previous_build_capture_remains_readable(self):
        d = Decoder(1)
        d.feed(capture(build=0x59490105))
        self.assertTrue(d.complete)
        self.assertEqual(d.metadata['build'], 0x59490105)

    def test_reject_wrong_firmware_before_arm(self):
        d = Decoder()
        with self.assertRaises(ValueError):
            d.feed(frame(0x3b, struct.pack('<IIHH', BUILD+1, MAGIC, 1, 48)))

    def test_missing_duplicate_reorder(self):
        data = capture(count=1)
        for corrupt in (data[:16]+data[32:], data[:16]+data, data[16:32]+data[:16]+data[32:]):
            with self.assertRaises(ValueError):
                Decoder(1).feed(corrupt)

    def test_crc_and_truncated(self):
        data = bytearray(capture(count=1)); data[9] ^= 1
        d = Decoder(1)
        with self.assertRaises(ValueError):
            d.feed(data)
        self.assertEqual(d.crc_errors, 1)
        d = Decoder(1); d.feed(capture(count=1)[:-1])
        self.assertFalse(d.complete)

    def test_aborted_partial_and_gaps_preserved(self):
        d = Decoder(1); d.feed(capture(count=3, phase=6, gap=True))
        self.assertTrue(d.complete)
        self.assertIn('trial_not_completed', d.report()['quality_issues'])
        self.assertIn('irregular_sample_interval', d.report()['quality_issues'])

    def test_empty_abort(self):
        d = Decoder(1); d.feed(capture(count=0, phase=6))
        self.assertTrue(d.complete)
        self.assertEqual(len(d.rows), 0)

    def test_remote_mask_and_fragment_snapshot(self):
        data = REMOTE.pack(100, -11, -10, 0, 10, 101, 2, -3, 1, 0, 0, 241)
        d = Decoder()
        for byte in group(0x3c, 65535, data, version=2):
            d.feed(bytes([byte]))
        self.assertEqual(d.remote['failed_inputs'], ['ch0', 'ch4', 'mouse_x', 'mouse_y', 'mouse_left'])
        self.assertEqual(d.remote['ch2'], 0)
        self.assertEqual(d.remote['sequence'], 65535)
        self.assertIsNone(d.metadata)
        d.feed(group(0x3c, 0, REMOTE.pack(101, 10, -10, 0, 0, 0, 0, 0, 0, 0, 5, 0)))
        self.assertEqual(d.remote['failed_inputs'], [])

    def test_remote_rejects_mixed_and_bad_mask(self):
        data = REMOTE.pack(1, *([0] * 11))
        wire = group(0x3c, 1, data)
        with self.assertRaises(ValueError):
            Decoder().feed(wire[:16] + wire[32:])
        with self.assertRaises(ValueError):
            Decoder().feed(wire[:16] + group(0x3c, 2, data)[16:])
        with self.assertRaises(ValueError):
            Decoder().feed(wire[:16] + group(0x3c, 1, data, version=2)[16:])
        data = REMOTE.pack(1, 11, *([0] * 10))
        with self.assertRaises(ValueError):
            Decoder().feed(group(0x3c, 1, data))

    def test_remote_wheel_policy_and_old_recordings(self):
        for version in (1, 2):
            for wheel in (-660, -11, 0, 10, 11, 100, 101, 660):
                with self.subTest(version=version, wheel=wheel):
                    failed = abs(wheel) > 10 if version == 1 else wheel > 100
                    data = REMOTE.pack(1, 0, 0, 0, 0, wheel, 0, 0, 0, 0, 0, 16 if failed else 0)
                    d = Decoder(); d.feed(group(0x3c, 1, data, version))
                    self.assertEqual(d.remote['failed_inputs'], ['ch4'] if failed else [])
                    self.assertEqual(d.remote['version'], version)
        data = REMOTE.pack(1, 0, 0, 0, 0, 101, 0, 0, 0, 0, 0, 0)
        with self.assertRaises(ValueError):
            Decoder().feed(group(0x3c, 1, data, version=2))

    def test_production_remote_wire_fixture(self):
        path = Path(__file__).resolve().parents[1] / 'NoMachineTemp/yaw-ident-remote-fixture.bin'
        if not path.exists():
            self.skipTest('run the C remote fixture generator first')
        d = Decoder(); d.feed(group(0x3c, 1, path.read_bytes(), version=2))
        self.assertEqual(d.remote['failed_mask'], 241)
        self.assertEqual(d.remote['ch2'], 0)
        self.assertEqual(d.remote['mouse_y'], -3)

    def test_production_c_wire_fixture(self):
        path = Path(__file__).resolve().parents[1] / 'NoMachineTemp/yaw-ident-wire-fixture.bin'
        if not path.exists():
            self.skipTest('run the C supervisor fixture generator first')
        data = path.read_bytes()
        self.assertEqual(len(data), 192 + 1001 * 48)
        wire = group(0x39, 0, data[:192])
        for i in range(1001):
            wire += group(0x3a, i+1, data[192+i*48:192+(i+1)*48])
        d = Decoder(1); d.feed(wire)
        self.assertEqual(d.report()['quality_issues'], [])
        self.assertEqual(d.metadata['configuration']['big_angle_kp'], 50)
        self.assertEqual(d.rows[0]['small_raw'], 6816)


class FakeSerial:
    def __init__(self, port):
        self.sent, self.pending, self.closed = [], bytearray(), False
        self.phase, self.trial = 0, 0
        self.lose_arm_ack = port == 'lost_ack'

    def send(self, packet):
        trial, _, op, _, _ = struct.unpack('<IHBBI', packet[2:14])
        self.sent.append(op)
        if op == ARM:
            self.trial, self.phase = trial, 5
            if self.lose_arm_ack:
                return
        if self.lose_arm_ack and op == KEEPALIVE:
            return
        if op in (PROBE, KEEPALIVE):
            self.pending.extend(frame(0x37, struct.pack('<IHBBBBH', self.trial,
                                1001 if self.trial else 0, self.phase, 0, 253 if self.trial else 252, 1, 1)))
            self.pending.extend(frame(0x3b, struct.pack('<IIHH', BUILD, MAGIC, 1, 48)))
            if op == PROBE and not self.trial:
                self.pending.extend(group(0x3c, 1, REMOTE.pack(100, *([0] * 11)), version=2))
        else:
            self.pending.extend(frame(0x38, struct.pack('<IBBBBI', trial, op, 1, self.phase, 0, 100)))
        if op == DOWNLOAD:
            self.pending.extend(capture(trial=trial))

    def read(self):
        chunk = bytes(self.pending[:4096]); del self.pending[:4096]
        return chunk

    def close(self):
        self.closed = True


class CliTests(unittest.TestCase):
    def execute(self, action, port_name='fake', remote_detail=False):
        port = FakeSerial(port_name)
        args = SimpleNamespace(port=port_name, action=action, trial_id=None, axis='big',
                               amplitude_deg=1, remote_detail=remote_detail)
        decoder = Decoder()
        ticks = iter(i*.01 for i in range(100000))
        with patch('yaw_identification_cli.time.monotonic', side_effect=lambda: next(ticks)):
            try:
                run(args, decoder, io.BytesIO(), io.StringIO(), lambda _: port)
            except RuntimeError:
                if port_name != 'lost_ack':
                    raise
        self.assertTrue(port.closed)
        return port, decoder

    def test_default_query_never_arms(self):
        port, _ = self.execute('probe')
        self.assertEqual(set(port.sent), {PROBE})

    def test_remote_query_never_arms(self):
        port, decoder = self.execute('probe', remote_detail=True)
        self.assertEqual(set(port.sent), {PROBE})
        self.assertEqual(decoder.remote['failed_inputs'], [])

    def test_trial_download_never_releases(self):
        port, decoder = self.execute('trial')
        self.assertEqual(port.sent.count(ARM), 1)
        self.assertIn(DOWNLOAD, port.sent)
        self.assertNotIn(RELEASE, port.sent)
        self.assertTrue(decoder.complete)

    def test_lost_arm_ack_attempts_cancel(self):
        port, _ = self.execute('trial', 'lost_ack')
        self.assertEqual(port.sent.count(ARM), 1)
        self.assertIn(CANCEL, port.sent)
        self.assertNotIn(DOWNLOAD, port.sent)


class DeploymentTests(unittest.TestCase):
    def test_trial_entrypoint_loads_deployed_setup_without_serial(self):
        import json
        import yaw_identification_cli as cli

        def fake_run(args, decoder, raw, events):
            decoder.trial_id = 1
            decoder.feed(capture())

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'capture'
            argv = ['yaw_identification_cli.py', '--port', '/not-a-real-serial-port',
                    '--action', 'trial', '--axis', 'big', '--amplitude-deg', '1',
                    '--confirm', 'FIXED_CHASSIS_CLEAR_YAW', '--output', str(output)]
            with patch('sys.argv', argv), patch('sys.stdout', new=io.StringIO()), \
                    patch.object(cli, 'run', side_effect=fake_run) as run_mock, \
                    patch.object(cli, 'Serial', side_effect=AssertionError('serial forbidden')):
                self.assertEqual(cli.main(), 0)
            run_mock.assert_called_once()
            setup = json.loads((output / 'setup.json').read_text(encoding='utf-8'))
            self.assertEqual(setup['setup_id'], 'fixed_chassis_no_camera_v1')
            report = json.loads((output / 'report.json').read_text(encoding='utf-8'))
            self.assertIsNone(report['error'])
            self.assertTrue(report['download_complete'])


class IntegrationContractTests(unittest.TestCase):
    """Call-site regressions only; not an RTOS scheduling or hardware simulation."""
    root = Path(__file__).resolve().parents[1] / 'AGVSentinel_v10090/AGVSentinel_gimbal/Src'

    def test_axis_selector_does_not_enable_remote_firing(self):
        control = (self.root / 'Application/app_control.c').read_text(encoding='utf-8')
        wire = (self.root.parent / 'Inc/Modules/module_yaw_ident_wire.h').read_text(encoding='utf-8')
        self.assertIn('#include "module_remote_intent.h"', wire)
        self.assertIn('> REMOTE_WHEEL_FIRE_THRESHOLD', wire)
        self.assertNotIn('remote->mouse.l', control)
        self.assertNotIn('remote->remote.ch[4]', control)
        self.assertEqual(control.count('remote->remote.s[0]'), 1)
        self.assertIn('manual_big_yaw = (uint8_t)(remote->remote.s[0] == Remote_SWITCH_DOWN)', control)
        self.assertIn('if (s_gimbal_target.manual_big_yaw) yaw_step = 0.0f;', control)
        self.assertIn('GimbalYaw_AddYawRef(applied_yaw_step);', control)
        self.assertIn('GimbalPitch_SetPitchRef(Gimbal_LimitPitch(pitch_step));', control)

    def test_middle_mode_preserves_mpc_flag_and_maps_remote_sticks(self):
        control = (self.root / 'Application/app_control.c').read_text(encoding='utf-8')
        auto = control.split('} else if (mode == CONTROL_MODE_AUTO) {', 1)[1].split(
            '\n    }\n\n    s_gimbal_target = gimbal_target;', 1)[0]
        self.assertIn('gimbal_target.flags |= CONTROL_GIMBAL_FLAG_MPC;', auto)
        self.assertIn('gimbal_target.flags |= CONTROL_GIMBAL_FLAG_ENABLE;', auto)
        self.assertNotIn('gimbal_target.flags = CONTROL_GIMBAL_FLAG_ENABLE;', auto)
        self.assertIn('(float)remote->remote.ch[2] * CONTROL_RC_YAW_CDEG_S_PER_COUNT', auto)
        self.assertIn('(float)remote->remote.ch[3] * CONTROL_RC_PITCH_MRAD_S_PER_COUNT', auto)

    def test_mpc_has_big_yaw_calculation_and_flush_permission(self):
        source = (self.root / 'Modules/module_gimbal.c').read_text(encoding='utf-8')
        calculate = source.split('static void GimbalYaw_ControlBig', 1)[1].split(
            'static void GimbalYaw_ClearSmallController', 1)[0]
        self.assertIn('(manual || mpc_active ||', calculate)
        output = source.split('void GimbalYaw_Output(void)', 1)[1].split('///', 1)[0]
        self.assertIn('uint8_t mpc_allow = (uint8_t)(!identifying &&', output)
        self.assertIn('GimbalYaw_DiagMpcActive != 0U && GimbalYaw_DiagBigActive != 0U', output)
        self.assertIn('allow = (uint8_t)(ident_allow || manual_allow || mpc_allow);', output)

    def test_manual_big_yaw_is_guarded_at_calculation_and_flush(self):
        source = (self.root / 'Modules/module_gimbal.c').read_text(encoding='utf-8')
        gate = source.split('static uint8_t GimbalYaw_BigManualPermitted', 1)[1].split('static float GimbalYaw_ClampEffort', 1)[0]
        for text in ('Remote_STATE_CONNECTED', 'BIG_YAW_MANUAL_TIMEOUT_MS',
                     'remote.remote.s[1] == Remote_SWITCH_UP',
                     'remote.remote.s[1] == Remote_SWITCH_DOWN',
                     'remote.remote.s[0] == Remote_SWITCH_DOWN',
                     '!PC_Comm_IsGimbalTuneControlLocked()', '!s_big_yaw_manual.fault'):
            self.assertIn(text, gate)
        output = source.split('void GimbalYaw_Output(void)', 1)[1].split('///', 1)[0]
        for text in ('!identifying && GimbalYaw_BigManualPermitted(now)',
                     'BigYawManual_Fresh(&command, now)',
                     'command.generation == s_big_yaw_manual.generation',
                     'GimbalYaw_StopBig(gimbalyaw)', 'GimbalImu_IsReady()'):
            self.assertLess(output.index(text), output.index('GimbalAxis_Flush'))
        calc = source.split('static void GimbalYaw_ControlBig', 1)[1].split('static void GimbalYaw_ClearSmallController', 1)[0]
        self.assertIn('!identifying && GimbalYaw_BigManualPermitted(now)', calc)
        self.assertIn('} else if (manual) {', calc)
        self.assertIn('BigYawManual_Step(', calc)
        self.assertIn('s_yaw_coordinator.feedforward_dps = 0.0f;', calc)
        self.assertNotIn('BIG_YAW_MANUAL_RATE_ABORT_RPM', source)
        manual = (self.root.parent / 'Inc/Modules/module_big_yaw_manual.h').read_text(encoding='utf-8')
        self.assertNotIn('BIG_YAW_MANUAL_RATE_ABORT_RPM', manual)
        self.assertNotIn('fabsf(speed_rpm)', manual)
        self.assertIn('!isfinite(speed_rpm)', manual)

    def test_identification_uses_persistent_pitch_gains(self):
        source = (self.root / 'Application/app_yaw_identification.c').read_text(encoding='utf-8')
        config = source.split('static void Ident_Config(', 1)[1].split('static uint8_t Ident_ConfigValid', 1)[0]
        self.assertIn('RobotActuators_GetPitchMitGains(&pitch_kp, &pitch_kd)', config)
        self.assertNotIn('.ctrl.kp_set', config)
        self.assertNotIn('.ctrl.kd_set', config)
        driver = (self.root / 'System/sys_robot_actuators.c').read_text(encoding='utf-8')
        getter = driver.split('void RobotActuators_GetPitchMitGains', 1)[1].split('uint8_t RobotActuators_AnyOffline', 1)[0]
        self.assertIn('*kp = s_pitch_context.kp_set', getter)
        self.assertIn('*kd = s_pitch_context.kd_set', getter)

    def test_big_yaw_boot_defaults_match_selected_tuning(self):
        header = (self.root.parent / 'Inc/Modules/module_big_yaw_tune.h').read_text(encoding='utf-8')
        expected = dict(BIG_YAW_ANGLE_KP=1, BIG_YAW_ANGLE_KI=0,
                        BIG_YAW_ANGLE_KD=5, BIG_YAW_SPEED_KP=.5,
                        BIG_YAW_SPEED_KI=.0001, BIG_YAW_SPEED_KD=0,
                        BIG_YAW_EFFORT_DEFAULT=30, BIG_YAW_SPEED_FILTER_TAU_S=.03)
        for name, value in expected.items():
            with self.subTest(name=name):
                match = re.search(r'^#define\s+' + name + r'\s+([0-9.]+)f\s*$', header, re.M)
                self.assertIsNotNone(match)
                self.assertEqual(float(match[1]), value)
        const = (self.root / 'System/sys_const.c').read_text(encoding='utf-8')
        module = (self.root / 'Modules/module_gimbal.c').read_text(encoding='utf-8')
        self.assertIn('{BIG_YAW_ANGLE_KP, BIG_YAW_ANGLE_KI, BIG_YAW_ANGLE_KD', const)
        self.assertIn('GimbalYaw_TuneBigAngKp = Const_GimbalYawAngParam[0][0];', module)

    def test_identification_rate_profile_and_build_match(self):
        from yaw_can_trace_protocol import BUILD as historical_streaming_build
        from yaw_dual_trace_protocol import BUILD as previous_streaming_build
        from yaw_cd_trace_protocol import BUILD as cd_streaming_build
        from yaw_speed_trace_protocol import BUILD as speed_s1_s2_build
        from yaw_s3_trace_protocol import BUILD as streaming_build
        header = (self.root.parent / 'Inc/Modules/module_yaw_identification.h').read_text(encoding='utf-8')
        wire = (self.root.parent / 'Inc/Modules/module_yaw_ident_wire.h').read_text(encoding='utf-8')
        mpc = (self.root.parent / 'Inc/Modules/module_yaw_mpc.h').read_text(encoding='utf-8')
        deployment_build = int(re.search(r'#define YAW_MPC_BUILD 0x([0-9A-F]+)UL', mpc).group(1), 16)
        self.assertEqual(BUILD, 0x59490106)
        self.assertNotEqual(BUILD, historical_streaming_build)
        self.assertNotEqual(historical_streaming_build, previous_streaming_build)
        self.assertNotEqual(previous_streaming_build, cd_streaming_build)
        self.assertNotEqual(cd_streaming_build, speed_s1_s2_build)
        self.assertNotEqual(speed_s1_s2_build, streaming_build)
        self.assertNotEqual(streaming_build, deployment_build)
        self.assertIn(f'#define YAW_IDENT_BUILD 0x{deployment_build:08X}UL', wire)
        with self.assertRaises(ValueError):
            Decoder().feed(frame(0x3b, struct.pack('<IIHH', deployment_build, MAGIC, 1, 48)))
        self.assertIn('#define YAW_IDENT_BIG_RATE_ABORT_DPS 360.0f', header)
        self.assertIn('#define YAW_IDENT_SMALL_RATE_ABORT_DPS 180.0f', header)
        self.assertIn('#define YAW_IDENT_TRAVEL_DEG 12.0f', header)

    def test_keil_build_includes_supervisor_and_full_float_checks(self):
        project = self.root.parent / 'MDK-ARM/AGVSentinel_Gimbal.uvprojx'
        target = ET.parse(project).find("./Targets/Target[TargetName='AGVSentinel_Gimbal']")
        self.assertIsNotNone(target)
        files = target.findall("./Groups/Group/Files/File[FileName='app_yaw_identification.c']")
        self.assertEqual(len(files), 1, 'identification supervisor must be in the Keil target exactly once')
        self.assertEqual(files[0].findtext('FileType'), '1')
        source = project.parent / files[0].findtext('FilePath').replace('\\', '/')
        self.assertTrue(source.is_file())
        self.assertNotEqual(files[0].findtext('./FileOption/CommonProperty/IncludeInBuild'), '0')
        mpc_files = target.findall("./Groups/Group/Files/File[FileName='module_yaw_mpc.c']")
        self.assertEqual(len(mpc_files), 1, 'deployed MPC must be in the Keil target exactly once')
        mpc_source = project.parent / mpc_files[0].findtext('FilePath').replace('\\', '/')
        self.assertTrue(mpc_source.is_file())
        flags = target.findtext('./TargetOption/TargetArmAds/Cads/VariousControls/MiscControls', '')
        self.assertIn('-ffp-mode=full', flags.split())

    def test_router_ownership_covers_all_reference_writes(self):
        source = (self.root / 'Application/app_control.c').read_text(encoding='utf-8')
        step = source.split('void Control_Step(void)', 1)[1].split('void Control_Task', 1)[0]
        self.assertLess(step.index('vTaskSuspendAll()'), step.index('YawIdentApp_OwnsControl()'))
        self.assertEqual(step.count('xTaskResumeAll()'), 2)
        self.assertLess(step.index('Comm_BoardLinkSetChassisCommand(&stopped)'), step.index('return;'))
        self.assertLess(step.index('Control_GimbalStep();'), step.rindex('xTaskResumeAll()'))

    def test_output_and_shooter_gates_remain_present(self):
        source = (self.root / 'Modules/module_gimbal.c').read_text(encoding='utf-8')
        output = source.split('void GimbalYaw_Output(void)', 1)[1].split('///', 1)[0]
        self.assertLess(output.index('YawIdentApp_ValidateOutput()'), output.index('GimbalAxis_Flush'))
        self.assertIn('BIG_YAW_OUTPUT_INHIBIT_TEST != 0U && !allow', output)
        shooter = (self.root / 'Modules/module_shoot.c').read_text(encoding='utf-8')
        self.assertEqual(shooter.count('if (Shooter_IdentificationStop()) return;'), 3)
        self.assertIn('(void)Shooter_IdentificationStop();', shooter)

    def test_rx_crc_then_identification_then_legacy_writes(self):
        source = (self.root / 'Periphal/periph_pc_comm.c').read_text(encoding='utf-8')
        decode = source.split('void PC_Comm_DecodePacket', 1)[1]
        self.assertLess(decode.index('PC_Comm_VerifyChecksum'), decode.index('YawIdentApp_Receive'))
        self.assertLess(decode.index('YawIdentApp_OwnsControl'), decode.index('memcpy(PC_Comm_Data.raw_data_payload'))


if __name__ == '__main__':
    unittest.main()
