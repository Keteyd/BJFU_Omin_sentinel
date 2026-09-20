"""Offline reference preview only. No serial, ARM packets or firmware output."""

import argparse
import csv
import json
import math
from pathlib import Path


DEFAULT_PLAN = Path(__file__).with_name('slow_excitation_review.json')


def validate(plan):
    if (plan['schema_version'] != 1 or plan['hardware_execution_authorized'] is not False
            or plan['status'] != 'DRAFT_OFFLINE_ONLY_NOT_FIRMWARE_CONFIGURATION'):
        raise ValueError('preview requires an unauthorized draft plan')
    times, targets = plan['knots_ms'], plan['normalized_targets']
    period, duration = plan['sample_period_ms'], plan['duration_ms']
    if (type(period) is not int or period <= 0 or type(duration) is not int
            or duration <= 0 or duration % period):
        raise ValueError('invalid sampling schedule')
    if (len(times) != len(targets) or len(times) < 2 or times[0] != 0
            or times[-1] != duration or targets[0] != 0 or targets[-1] != 0
            or any(type(t) is not int or t % period for t in times)
            or any(b <= a for a, b in zip(times, times[1:]))):
        raise ValueError('invalid waveform knots')
    if not all(math.isfinite(y) and abs(y) <= 1 for y in targets):
        raise ValueError('invalid normalized target')
    if max(targets) != 1 or min(targets) != -1:
        raise ValueError('profile must reach both declared peaks')
    for axis in ('big', 'small'):
        amplitude = plan['peak_target_deg'][axis]
        if not math.isfinite(amplitude) or amplitude <= 0:
            raise ValueError('invalid target amplitude')
    start = plan['small_joint_start_abs_max_deg']
    travel = plan['proposed_observed_travel_abort_deg']['small_joint']
    guard = plan['small_soft_guard_deg']
    low, high = plan['small_soft_min_deg'], plan['small_soft_max_deg']
    if (not all(math.isfinite(x) for x in (start, travel, guard, low, high))
            or start < 0 or travel <= 0 or guard < 0 or low >= high
            or -start-travel < low+guard or start+travel > high-guard):
        raise ValueError('proposed small-joint corridor exceeds guarded soft limits')
    # A nominal fixed-base geometric check, not a guarantee under dynamic coupling.
    if max(plan['peak_target_deg'].values()) >= travel:
        raise ValueError('no nominal joint-travel reserve beyond target amplitude')


def reference(plan, time_ms, axis, first_sign=1):
    if axis not in ('big', 'small') or first_sign not in (-1, 1):
        raise ValueError('invalid axis or direction')
    if not math.isfinite(time_ms) or not 0 <= time_ms <= plan['duration_ms']:
        raise ValueError('time outside profile')
    times, targets = plan['knots_ms'], plan['normalized_targets']
    if time_ms == times[-1]:
        return 0., 0., 0.
    index = next(i for i in range(len(times)-1) if time_ms < times[i+1])
    seconds = (times[index+1]-times[index])*.001
    s = (time_ms-times[index]) / (times[index+1]-times[index])
    amplitude = first_sign*plan['peak_target_deg'][axis]
    delta = amplitude*(targets[index+1]-targets[index])
    h = s**3*(10+s*(-15+6*s))
    dh = 30*s**2*(1-s)**2
    ddh = 60*s*(1-s)*(1-2*s)
    return amplitude*targets[index]+delta*h, delta*dh/seconds, delta*ddh/seconds**2


def audit(plan):
    validate(plan)
    count = plan['duration_ms']//plan['sample_period_ms']+1
    wire_bytes_s = 1000/plan['sample_period_ms']*plan['existing_frame_bytes']*plan['existing_frames_per_record']
    limits = {}
    for axis in ('big', 'small'):
        rates, accelerations = [], []
        for i in range(len(plan['knots_ms'])-1):
            dt = (plan['knots_ms'][i+1]-plan['knots_ms'][i])*.001
            delta = abs(plan['normalized_targets'][i+1]-plan['normalized_targets'][i])*plan['peak_target_deg'][axis]
            rates.append(1.875*delta/dt)
            accelerations.append(10/math.sqrt(3)*delta/dt**2)
        limits[axis] = dict(peak_target_deg=plan['peak_target_deg'][axis],
                            max_target_rate_dps=max(rates), max_target_acceleration_dps2=max(accelerations))
    return dict(status='DRAFT_REFERENCE_PREVIEW_ONLY', hardware_execution_authorized=False,
                axes=limits, records=count,
                full_capture_bytes=count*plan['existing_record_bytes'],
                existing_capture_bytes=plan['existing_buffer_records']*plan['existing_record_bytes'],
                sample_wire_bytes_per_second=wire_bytes_s,
                existing_8n1_uart_sample_utilization=10*wire_bytes_s/plan['existing_uart_baud'],
                proposed_8n1_uart_sample_utilization=10*wire_bytes_s/plan['proposed_uart_baud_for_bench_validation'],
                warnings=['Target trajectories are not predicted measured motion.',
                          'Reference derivative is not a motor speed/output limit.',
                          'Nominal joint corridor does not establish dynamic safety or stopping distance.',
                          'Wire utilization excludes status/metadata and scheduler/DMA overhead.',
                          'Current firmware, live CLI and offline ARX loader do not accept this profile.'],
                blockers=plan['required_before_execution'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, default=DEFAULT_PLAN)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    plan = json.loads(args.plan.read_text(encoding='utf-8'))
    report = audit(plan)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'review.json').write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    (args.output/'plan.json').write_text(json.dumps(plan, indent=2, allow_nan=False), encoding='utf-8')
    with (args.output/'reference.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['time_s', 'big_target_deg', 'big_target_rate_dps', 'big_target_accel_dps2',
                         'small_target_deg', 'small_target_rate_dps', 'small_target_accel_dps2'])
        for t in range(0, plan['duration_ms']+1, plan['sample_period_ms']):
            # Separate proposed trials, not simultaneous two-axis commands.
            writer.writerow([t*.001, *reference(plan, t, 'big'), *reference(plan, t, 'small')])
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
