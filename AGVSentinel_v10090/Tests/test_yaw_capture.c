#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include "module_yaw_capture.h"

static uint8_t crc(const uint8_t *data, unsigned length)
{
    uint8_t result = 0;
    for (unsigned i = 0; i < length; ++i) {
        result ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            result = (uint8_t)((result << 1) ^ ((result & 128) ? 0x31 : 0));
    }
    return result;
}

int main(void)
{
    YawCapture_Record record = {0};
    uint8_t reconstructed[88];
    assert(sizeof(record) == 88);
    assert(offsetof(YawCapture_Record, imu_dt_s) == 20);
    assert(offsetof(YawCapture_Record, small_current_raw) == 76);
    assert(YawCapture_Age(5, UINT32_MAX - 4) == 10);
    assert(YawCapture_Age(100000, 0) == 65535);
    record.tick_ms = 100;
    record.imu_sequence = 99;
    record.imu_age_ms = record.big_age_ms = record.small_age_ms = 1;
    record.rc_yaw = -321;
    record.flags = 15;
    record.imu_dt_s = .001f;
    record.yaw_deg = -87.15f;
    record.gyro_z_rad_s = -1.25f;
    record.big_encoder_deg = 359.5f;
    record.small_encoder_deg = 300;
    record.big_command = 0;
    record.small_command = -2.5f;
    record.small_current_raw = -1234;
    for (uint8_t part = 0; part < YAW_CAPTURE_PARTS; ++part) {
        uint8_t packet[16] = {255, 0x32};
        YawCapture_Part(&record, 65535, part, packet + 2);
        assert(packet[2] == 255 && packet[3] == 255);
        assert(packet[4] == part && packet[5] == 1);
        memcpy(reconstructed + part * 8, packet + 6, 8);
        packet[14] = crc(packet, 14);
        packet[15] = 13;
        for (unsigned i = 0; i < sizeof(packet); ++i) printf("%02x", packet[i]);
    }
    assert(memcmp(reconstructed, &record, sizeof(record)) == 0);
    puts("");
    return 0;
}
