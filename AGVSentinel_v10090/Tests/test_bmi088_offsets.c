#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Compile the actual driver with register-level hardware substitutes. */
#define SPI_UTIL_H
#define GPIO_UTIL_H
#define MATH_ALG_H
#define DELAY_LIB_H
#define BUFF_LIB_H
#define FLASH_UTIL_H
#define DWT_LIB_H
typedef struct { int unused; } SPI_HandleTypeDef;
static SPI_HandleTypeDef hspi1;
static uint32_t __get_PRIMASK(void) { return 0U; }
static void __disable_irq(void) {}
static void __set_PRIMASK(uint32_t value) { (void)value; }
static double seconds;
static uint32_t HAL_GetTick(void) { return (uint32_t)(seconds * 1000.0); }
static void HAL_Delay(uint32_t ms) { seconds += ms / 1000.0; }
static float DWT_GetTimeline_s(void) { return (float)seconds; }
static void DWT_Delay(float dt) { seconds += dt; }

#include "periph_bmi088.h"

static uint8_t registers[2][256];
static void write_reg(unsigned sensor, uint8_t reg, uint8_t value)
{
    registers[sensor][reg] = value;
}
static uint8_t read_reg(unsigned sensor, uint8_t reg)
{
    if (sensor == 0 && reg == BMI088_ACC_CHIP_ID)
        return BMI088_ACC_CHIP_ID_VALUE;
    if (sensor == 1 && reg == BMI088_GYRO_CHIP_ID)
        return BMI088_GYRO_CHIP_ID_VALUE;
    if (sensor == 1 && reg == BMI088_GYRO_SELF_TEST)
        return BMI088_GYRO_BIST_RDY;
    return registers[sensor][reg];
}
static void pack_i16(uint8_t *dst, int16_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)((uint16_t)value >> 8);
}
static void read_many(unsigned sensor, uint8_t reg, uint8_t *dst, unsigned len)
{
    memset(dst, 0, len);
    if (sensor == 0 && reg == BMI088_ACCEL_XOUT_L) {
        uint8_t self_test = registers[0][BMI088_ACC_SELF_TEST];
        if (self_test != BMI088_ACC_SELF_TEST_OFF) {
            int16_t value = self_test ==
                write_BMI088_ACCEL_self_test_Reg_Data_Error[4][1] ? 2000 : -2000;
            pack_i16(dst, value);
            pack_i16(dst + 2, value);
            pack_i16(dst + 4, value);
        } else {
            pack_i16(dst + 4, 5450);
        }
    } else if (sensor == 1 && reg == BMI088_GYRO_CHIP_ID) {
        dst[0] = BMI088_GYRO_CHIP_ID_VALUE;
        pack_i16(dst + 2, 3);
        pack_i16(dst + 4, -2);
        pack_i16(dst + 6, 1);
    }
}

#undef BMI088_ACCEL_WRITE_SINGLE_REG
#undef BMI088_GYRO_WRITE_SINGLE_REG
#undef BMI088_ACCEL_READ_SINGLE_REG
#undef BMI088_GYRO_READ_SINGLE_REG
#undef BMI088_ACCEL_READ_MULI_REG
#undef BMI088_GYRO_READ_MULI_REG
#define BMI088_ACCEL_WRITE_SINGLE_REG(r, v) write_reg(0, (r), (v))
#define BMI088_GYRO_WRITE_SINGLE_REG(r, v) write_reg(1, (r), (v))
#define BMI088_ACCEL_READ_SINGLE_REG(r, v) ((v) = read_reg(0, (r)))
#define BMI088_GYRO_READ_SINGLE_REG(r, v) ((v) = read_reg(1, (r)))
#define BMI088_ACCEL_READ_MULI_REG(r, p, n) read_many(0, (r), (p), (n))
#define BMI088_GYRO_READ_MULI_REG(r, p, n) read_many(1, (r), (p), (n))

typedef struct { unsigned sensor; } GPIO_GPIOTypeDef;
static GPIO_GPIOTypeDef accel_cs = {0}, gyro_cs = {1};
static GPIO_GPIOTypeDef *CS_ACCEL = &accel_cs, *CS_GYRO = &gyro_cs;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
static GPIO_GPIOTypeDef *selected;
static unsigned transfers, fail_transfer;
static unsigned byte_index, fail_byte;
static HAL_StatusTypeDef failure_status;
static int bad_chip = -1;
static void GPIO_Reset(GPIO_GPIOTypeDef *cs) {
    assert(!selected); selected = cs; byte_index = 0; ++transfers;
}
static void GPIO_Set(GPIO_GPIOTypeDef *cs) { assert(selected == cs); selected = NULL; }
static HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi,
    uint8_t *tx, uint8_t *rx, uint16_t length, uint32_t timeout)
{
    static uint8_t reg, data[8];
    unsigned prefix = selected->sensor == 0 ? 2U : 1U;
    assert(spi == &hspi1 && selected && timeout == 10U && length == 1U);
    if (transfers == fail_transfer && byte_index == fail_byte) return failure_status;
    *rx = 0;
    if (byte_index == 0) {
        reg = *tx & 0x7FU;
        assert(*tx & 0x80U);
        memset(data, 0, sizeof(data));
        if (selected->sensor == 0 && reg == BMI088_ACC_CHIP_ID)
            data[0] = BMI088_ACC_CHIP_ID_VALUE;
        else
            read_many(selected->sensor, reg, data, sizeof(data));
        if ((int)selected->sensor == bad_chip && reg == 0U) data[0] = 0;
    } else if (byte_index < prefix) {
        assert(*tx == (reg | 0x80U));
    } else {
        assert(*tx == 0x55U && byte_index - prefix < sizeof(data));
        *rx = data[byte_index - prefix];
    }
    ++byte_index;
    return HAL_OK;
}

#include "../AGVSentinel_gimbal/Src/Periphal/periph_bmi088.c"

static void check_defaults(void)
{
    assert(BMI088_BMI088Data.gyro_offset[0] == BMI088_GxOFFSET);
    assert(BMI088_BMI088Data.gyro_offset[1] == BMI088_GyOFFSET);
    assert(BMI088_BMI088Data.gyro_offset[2] == BMI088_GzOFFSET);
    assert(BMI088_BMI088Data.gNorm == BMI088_gNORM);
    assert(fabsf(BMI088_BMI088Data.accelScale - 9.81f / BMI088_gNORM) < 1e-6f);
}

int main(void)
{
    const unsigned patterns[] = {0, 0x55, 0xAA, 0xFF};
    unsigned i;
    for (i = 0; i < sizeof(patterns) / sizeof(patterns[0]); ++i) {
        memset(&BMI088_BMI088Data, patterns[i], sizeof(BMI088_BMI088Data));
        BMI088_GetOffset();
        check_defaults();
    }

    assert(BMI088_Init(0) == BMI088_NO_ERROR);
    check_defaults();
    assert(BMI088_Init(1) == BMI088_NO_ERROR);
    assert(fabsf(BMI088_BMI088Data.gyro_offset[0] - 3 * BMI088_GYRO_SEN) < 1e-6f);
    assert(fabsf(BMI088_BMI088Data.gyro_offset[1] + 2 * BMI088_GYRO_SEN) < 1e-6f);
    assert(fabsf(BMI088_BMI088Data.gyro_offset[2] - BMI088_GYRO_SEN) < 1e-6f);
    assert(fabsf(BMI088_BMI088Data.gNorm - 5450 * BMI088_ACCEL_SEN) < 0.002f);
    assert(fabsf(BMI088_BMI088Data.accelScale * BMI088_BMI088Data.gNorm - 9.81f) < 1e-5f);

    /* No calibration requested on the next boot: defaults, not stale RAM. */
    assert(BMI088_Init(0) == BMI088_NO_ERROR);
    check_defaults();
    seconds += 1;
    BMI088_BMI088DecodeData();
    assert(BMI088_BMI088Data.state == BMI088_STATE_CONNECTED);
    assert(BMI088_BMI088Data.last_update_time == HAL_GetTick());
    for (unsigned status = HAL_ERROR; status <= HAL_TIMEOUT; ++status) {
        for (unsigned transfer = 1; transfer <= 4; ++transfer) {
            BMI088_BMI088DataTypeDef previous = BMI088_BMI088Data;
            transfers = 0;
            fail_transfer = transfer;
            failure_status = (HAL_StatusTypeDef)status;
            seconds += 1;
            BMI088_BMI088DecodeData();
            assert(!selected);
            assert(BMI088_BMI088Data.state == BMI088_STATE_ERROR);
            assert(BMI088_BMI088Data.last_update_time == previous.last_update_time);
            assert(memcmp(&BMI088_BMI088Data.accel, &previous.accel, sizeof(previous.accel)) == 0);
            assert(memcmp(&BMI088_BMI088Data.gyro, &previous.gyro, sizeof(previous.gyro)) == 0);
        }
    }
    fail_transfer = 0;
    for (fail_byte = 0; fail_byte < 9; ++fail_byte) {
        uint32_t stamp = BMI088_BMI088Data.last_update_time;
        transfers = 0;
        fail_transfer = 3;
        failure_status = HAL_TIMEOUT;
        BMI088_BMI088DecodeData();
        assert(!selected && BMI088_BMI088Data.state == BMI088_STATE_ERROR);
        assert(BMI088_BMI088Data.last_update_time == stamp);
        assert(BMI088_ReadDiag.failed_stage == 3U);
        assert(BMI088_ReadDiag.hal_status == HAL_TIMEOUT);
    }
    fail_transfer = fail_byte = 0;
    for (bad_chip = 0; bad_chip < 2; ++bad_chip) {
        uint32_t stamp = BMI088_BMI088Data.last_update_time;
        seconds += 1;
        BMI088_BMI088DecodeData();
        assert(!selected && BMI088_BMI088Data.state == BMI088_STATE_ERROR);
        assert(BMI088_BMI088Data.last_update_time == stamp);
    }
    bad_chip = -1;
    BMI088_BMI088DecodeData();
    assert(BMI088_BMI088Data.state == BMI088_STATE_CONNECTED);
    assert(BMI088_BMI088Data.last_update_time == HAL_GetTick());
    puts("PASS: BMI088 offsets, checked SPI failures, chip IDs and recovery");
    return 0;
}
