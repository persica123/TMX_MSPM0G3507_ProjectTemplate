#ifndef _BSP_JY62_H
#define _BSP_JY62_H

#include <stdint.h>
#include "board.h"

/*
 * JY61P 三维姿态测量传感器驱动。
 *
 * 说明：
 * - 当前文件保留 JY62_* API 名称，是为了兼容上层已有的直线/弧线控制代码。
 * - 底层已经按 JY61P 官方例程改为软件 I2C 寄存器读取，不再使用 UART 帧解析。
 * - 官方例程默认 SCL=PA1、SDA=PA0；本工程 PA0/PA1 已用于 TB6612 PWM，
 *   因此改用 PA10/PA11：
 *   JY61P SCL -> MSPM0 PA10
 *   JY61P SDA -> MSPM0 PA11
 */

#define JY61P_I2C_ADDR              (0x50U)
#define JY61P_REG_UNLOCK            (0x69U)
#define JY61P_REG_SAVE              (0x00U)
#define JY61P_REG_ANGLE_REFERENCE   (0x01U)
#define JY61P_REG_ROLL_L            (0x3DU)
#define JY61P_READ_ANGLE_LEN        (6U)
#define JY61P_I2C_WAIT_ACK_LIMIT    (50U)
#define JY61P_BOOT_DELAY_MS         (500U)
#define JY61P_HARDWARE_ZERO_ON_BOOT (0U)
#define JY61P_GZ_FILTER_NUM         (3)
#define JY61P_GZ_FILTER_DEN         (4)

#define JY61P_STATUS_OK             (0U)
#define JY61P_STATUS_BUS_STUCK      (1U)
#define JY61P_STATUS_ADDR_W_NACK    (2U)
#define JY61P_STATUS_REG_NACK       (3U)
#define JY61P_STATUS_ADDR_R_NACK    (4U)
#define JY61P_STATUS_ALL_ZERO       (5U)
#define JY61P_STATUS_ALL_HIGH       (6U)
#define JY61P_STATUS_BAD_ARGUMENT   (7U)

/* 兼容旧日志字段；JY61P 当前不走 UART。 */
#define JY62_UART_BAUD_RATE         (0U)

typedef struct {
    int16_t angle_raw[3];      /* Roll/Pitch/Yaw 原始角度，换算为 deg 时 raw / 32768 * 180。 */
    uint8_t update_flags;
    uint8_t received_flags;
    uint8_t last_frame_type;
    uint8_t raw_count;
    uint8_t raw_write_index;
    uint8_t i2c_status;
    uint8_t recent_raw[JY61P_READ_ANGLE_LEN];
    uint32_t header_count;
    uint32_t frame_count;
    uint32_t unknown_frame_count;
    uint32_t checksum_error;
    uint32_t rx_byte_count;
    uint32_t rx_irq_count;
    uint32_t uart_error_count;
    uint32_t overrun_count;
    uint32_t i2c_error_count;
} jy62_sample_t;

typedef struct {
    int32_t yaw_cdeg;
    int32_t yaw_relative_cdeg;
    int32_t yaw_zero_cdeg;
    int32_t gyro_z_mdps;
    int32_t gyro_z_filtered_mdps;
    int32_t roll_cdeg;
    int32_t pitch_cdeg;
    uint8_t valid;
    uint8_t update_flags;
    uint32_t rx_byte_count;
    uint32_t header_count;
    uint32_t frame_count;
    uint8_t i2c_status;
    uint32_t checksum_error;
    uint32_t uart_error_count;
    uint32_t overrun_count;
} jy62_navigation_t;

static jy62_sample_t g_jy61p_sample;
static int32_t g_jy61p_yaw_zero_cdeg;
static uint8_t g_jy61p_yaw_zero_valid;
static uint32_t g_jy61p_last_poll_count;
static int32_t g_jy61p_gyro_z_filtered_mdps;
static uint8_t g_jy61p_gyro_z_filter_valid;

static void JY61P_DriveSclLow(void)
{
    DL_GPIO_initDigitalOutput(JY61P_IIC_SCL_IOMUX);
    DL_GPIO_clearPins(JY61P_IIC_PORT, JY61P_IIC_SCL_PIN);
    DL_GPIO_enableOutput(JY61P_IIC_PORT, JY61P_IIC_SCL_PIN);
}

static void JY61P_ReleaseScl(void)
{
    DL_GPIO_disableOutput(JY61P_IIC_PORT, JY61P_IIC_SCL_PIN);
    DL_GPIO_initDigitalInput(JY61P_IIC_SCL_IOMUX);
}

static void JY61P_DriveSdaLow(void)
{
    DL_GPIO_initDigitalOutput(JY61P_IIC_SDA_IOMUX);
    DL_GPIO_clearPins(JY61P_IIC_PORT, JY61P_IIC_SDA_PIN);
    DL_GPIO_enableOutput(JY61P_IIC_PORT, JY61P_IIC_SDA_PIN);
}

static void JY61P_ReleaseSda(void)
{
    DL_GPIO_disableOutput(JY61P_IIC_PORT, JY61P_IIC_SDA_PIN);
    DL_GPIO_initDigitalInput(JY61P_IIC_SDA_IOMUX);
}

static void JY61P_WriteScl(uint8_t level)
{
    if (level != 0U) {
        JY61P_ReleaseScl();
    } else {
        JY61P_DriveSclLow();
    }
}

static void JY61P_WriteSda(uint8_t level)
{
    if (level != 0U) {
        JY61P_ReleaseSda();
    } else {
        JY61P_DriveSdaLow();
    }
}

static uint8_t JY61P_ReadSda(void)
{
    return ((DL_GPIO_readPins(JY61P_IIC_PORT, JY61P_IIC_SDA_PIN) & JY61P_IIC_SDA_PIN) != 0U) ? 1U : 0U;
}

static uint8_t JY61P_ReadScl(void)
{
    return ((DL_GPIO_readPins(JY61P_IIC_PORT, JY61P_IIC_SCL_PIN) & JY61P_IIC_SCL_PIN) != 0U) ? 1U : 0U;
}

static void JY61P_Start(void)
{
    JY61P_WriteScl(0U);
    JY61P_WriteSda(1U);
    JY61P_WriteScl(1U);
    delay_us(5);
    JY61P_WriteSda(0U);
    delay_us(5);
    JY61P_WriteScl(0U);
    delay_us(5);
}

static void JY61P_Stop(void)
{
    JY61P_WriteScl(0U);
    JY61P_WriteSda(0U);
    JY61P_WriteScl(1U);
    delay_us(5);
    JY61P_WriteSda(1U);
    delay_us(5);
}

static void JY61P_SendAck(uint8_t nack)
{
    JY61P_WriteScl(0U);
    JY61P_WriteSda((nack == 0U) ? 0U : 1U);
    delay_us(5);
    JY61P_WriteScl(1U);
    delay_us(5);
    JY61P_WriteScl(0U);
    JY61P_WriteSda(1U);
}

static uint8_t JY61P_WaitAck(void)
{
    uint8_t wait_count = JY61P_I2C_WAIT_ACK_LIMIT;
    uint8_t ack;

    JY61P_WriteSda(1U);
    delay_us(2);
    JY61P_WriteScl(1U);

    while ((JY61P_ReadScl() == 0U) && (wait_count != 0U)) {
        wait_count--;
        delay_us(2);
    }

    if (wait_count == 0U) {
        JY61P_Stop();
        return 0U;
    }

    delay_us(5);
    ack = (JY61P_ReadSda() == 0U) ? 1U : 0U;
    JY61P_WriteScl(0U);

    if (ack == 0U) {
        JY61P_Stop();
    }

    return ack;
}

static uint8_t JY61P_EnsureBusIdle(void)
{
    uint8_t pulse;

    JY61P_WriteSda(1U);
    JY61P_WriteScl(1U);
    delay_us(10);
    if ((JY61P_ReadScl() != 0U) && (JY61P_ReadSda() != 0U)) {
        return 1U;
    }

    JY61P_WriteSda(1U);
    for (pulse = 0U; pulse < 9U; pulse++) {
        JY61P_WriteScl(0U);
        delay_us(5);
        JY61P_WriteScl(1U);
        delay_us(5);
    }
    JY61P_Stop();

    return ((JY61P_ReadScl() != 0U) && (JY61P_ReadSda() != 0U)) ? 1U : 0U;
}

static void JY61P_SendByte(uint8_t data)
{
    uint8_t index;

    JY61P_WriteScl(0U);

    for (index = 0U; index < 8U; index++) {
        JY61P_WriteSda((uint8_t)((data & 0x80U) >> 7U));
        delay_us(2);
        JY61P_WriteScl(1U);
        delay_us(5);
        JY61P_WriteScl(0U);
        delay_us(5);
        data <<= 1U;
    }
}

static uint8_t JY61P_ReadByte(void)
{
    uint8_t index;
    uint8_t data = 0U;

    JY61P_WriteSda(1U);

    for (index = 0U; index < 8U; index++) {
        JY61P_WriteScl(0U);
        delay_us(5);
        JY61P_WriteScl(1U);
        delay_us(5);
        data <<= 1U;
        if (JY61P_ReadSda() != 0U) {
            data |= 1U;
        }
        delay_us(5);
    }

    return data;
}

static uint8_t JY61P_WriteData(uint8_t reg, const uint8_t *data, uint32_t length)
{
    uint32_t index;

    JY61P_Start();
    JY61P_SendByte((uint8_t)(JY61P_I2C_ADDR << 1U));
    if (JY61P_WaitAck() == 0U) {
        return 0U;
    }

    JY61P_SendByte(reg);
    if (JY61P_WaitAck() == 0U) {
        return 0U;
    }

    for (index = 0U; index < length; index++) {
        JY61P_SendByte(data[index]);
        if (JY61P_WaitAck() == 0U) {
            return 0U;
        }
    }

    JY61P_Stop();
    return 1U;
}

static uint8_t JY61P_ReadData(uint8_t reg, uint8_t *data, uint32_t length)
{
    uint32_t index;

    if ((data == 0) || (length == 0U)) {
        g_jy61p_sample.i2c_status = JY61P_STATUS_BAD_ARGUMENT;
        return 0U;
    }

    if (JY61P_EnsureBusIdle() == 0U) {
        g_jy61p_sample.i2c_status = JY61P_STATUS_BUS_STUCK;
        return 0U;
    }

    JY61P_Start();
    JY61P_SendByte((uint8_t)(JY61P_I2C_ADDR << 1U));
    if (JY61P_WaitAck() == 0U) {
        g_jy61p_sample.i2c_status = JY61P_STATUS_ADDR_W_NACK;
        return 0U;
    }

    JY61P_SendByte(reg);
    if (JY61P_WaitAck() == 0U) {
        g_jy61p_sample.i2c_status = JY61P_STATUS_REG_NACK;
        return 0U;
    }

    delay_us(5);
    JY61P_Start();
    JY61P_SendByte((uint8_t)((JY61P_I2C_ADDR << 1U) | 1U));
    if (JY61P_WaitAck() == 0U) {
        g_jy61p_sample.i2c_status = JY61P_STATUS_ADDR_R_NACK;
        return 0U;
    }

    for (index = 0U; index < length; index++) {
        data[index] = JY61P_ReadByte();
        JY61P_SendAck((index == (length - 1U)) ? 1U : 0U);
    }

    JY61P_Stop();
    g_jy61p_sample.i2c_status = JY61P_STATUS_OK;
    return 1U;
}

static int16_t JY61P_MakeI16(uint8_t low, uint8_t high)
{
    return (int16_t)((uint16_t)low | ((uint16_t)high << 8));
}

static int32_t JY62_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t JY62_RawToAngleCdeg(int16_t raw)
{
    return (int32_t)(((int64_t)raw * 18000LL) / 32768LL);
}

static int32_t JY62_NormalizeAngleCdeg(int32_t angle_cdeg)
{
    while (angle_cdeg > 18000L) {
        angle_cdeg -= 36000L;
    }
    while (angle_cdeg < -18000L) {
        angle_cdeg += 36000L;
    }
    return angle_cdeg;
}

static void JY62_PrintSignedFixed(int32_t value, uint16_t scale, uint8_t digits)
{
    int32_t abs_value = JY62_Abs32(value);
    int32_t integer = abs_value / scale;
    int32_t decimal = abs_value % scale;

    if (value < 0) {
        lc_printf("-");
    }

    if (digits == 3U) {
        lc_printf("%ld.%03ld", integer, decimal);
    } else {
        lc_printf("%ld.%02ld", integer, decimal);
    }
}

static uint8_t JY61P_ReadAngles(void)
{
    uint8_t data[JY61P_READ_ANGLE_LEN] = {0U};
    uint8_t ok;
    uint8_t all_zero = 1U;
    uint8_t all_high = 1U;
    int32_t yaw_now_cdeg;
    int32_t yaw_delta_cdeg;
    int32_t gyro_est_mdps = 0;
    static int32_t last_yaw_cdeg;
    static uint8_t last_yaw_valid;

    ok = JY61P_ReadData(JY61P_REG_ROLL_L, data, JY61P_READ_ANGLE_LEN);
    if (ok == 0U) {
        g_jy61p_sample.i2c_error_count++;
        return 0U;
    }

    for (uint8_t index = 0U; index < JY61P_READ_ANGLE_LEN; index++) {
        if (data[index] != 0U) {
            all_zero = 0U;
        }
        if (data[index] != 0xFFU) {
            all_high = 0U;
        }
    }

    if (all_zero != 0U) {
        g_jy61p_sample.i2c_status = JY61P_STATUS_ALL_ZERO;
        g_jy61p_sample.i2c_error_count++;
        return 0U;
    }
    if (all_high != 0U) {
        g_jy61p_sample.i2c_status = JY61P_STATUS_ALL_HIGH;
        g_jy61p_sample.i2c_error_count++;
        return 0U;
    }

    g_jy61p_sample.angle_raw[0] = JY61P_MakeI16(data[0], data[1]);
    g_jy61p_sample.angle_raw[1] = JY61P_MakeI16(data[2], data[3]);
    g_jy61p_sample.angle_raw[2] = JY61P_MakeI16(data[4], data[5]);
    g_jy61p_sample.update_flags = 0x04U;
    g_jy61p_sample.received_flags = 0x04U;
    g_jy61p_sample.last_frame_type = 0x53U;
    g_jy61p_sample.frame_count++;
    g_jy61p_sample.rx_byte_count += JY61P_READ_ANGLE_LEN;
    g_jy61p_sample.raw_count = JY61P_READ_ANGLE_LEN;
    g_jy61p_sample.raw_write_index = 0U;

    for (uint8_t index = 0U; index < JY61P_READ_ANGLE_LEN; index++) {
        g_jy61p_sample.recent_raw[index] = data[index];
    }

    yaw_now_cdeg = JY62_RawToAngleCdeg(g_jy61p_sample.angle_raw[2]);
    if (last_yaw_valid != 0U) {
        yaw_delta_cdeg = JY62_NormalizeAngleCdeg(yaw_now_cdeg - last_yaw_cdeg);
        gyro_est_mdps = yaw_delta_cdeg * 1000L;
    }
    last_yaw_cdeg = yaw_now_cdeg;
    last_yaw_valid = 1U;

    if (g_jy61p_gyro_z_filter_valid == 0U) {
        g_jy61p_gyro_z_filtered_mdps = gyro_est_mdps;
        g_jy61p_gyro_z_filter_valid = 1U;
    } else {
        g_jy61p_gyro_z_filtered_mdps =
            ((g_jy61p_gyro_z_filtered_mdps * JY61P_GZ_FILTER_NUM) +
            (gyro_est_mdps * (JY61P_GZ_FILTER_DEN - JY61P_GZ_FILTER_NUM))) /
            JY61P_GZ_FILTER_DEN;
    }

    if (g_jy61p_yaw_zero_valid == 0U) {
        g_jy61p_yaw_zero_cdeg = yaw_now_cdeg;
        g_jy61p_yaw_zero_valid = 1U;
    }

    return 1U;
}

#if JY61P_HARDWARE_ZERO_ON_BOOT
static void JY61P_ZeroHardware(void)
{
    const uint8_t unlock_reg[2] = {0x88U, 0xB5U};
    const uint8_t z_axis_zero_reg[2] = {0x04U, 0x00U};
    const uint8_t angle_zero_reg[2] = {0x08U, 0x00U};
    const uint8_t save_reg[2] = {0x00U, 0x00U};

    (void)JY61P_WriteData(JY61P_REG_UNLOCK, unlock_reg, 2U);
    delay_ms(200);
    (void)JY61P_WriteData(JY61P_REG_ANGLE_REFERENCE, z_axis_zero_reg, 2U);
    delay_ms(200);
    (void)JY61P_WriteData(JY61P_REG_SAVE, save_reg, 2U);
    delay_ms(200);

    (void)JY61P_WriteData(JY61P_REG_UNLOCK, unlock_reg, 2U);
    delay_ms(200);
    (void)JY61P_WriteData(JY61P_REG_ANGLE_REFERENCE, angle_zero_reg, 2U);
    delay_ms(200);
    (void)JY61P_WriteData(JY61P_REG_SAVE, save_reg, 2U);
    delay_ms(200);
}
#endif

static void JY62_Init(void)
{
    for (uint8_t index = 0U; index < 3U; index++) {
        g_jy61p_sample.angle_raw[index] = 0;
    }
    for (uint8_t index = 0U; index < JY61P_READ_ANGLE_LEN; index++) {
        g_jy61p_sample.recent_raw[index] = 0U;
    }

    g_jy61p_sample.update_flags = 0U;
    g_jy61p_sample.received_flags = 0U;
    g_jy61p_sample.last_frame_type = 0U;
    g_jy61p_sample.raw_count = 0U;
    g_jy61p_sample.raw_write_index = 0U;
    g_jy61p_sample.i2c_status = JY61P_STATUS_OK;
    g_jy61p_sample.header_count = 0U;
    g_jy61p_sample.frame_count = 0U;
    g_jy61p_sample.unknown_frame_count = 0U;
    g_jy61p_sample.checksum_error = 0U;
    g_jy61p_sample.rx_byte_count = 0U;
    g_jy61p_sample.rx_irq_count = 0U;
    g_jy61p_sample.uart_error_count = 0U;
    g_jy61p_sample.overrun_count = 0U;
    g_jy61p_sample.i2c_error_count = 0U;
    g_jy61p_yaw_zero_cdeg = 0;
    g_jy61p_yaw_zero_valid = 0U;
    g_jy61p_last_poll_count = 0U;
    g_jy61p_gyro_z_filtered_mdps = 0;
    g_jy61p_gyro_z_filter_valid = 0U;

    JY61P_WriteScl(1U);
    JY61P_WriteSda(1U);
    delay_ms(JY61P_BOOT_DELAY_MS);
#if JY61P_HARDWARE_ZERO_ON_BOOT
    JY61P_ZeroHardware();
#endif
    (void)JY61P_ReadAngles();
}

static void JY62_SetYawZeroToCurrent(void)
{
    if (JY61P_ReadAngles() != 0U) {
        g_jy61p_yaw_zero_cdeg = JY62_RawToAngleCdeg(g_jy61p_sample.angle_raw[2]);
        g_jy61p_yaw_zero_valid = 1U;
    }
}

static uint32_t JY62_GetNavigation(jy62_navigation_t *nav)
{
    uint32_t before = g_jy61p_sample.frame_count;
    uint32_t frame_delta;
    uint8_t ok;
    int32_t yaw_cdeg;

    ok = JY61P_ReadAngles();
    frame_delta = g_jy61p_sample.frame_count - g_jy61p_last_poll_count;
    g_jy61p_last_poll_count = g_jy61p_sample.frame_count;

    if (nav != 0) {
        yaw_cdeg = JY62_RawToAngleCdeg(g_jy61p_sample.angle_raw[2]);
        nav->yaw_cdeg = yaw_cdeg;
        nav->yaw_zero_cdeg = g_jy61p_yaw_zero_cdeg;
        nav->yaw_relative_cdeg = JY62_NormalizeAngleCdeg(yaw_cdeg - g_jy61p_yaw_zero_cdeg);
        nav->gyro_z_mdps = g_jy61p_gyro_z_filtered_mdps;
        nav->gyro_z_filtered_mdps = g_jy61p_gyro_z_filtered_mdps;
        nav->roll_cdeg = JY62_RawToAngleCdeg(g_jy61p_sample.angle_raw[0]);
        nav->pitch_cdeg = JY62_RawToAngleCdeg(g_jy61p_sample.angle_raw[1]);
        nav->valid = ((ok != 0U) && (g_jy61p_yaw_zero_valid != 0U)) ? 1U : 0U;
        nav->update_flags = g_jy61p_sample.update_flags;
        nav->rx_byte_count = g_jy61p_sample.rx_byte_count;
        nav->header_count = g_jy61p_sample.header_count;
        nav->frame_count = g_jy61p_sample.frame_count;
        nav->i2c_status = g_jy61p_sample.i2c_status;
        nav->checksum_error = g_jy61p_sample.i2c_error_count;
        nav->uart_error_count = 0U;
        nav->overrun_count = 0U;
    }

    return (g_jy61p_sample.frame_count != before) ? frame_delta : 0U;
}

static uint8_t JY62_PeekNavigation(jy62_navigation_t *nav)
{
    (void)JY62_GetNavigation(nav);
    return (nav != 0) ? nav->valid : 0U;
}

static void JY62_PrintSample(const jy62_sample_t *sample)
{
    int32_t roll = JY62_RawToAngleCdeg(sample->angle_raw[0]);
    int32_t pitch = JY62_RawToAngleCdeg(sample->angle_raw[1]);
    int32_t yaw = JY62_RawToAngleCdeg(sample->angle_raw[2]);

    lc_printf("jy61p angle_cdeg=%ld,%ld,%ld yaw=", roll, pitch, yaw);
    JY62_PrintSignedFixed(yaw, 100U, 2U);
}

static void JY62_PrintNavigation(const jy62_navigation_t *nav)
{
    lc_printf("ok=%u yaw=", nav->valid);
    JY62_PrintSignedFixed(nav->yaw_cdeg, 100U, 2U);
    lc_printf(" rel=");
    JY62_PrintSignedFixed(nav->yaw_relative_cdeg, 100U, 2U);
    lc_printf(" gzlp=");
    JY62_PrintSignedFixed(nav->gyro_z_filtered_mdps, 1000U, 3U);
}

static void JY62_PrintRecentRaw(const jy62_sample_t *sample)
{
    for (uint8_t index = 0U; index < sample->raw_count; index++) {
        if (index != 0U) {
            lc_printf(" ");
        }
        lc_printf("%02X", sample->recent_raw[index]);
    }
}

#endif /* _BSP_JY62_H */
