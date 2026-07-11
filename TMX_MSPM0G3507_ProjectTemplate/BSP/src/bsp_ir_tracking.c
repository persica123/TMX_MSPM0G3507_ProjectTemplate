#include "bsp_ir_tracking.h"

/* 最近一次有效线位置。丢线时继续返回它，方便后续按最后方向低速找线。 */
static int16_t g_ir_tracking_last_error;

static void IRTracking_WritePin(uint32_t pin, uint8_t level)
{
    if (level != 0U) {
        DL_GPIO_setPins(IR_TRACKING_PORT, pin);
    } else {
        DL_GPIO_clearPins(IR_TRACKING_PORT, pin);
    }
}

static uint8_t IRTracking_ReadOut(void)
{
    return (DL_GPIO_readPins(IR_TRACKING_PORT, IR_TRACKING_OUT_PIN) != 0U) ? 1U : 0U;
}

static void IRTracking_SelectChannel(uint8_t channel)
{
    IRTracking_WritePin(IR_TRACKING_AD0_PIN, (uint8_t)((channel >> 0U) & 0x01U));
    IRTracking_WritePin(IR_TRACKING_AD1_PIN, (uint8_t)((channel >> 1U) & 0x01U));
    IRTracking_WritePin(IR_TRACKING_AD2_PIN, (uint8_t)((channel >> 2U) & 0x01U));
}

static int16_t IRTracking_CalculatePosition(uint8_t line_mask, uint8_t *active_count)
{
    static const int16_t weights[IR_TRACKING_SENSOR_COUNT] = {
        -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
    };
    int32_t weighted_sum = 0;
    uint8_t count = 0U;
    uint8_t index;

    for (index = 0U; index < IR_TRACKING_SENSOR_COUNT; index++) {
        if ((line_mask & (1U << index)) != 0U) {
            weighted_sum += weights[index];
            count++;
        }
    }

    *active_count = count;

    if (count == 0U) {
        return g_ir_tracking_last_error;
    }

    return (int16_t)(weighted_sum / (int32_t)count);
}

void IRTracking_Init(void)
{
    g_ir_tracking_last_error = 0;
    IRTracking_SelectChannel(0U);
}

uint8_t IRTracking_ReadRaw(uint8_t *raw)
{
    uint8_t channel;
    uint8_t value = 0U;

    if (raw == 0) {
        return 0U;
    }

    for (channel = 0U; channel < IR_TRACKING_SENSOR_COUNT; channel++) {
        IRTracking_SelectChannel(channel);
        delay_us(IR_TRACKING_SELECT_DELAY_US);

        /*
         * raw 保持原工程约定：bit7=X1，bit0=X8。
         * line_mask 再转换成 bit0=X1，bit7=X8，方便上层按左右位置判断。
         */
        if (IRTracking_ReadOut() != 0U) {
            value |= (uint8_t)(1U << (7U - channel));
        }
    }

    *raw = value;
    return 1U;
}

uint8_t IRTracking_RawToLineMask(uint8_t raw)
{
    uint8_t line_mask = 0U;
    uint8_t index;

    for (index = 0U; index < IR_TRACKING_SENSOR_COUNT; index++) {
        uint8_t raw_bit = (uint8_t)((raw >> (7U - index)) & 0x01U);

        if (raw_bit == (uint8_t)IR_TRACKING_BLACK_LEVEL) {
            line_mask |= (uint8_t)(1U << index);
        }
    }

    return line_mask;
}

uint8_t IRTracking_ReadSample(ir_tracking_sample_t *sample)
{
    uint8_t raw;
    uint8_t line_mask;
    uint8_t active_count;
    int16_t position;

    if (sample == 0) {
        return 0U;
    }

    if (IRTracking_ReadRaw(&raw) == 0U) {
        return 0U;
    }

    line_mask = IRTracking_RawToLineMask(raw);
    position = IRTracking_CalculatePosition(line_mask, &active_count);

    sample->raw = raw;
    sample->line_mask = line_mask;
    sample->active_count = active_count;
    sample->line_lost = (active_count == 0U) ? 1U : 0U;
    sample->position = position;
    sample->error = position;

    if (active_count != 0U) {
        g_ir_tracking_last_error = position;
    }

    return 1U;
}

int16_t IRTracking_GetLastError(void)
{
    return g_ir_tracking_last_error;
}

void IRTracking_PrintSample(const ir_tracking_sample_t *sample)
{
    if (sample == 0) {
        return;
    }

    lc_printf("IR raw=0x%02X mask=0x%02X count=%u lost=%u pos=%d err=%d black=%u\r\n",
        sample->raw,
        sample->line_mask,
        sample->active_count,
        sample->line_lost,
        sample->position,
        sample->error,
        (uint8_t)IR_TRACKING_BLACK_LEVEL);
}
