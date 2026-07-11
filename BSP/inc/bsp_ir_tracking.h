#ifndef _BSP_IR_TRACKING_H
#define _BSP_IR_TRACKING_H

#include <stdint.h>
#include "board.h"

/*
 * 八路灰度/红外循迹模块驱动。
 *
 * 当前模块资料给出的 MSPM0 读取方式不是 I2C，而是四线 GPIO 复用读取：
 * - AD0、AD1、AD2 选择当前通道，AD0 为最低位，AD2 为最高位。
 * - OUT 输出当前通道数字量。
 * - 通道 0..7 对应 X1..X8，从车头视角左到右排列。
 *
 * 本驱动仍对上层输出统一的 line_mask：
 * - bit0 对应 X1，bit7 对应 X8。
 * - 1 表示该路检测到黑线。
 */
#define IR_TRACKING_SENSOR_COUNT      (8U)
#define IR_TRACKING_POSITION_SCALE    (1000)
#define IR_TRACKING_SELECT_DELAY_US   (50)

/*
 * 厂家巡线例程中 ACTIVE_LEVEL=1 表示检测到黑线。
 * 如果你实测串口日志里白底 mask=0xFF、黑线 mask=0x00，把这里改成 0U。
 */
#ifndef IR_TRACKING_BLACK_LEVEL
#define IR_TRACKING_BLACK_LEVEL       (1U)
#endif

typedef struct {
    uint8_t raw;          /* 原始 OUT 采样：bit7=X1，bit0=X8，位值为 OUT 直接电平。 */
    uint8_t line_mask;    /* 归一化黑线掩码：bit0=X1，bit7=X8，1 表示该路检测到黑线。 */
    uint8_t active_count; /* 当前检测到黑线的探头数量。 */
    uint8_t line_lost;    /* 1 表示 8 路都没有检测到黑线。 */
    int16_t position;     /* 加权线位置，中心为 0，左负右正。 */
    int16_t error;        /* 给循迹控制器使用的误差，当前等于 position。 */
} ir_tracking_sample_t;

/** 复位驱动内部状态；GPIO 初始化由 SysConfig 负责。 */
void IRTracking_Init(void);

/** 读取一次 8 路 OUT 原始数字状态。 */
uint8_t IRTracking_ReadRaw(uint8_t *raw);

/** 读取并解析一次完整灰度循迹采样。 */
uint8_t IRTracking_ReadSample(ir_tracking_sample_t *sample);

/** 将模块原始字节转换为归一化黑线掩码。 */
uint8_t IRTracking_RawToLineMask(uint8_t raw);

/** 返回最近一次有效线误差；丢线时仍保留该值。 */
int16_t IRTracking_GetLastError(void);

/** 通过 UART0 打印一次解析后的采样，便于调试。 */
void IRTracking_PrintSample(const ir_tracking_sample_t *sample);

#endif /* _BSP_IR_TRACKING_H */
