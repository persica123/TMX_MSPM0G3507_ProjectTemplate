#ifndef _BSP_OLED_H
#define _BSP_OLED_H

#include <stdint.h>

#define OLED_I2C_ADDR          (0x3CU)
#define OLED_REFRESH_MIN_MS    (200U)

uint8_t OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowNavigation(int32_t yaw_cdeg,
    int32_t rel_cdeg,
    int32_t roll_cdeg,
    int32_t pitch_cdeg,
    uint8_t valid,
    uint8_t i2c_status,
    uint32_t frame_count,
    uint32_t i2c_error_count);
void OLED_ShowYawDistance(int32_t yaw_cdeg,
    int32_t distance_cm,
    uint8_t valid);

#endif /* _BSP_OLED_H */
