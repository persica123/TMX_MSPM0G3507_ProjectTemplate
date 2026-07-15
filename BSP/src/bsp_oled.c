#include "bsp_oled.h"
#include "board.h"

#define OLED_WIDTH             (128U)
#define OLED_PAGES             (8U)
#define OLED_I2C_TIMEOUT       (100000UL)
#define OLED_PACKET_MAX        (8U)
#define OLED_DATA_CHUNK        (7U)

static uint8_t g_oled_ready;

static uint8_t OLED_I2CWaitIdle(void)
{
    uint32_t timeout = OLED_I2C_TIMEOUT;

    while (((DL_I2C_getControllerStatus(I2C_OLED_INST) &
        DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) && (timeout != 0U)) {
        timeout--;
    }

    return (timeout != 0U) ? 1U : 0U;
}

static uint8_t OLED_I2CWaitDone(void)
{
    uint32_t timeout = OLED_I2C_TIMEOUT;

    while (((DL_I2C_getControllerStatus(I2C_OLED_INST) &
        DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) && (timeout != 0U)) {
        timeout--;
    }

    if (timeout == 0U) {
        DL_I2C_resetControllerTransfer(I2C_OLED_INST);
        return 0U;
    }

    if ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
        DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        DL_I2C_resetControllerTransfer(I2C_OLED_INST);
        return 0U;
    }

    return 1U;
}

static uint8_t OLED_I2CWritePacket(const uint8_t *data, uint16_t length)
{
    if ((data == 0) || (length == 0U) || (length > OLED_PACKET_MAX)) {
        return 0U;
    }

    if (OLED_I2CWaitIdle() == 0U) {
        return 0U;
    }

    DL_I2C_fillControllerTXFIFO(I2C_OLED_INST, data, length);
    DL_I2C_startControllerTransfer(I2C_OLED_INST,
        OLED_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        length);
    delay_cycles(16);

    return OLED_I2CWaitDone();
}

static uint8_t OLED_WriteControlBytes(uint8_t control,
    const uint8_t *data,
    uint16_t length)
{
    uint8_t packet[OLED_PACKET_MAX];
    uint16_t offset = 0U;

    while (offset < length) {
        uint16_t chunk = (uint16_t)(length - offset);

        if (chunk > OLED_DATA_CHUNK) {
            chunk = OLED_DATA_CHUNK;
        }

        packet[0] = control;
        for (uint16_t index = 0U; index < chunk; index++) {
            packet[index + 1U] = data[offset + index];
        }

        if (OLED_I2CWritePacket(packet, (uint16_t)(chunk + 1U)) == 0U) {
            return 0U;
        }

        offset = (uint16_t)(offset + chunk);
    }

    return 1U;
}

static uint8_t OLED_WriteCommand(uint8_t command)
{
    uint8_t packet[2U] = {0x00U, command};

    return OLED_I2CWritePacket(packet, 2U);
}

static uint8_t OLED_WriteData(const uint8_t *data, uint16_t length)
{
    return OLED_WriteControlBytes(0x40U, data, length);
}

static void OLED_SetCursor(uint8_t page, uint8_t column)
{
    (void)OLED_WriteCommand((uint8_t)(0xB0U | (page & 0x07U)));
    (void)OLED_WriteCommand((uint8_t)(0x00U | (column & 0x0FU)));
    (void)OLED_WriteCommand((uint8_t)(0x10U | ((column >> 4U) & 0x0FU)));
}

static const uint8_t *OLED_GetGlyph(char c)
{
    static const uint8_t blank[5U] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    static const uint8_t dash[5U]  = {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};
    static const uint8_t dot[5U]   = {0x00U, 0x60U, 0x60U, 0x00U, 0x00U};
    static const uint8_t colon[5U] = {0x00U, 0x36U, 0x36U, 0x00U, 0x00U};
    static const uint8_t digits[10U][5U] = {
        {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU},
        {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U},
        {0x42U, 0x61U, 0x51U, 0x49U, 0x46U},
        {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U},
        {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U},
        {0x27U, 0x45U, 0x45U, 0x45U, 0x39U},
        {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U},
        {0x01U, 0x71U, 0x09U, 0x05U, 0x03U},
        {0x36U, 0x49U, 0x49U, 0x49U, 0x36U},
        {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU},
    };
    static const uint8_t letter_a[5U] = {0x20U, 0x54U, 0x54U, 0x54U, 0x78U};
    static const uint8_t letter_e[5U] = {0x38U, 0x54U, 0x54U, 0x54U, 0x18U};
    static const uint8_t letter_i[5U] = {0x00U, 0x44U, 0x7DU, 0x40U, 0x00U};
    static const uint8_t letter_l[5U] = {0x00U, 0x41U, 0x7FU, 0x40U, 0x00U};
    static const uint8_t letter_o[5U] = {0x38U, 0x44U, 0x44U, 0x44U, 0x38U};
    static const uint8_t letter_t[5U] = {0x04U, 0x3FU, 0x44U, 0x40U, 0x20U};
    static const uint8_t letter_w[5U] = {0x3CU, 0x40U, 0x30U, 0x40U, 0x3CU};
    static const uint8_t letter_A[5U] = {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU};
    static const uint8_t letter_D[5U] = {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU};
    static const uint8_t letter_E[5U] = {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U};
    static const uint8_t letter_J[5U] = {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U};
    static const uint8_t letter_K[5U] = {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U};
    static const uint8_t letter_M[5U] = {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU};
    static const uint8_t letter_O[5U] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU};
    static const uint8_t letter_P[5U] = {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U};
    static const uint8_t letter_R[5U] = {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U};
    static const uint8_t letter_S[5U] = {0x46U, 0x49U, 0x49U, 0x49U, 0x31U};
    static const uint8_t letter_Y[5U] = {0x07U, 0x08U, 0x70U, 0x08U, 0x07U};
    static const uint8_t letter_c[5U] = {0x38U, 0x44U, 0x44U, 0x44U, 0x20U};
    static const uint8_t letter_m[5U] = {0x7CU, 0x04U, 0x18U, 0x04U, 0x78U};
    static const uint8_t letter_s[5U] = {0x48U, 0x54U, 0x54U, 0x54U, 0x20U};

    if ((c >= '0') && (c <= '9')) {
        return digits[(uint8_t)(c - '0')];
    }

    switch (c) {
        case '-': return dash;
        case '.': return dot;
        case ':': return colon;
        case 'a': return letter_a;
        case 'c': return letter_c;
        case 'e': return letter_e;
        case 'i': return letter_i;
        case 'l': return letter_l;
        case 'm': return letter_m;
        case 'o': return letter_o;
        case 's': return letter_s;
        case 't': return letter_t;
        case 'w': return letter_w;
        case 'A': return letter_A;
        case 'D': return letter_D;
        case 'E': return letter_E;
        case 'J': return letter_J;
        case 'K': return letter_K;
        case 'M': return letter_M;
        case 'O': return letter_O;
        case 'P': return letter_P;
        case 'R': return letter_R;
        case 'S': return letter_S;
        case 'Y': return letter_Y;
        default: return blank;
    }
}

static void OLED_WriteChar(char c)
{
    uint8_t data[6U];
    const uint8_t *glyph = OLED_GetGlyph(c);

    for (uint8_t index = 0U; index < 5U; index++) {
        data[index] = glyph[index];
    }
    data[5U] = 0x00U;

    (void)OLED_WriteData(data, sizeof(data));
}

static void OLED_PrintAt(uint8_t page, uint8_t column, const char *text)
{
    OLED_SetCursor(page, column);
    while ((text != 0) && (*text != '\0')) {
        OLED_WriteChar(*text);
        text++;
    }
}

static void OLED_FillLine(char *line)
{
    for (uint8_t index = 0U; index < 21U; index++) {
        line[index] = ' ';
    }
    line[21U] = '\0';
}

static uint8_t OLED_AppendText(char *line, uint8_t pos, const char *text)
{
    while ((text != 0) && (*text != '\0') && (pos < 21U)) {
        line[pos] = *text;
        pos++;
        text++;
    }

    return pos;
}

static uint8_t OLED_AppendUInt(char *line, uint8_t pos, uint32_t value)
{
    char tmp[10U];
    uint8_t count = 0U;

    do {
        tmp[count] = (char)('0' + (value % 10U));
        count++;
        value /= 10U;
    } while ((value != 0U) && (count < sizeof(tmp)));

    while ((count != 0U) && (pos < 21U)) {
        count--;
        line[pos] = tmp[count];
        pos++;
    }

    return pos;
}

static uint8_t OLED_AppendCdeg(char *line, uint8_t pos, int32_t cdeg)
{
    uint32_t value;

    if (cdeg < 0) {
        line[pos] = '-';
        pos++;
        value = (uint32_t)(-cdeg);
    } else {
        value = (uint32_t)cdeg;
    }

    pos = OLED_AppendUInt(line, pos, value / 100U);
    if (pos < 21U) {
        line[pos] = '.';
        pos++;
    }
    if (pos < 21U) {
        line[pos] = (char)('0' + ((value / 10U) % 10U));
        pos++;
    }
    if (pos < 21U) {
        line[pos] = (char)('0' + (value % 10U));
        pos++;
    }

    return pos;
}

static void OLED_FormatAngleLine(char *line, const char *label, int32_t value)
{
    uint8_t pos;

    OLED_FillLine(line);
    pos = OLED_AppendText(line, 0U, label);
    pos = OLED_AppendText(line, pos, ":");
    (void)OLED_AppendCdeg(line, pos, value);
}

static void OLED_FormatDistanceLine(char *line, int32_t distance_cm)
{
    uint8_t pos;
    uint32_t value;

    OLED_FillLine(line);
    pos = OLED_AppendText(line, 0U, "Dis:");
    if (distance_cm < 0) {
        if (pos < 21U) {
            line[pos] = '-';
            pos++;
        }
        value = (uint32_t)(-distance_cm);
    } else {
        value = (uint32_t)distance_cm;
    }
    pos = OLED_AppendUInt(line, pos, value);
    (void)OLED_AppendText(line, pos, "cm");
}

uint8_t OLED_Init(void)
{
    static const uint8_t init_cmds[] = {
        0xAEU, 0x20U, 0x02U, 0xB0U, 0xC8U, 0x00U, 0x10U, 0x40U,
        0x81U, 0x7FU, 0xA1U, 0xA6U, 0xA8U, 0x3FU, 0xA4U, 0xD3U,
        0x00U, 0xD5U, 0x80U, 0xD9U, 0xF1U, 0xDAU, 0x12U, 0xDBU,
        0x40U, 0x8DU, 0x14U, 0xAFU
    };

    delay_ms(80);
    for (uint8_t index = 0U; index < sizeof(init_cmds); index++) {
        if (OLED_WriteCommand(init_cmds[index]) == 0U) {
            g_oled_ready = 0U;
            return 0U;
        }
    }

    g_oled_ready = 1U;
    OLED_Clear();
    OLED_PrintAt(0U, 0U, "JY61P OLED OK");
    return 1U;
}

void OLED_Clear(void)
{
    uint8_t zeros[OLED_DATA_CHUNK] = {0U};

    if (g_oled_ready == 0U) {
        return;
    }

    for (uint8_t page = 0U; page < OLED_PAGES; page++) {
        OLED_SetCursor(page, 0U);
        for (uint8_t column = 0U; column < OLED_WIDTH; column += OLED_DATA_CHUNK) {
            uint8_t chunk = (uint8_t)(OLED_WIDTH - column);

            if (chunk > OLED_DATA_CHUNK) {
                chunk = OLED_DATA_CHUNK;
            }
            (void)OLED_WriteData(zeros, chunk);
        }
    }
}

void OLED_ShowNavigation(int32_t yaw_cdeg,
    int32_t rel_cdeg,
    int32_t roll_cdeg,
    int32_t pitch_cdeg,
    uint8_t valid,
    uint8_t i2c_status,
    uint32_t frame_count,
    uint32_t i2c_error_count)
{
    char line[22U];
    uint8_t pos;

    if (g_oled_ready == 0U) {
        return;
    }

    OLED_FormatAngleLine(line, "Yaw", yaw_cdeg);
    OLED_PrintAt(0U, 0U, line);

    OLED_FormatAngleLine(line, "Rel", rel_cdeg);
    OLED_PrintAt(2U, 0U, line);

    OLED_FormatAngleLine(line, "Rol", roll_cdeg);
    OLED_PrintAt(4U, 0U, line);

    OLED_FormatAngleLine(line, "Pit", pitch_cdeg);
    if ((valid != 0U) && (frame_count != 0U)) {
        pos = OLED_AppendText(line, 13U, "OK:");
        (void)OLED_AppendUInt(line, pos, frame_count);
    } else {
        pos = OLED_AppendText(line, 13U, "E");
        pos = OLED_AppendUInt(line, pos, i2c_status);
        pos = OLED_AppendText(line, pos, ":");
        (void)OLED_AppendUInt(line, pos, i2c_error_count);
    }
    OLED_PrintAt(6U, 0U, line);
}

void OLED_ShowYawDistance(int32_t yaw_cdeg,
    int32_t distance_cm,
    uint8_t valid)
{
    char line[22U];

    if (g_oled_ready == 0U) {
        return;
    }

    OLED_FormatAngleLine(line, "Yaw", yaw_cdeg);
    OLED_PrintAt(0U, 0U, line);

    OLED_FormatDistanceLine(line, distance_cm);
    OLED_PrintAt(3U, 0U, line);

    OLED_FillLine(line);
    (void)OLED_AppendText(line, 0U, (valid != 0U) ? "OK" : "E");
    OLED_PrintAt(6U, 0U, line);
}
