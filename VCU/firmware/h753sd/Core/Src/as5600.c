/************************************************************************
Title:    as5600.c - Driver for AMS AS5600 12-Bit Programmable Contactless
                     Potentiometer (magnetic rotary position sensor)
Original: Nicholas Morrow <nickhudspeth@gmail.com> (STM32F0 version)
Port:     Adapted for STM32H7 series (tested target: STM32H753VIT6)
License:  The MIT License (MIT)
Usage:    Refer to the header file as5600.h for a description of the
          routines and the list of changes made in this port.
************************************************************************/

/**********************    INCLUDE DIRECTIVES    ***********************/
#include "as5600.h"
#include <stdlib.h>

/**********************    PRIVATE HELPERS    ***************************/
/* HAL expects the 7-bit address shifted left by one. */
#define AS5600_HAL_ADDR ((uint16_t)(AS5600_SLAVE_ADDRESS << 1))

static HAL_StatusTypeDef AS5600_ReadRegisters(AS5600_TypeDef *const a,
                                              const uint8_t reg,
                                              uint8_t *const data,
                                              const uint16_t len)
{
    return HAL_I2C_Mem_Read(a->i2cHandle, AS5600_HAL_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, data, len,
                            AS5600_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef AS5600_WriteRegisters(AS5600_TypeDef *const a,
                                               const uint8_t reg,
                                               uint8_t *const data,
                                               const uint16_t len)
{
    return HAL_I2C_Mem_Write(a->i2cHandle, AS5600_HAL_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, data, len,
                             AS5600_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef AS5600_WriteConf(AS5600_TypeDef *const a)
{
    return AS5600_WriteRegisters(a, AS5600_REGISTER_CONF_HIGH,
                                 a->confRegister, 2U);
}

/*******************    FUNCTION IMPLEMENTATIONS    ********************/
void AS5600_InitStruct(AS5600_TypeDef *a)
{
    if (a == NULL) {
        return;
    }
    a->i2cHandle                 = NULL;
    a->DirPort                   = NULL;
    a->DirPin                    = 0U;
    a->PositiveRotationDirection = 0U;
    a->LowPowerMode              = 0U;
    a->Hysteresis                = 0U;
    a->OutputMode                = 0U;
    a->PWMFrequency              = 0U;
    a->SlowFilter                = 0U;
    a->FastFilterThreshold       = 0U;
    a->WatchdogTimer             = 0U;
    a->confRegister[0]           = 0U;
    a->confRegister[1]           = 0U;
}

AS5600_TypeDef *AS5600_New(void)
{
    AS5600_TypeDef *a = (AS5600_TypeDef *)calloc(1, sizeof(AS5600_TypeDef));
    return a;
}

HAL_StatusTypeDef AS5600_Init(AS5600_TypeDef *a)
{
    uint8_t pwm        = 0U;
    uint8_t mag_status = 0U;

    if ((a == NULL) || (a->i2cHandle == NULL)) {
        return HAL_ERROR;
    }

    /* Set configuration defaults for uninitialized values. */
    if (!(a->PositiveRotationDirection)) {
        a->PositiveRotationDirection = AS5600_DIR_CW;
    }
    if (!(a->LowPowerMode)) {
        a->LowPowerMode = AS5600_POWER_MODE_DEFAULT;
    }
    if (!(a->Hysteresis)) {
        a->Hysteresis = AS5600_HYSTERESIS_DEFAULT;
    }
    if (!(a->OutputMode)) {
        a->OutputMode = AS5600_OUTPUT_STAGE_DEFAULT;
    }
    if (!(a->PWMFrequency)) {
        a->PWMFrequency = AS5600_PWM_FREQUENCY_DEFAULT;
    }
    if (!(a->SlowFilter)) {
        a->SlowFilter = AS5600_SLOW_FILTER_DEFAULT;
    }
    if (!(a->FastFilterThreshold)) {
        a->FastFilterThreshold = AS5600_FAST_FILTER_DEFAULT;
    }
    if (!(a->WatchdogTimer)) {
        a->WatchdogTimer = AS5600_WATCHDOG_DEFAULT;
    }

    /* Drive DIR pin if user assigned one. */
    if (a->DirPort != NULL) {
        if (AS5600_SetPositiveRotationDirection(
                a, a->PositiveRotationDirection) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    /* Build CONF register content locally, then write in a single
     * blocking transaction. confRegister[0] = 0x07 (high byte),
     * confRegister[1] = 0x08 (low byte). */
    switch (a->LowPowerMode) { /* PM: CONF low byte, bits 1:0 */
        case AS5600_POWER_MODE_NOM:
            a->confRegister[1] &= (uint8_t)~((1U << 1) | (1U << 0));
            break;
        case AS5600_POWER_MODE_LPM1:
            a->confRegister[1] |= (uint8_t)(1U << 0);
            a->confRegister[1] &= (uint8_t)~(1U << 1);
            break;
        case AS5600_POWER_MODE_LPM2:
            a->confRegister[1] |= (uint8_t)(1U << 1);
            a->confRegister[1] &= (uint8_t)~(1U << 0); /* FIX: was '&= (1<<0)' */
            break;
        case AS5600_POWER_MODE_LPM3:
            a->confRegister[1] |= (uint8_t)((1U << 1) | (1U << 0));
            break;
        default:
            return HAL_ERROR; /* Invalid low power mode specified */
    }
    switch (a->Hysteresis) { /* HYST: CONF low byte, bits 3:2 */
        case AS5600_HYSTERESIS_OFF:
            a->confRegister[1] &= (uint8_t)~((1U << 3) | (1U << 2));
            break;
        case AS5600_HYSTERESIS_1LSB:
            a->confRegister[1] |= (uint8_t)(1U << 2);
            a->confRegister[1] &= (uint8_t)~(1U << 3);
            break;
        case AS5600_HYSTERESIS_2LSB:
            a->confRegister[1] &= (uint8_t)~(1U << 2);
            a->confRegister[1] |= (uint8_t)(1U << 3);
            break;
        case AS5600_HYSTERESIS_3LSB:
            a->confRegister[1] |= (uint8_t)((1U << 3) | (1U << 2));
            break;
        default:
            return HAL_ERROR; /* Invalid hysteresis mode specified */
    }
    switch (a->OutputMode) { /* OUTS: CONF low byte, bits 5:4 */
        case AS5600_OUTPUT_STAGE_FULL:
            a->confRegister[1] &= (uint8_t)~((1U << 5) | (1U << 4));
            break;
        case AS5600_OUTPUT_STAGE_REDUCED:
            a->confRegister[1] |= (uint8_t)(1U << 4);
            a->confRegister[1] &= (uint8_t)~(1U << 5);
            break;
        case AS5600_OUTPUT_STAGE_PWM:
            a->confRegister[1] &= (uint8_t)~(1U << 4); /* FIX: was '&= (1<<4)' */
            a->confRegister[1] |= (uint8_t)(1U << 5);
            pwm = 1U;
            break;
        default:
            return HAL_ERROR; /* Invalid output mode specified */
    }
    if (pwm) {
        switch (a->PWMFrequency) { /* PWMF: CONF low byte, bits 7:6 */
            case AS5600_PWM_FREQUENCY_115HZ:
                a->confRegister[1] &= (uint8_t)~((1U << 7) | (1U << 6));
                break;
            case AS5600_PWM_FREQUENCY_230HZ:
                a->confRegister[1] |= (uint8_t)(1U << 6);
                a->confRegister[1] &= (uint8_t)~(1U << 7);
                break;
            case AS5600_PWM_FREQUENCY_460HZ:
                a->confRegister[1] &= (uint8_t)~(1U << 6);
                a->confRegister[1] |= (uint8_t)(1U << 7);
                break;
            case AS5600_PWM_FREQUENCY_920HZ:
                a->confRegister[1] |= (uint8_t)((1U << 7) | (1U << 6));
                break;
            default:
                return HAL_ERROR; /* Invalid PWM frequency specified */
        }
    }
    switch (a->SlowFilter) { /* SF: CONF high byte, bits 1:0 */
        case AS5600_SLOW_FILTER_16X:
            a->confRegister[0] &= (uint8_t)~((1U << 1) | (1U << 0));
            break;
        case AS5600_SLOW_FILTER_8X:
            a->confRegister[0] |= (uint8_t)(1U << 0);
            a->confRegister[0] &= (uint8_t)~(1U << 1);
            break;
        case AS5600_SLOW_FILTER_4X:
            a->confRegister[0] &= (uint8_t)~(1U << 0);
            a->confRegister[0] |= (uint8_t)(1U << 1);
            break;
        case AS5600_SLOW_FILTER_2X:
            a->confRegister[0] |= (uint8_t)((1U << 1) | (1U << 0));
            break;
        default:
            return HAL_ERROR; /* Invalid slow filter mode specified */
    }
    switch (a->FastFilterThreshold) { /* FTH: CONF high byte, bits 4:2 */
        case AS5600_FAST_FILTER_SLOW_ONLY:
            a->confRegister[0] &= (uint8_t)~((1U << 4) | (1U << 3) | (1U << 2));
            break;
        case AS5600_FAST_FILTER_6LSB:
            a->confRegister[0] &= (uint8_t)~((1U << 4) | (1U << 3));
            a->confRegister[0] |= (uint8_t)(1U << 2);
            break;
        case AS5600_FAST_FILTER_7LSB:
            a->confRegister[0] &= (uint8_t)~((1U << 4) | (1U << 2));
            a->confRegister[0] |= (uint8_t)(1U << 3);
            break;
        case AS5600_FAST_FILTER_9LSB:
            a->confRegister[0] &= (uint8_t)~(1U << 4);
            a->confRegister[0] |= (uint8_t)((1U << 3) | (1U << 2));
            break;
        case AS5600_FAST_FILTER_18LSB:
            a->confRegister[0] &= (uint8_t)~((1U << 3) | (1U << 2));
            a->confRegister[0] |= (uint8_t)(1U << 4);
            break;
        case AS5600_FAST_FILTER_21LSB:
            a->confRegister[0] &= (uint8_t)~(1U << 3);
            a->confRegister[0] |= (uint8_t)((1U << 4) | (1U << 2));
            break;
        case AS5600_FAST_FILTER_24LSB:
            a->confRegister[0] &= (uint8_t)~(1U << 2);
            a->confRegister[0] |= (uint8_t)((1U << 4) | (1U << 3));
            break;
        case AS5600_FAST_FILTER_10LSB:
            a->confRegister[0] |= (uint8_t)((1U << 4) | (1U << 3) | (1U << 2));
            break;
        default:
            return HAL_ERROR; /* Invalid fast filter threshold specified */
    }
    switch (a->WatchdogTimer) { /* WD: CONF high byte, bit 5 (bit 13 of CONF) */
        case AS5600_WATCHDOG_OFF:
            a->confRegister[0] &= (uint8_t)~(1U << 5); /* FIX: was bit 6 */
            break;
        case AS5600_WATCHDOG_ON:
            a->confRegister[0] |= (uint8_t)(1U << 5);  /* FIX: was bit 6 */
            break;
        default:
            return HAL_ERROR; /* Invalid watchdog state specified */
    }

    /* Write configuration in a single blocking transaction. */
    if (AS5600_WriteConf(a) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Check magnet status. */
    if (AS5600_GetMagnetStatus(a, &mag_status) != HAL_OK) {
        return HAL_ERROR;
    }
    if (!(mag_status & AS5600_MAGNET_DETECTED)) {
        return HAL_ERROR; /* Magnet not detected */
    }
    if (mag_status & AS5600_AGC_MIN_GAIN_OVERFLOW) {
        return HAL_ERROR; /* B-field is too strong */
    }
    if (mag_status & AS5600_AGC_MAX_GAIN_OVERFLOW) {
        return HAL_ERROR; /* B-field is too weak */
    }

    return HAL_OK;
}

HAL_StatusTypeDef AS5600_SetStartPosition(AS5600_TypeDef *const a,
                                          const uint16_t pos)
{
    uint8_t data[2];
    data[0] = (uint8_t)((pos & AS5600_12_BIT_MASK) >> 8);
    data[1] = (uint8_t)pos;
    return AS5600_WriteRegisters(a, AS5600_REGISTER_ZPOS_HIGH, data, 2U);
}

HAL_StatusTypeDef AS5600_SetStopPosition(AS5600_TypeDef *const a,
                                         const uint16_t pos)
{
    uint8_t data[2];
    data[0] = (uint8_t)((pos & AS5600_12_BIT_MASK) >> 8);
    data[1] = (uint8_t)pos;
    return AS5600_WriteRegisters(a, AS5600_REGISTER_MPOS_HIGH, data, 2U);
}

HAL_StatusTypeDef AS5600_SetMaxAngle(AS5600_TypeDef *const a,
                                     const uint16_t angle)
{
    uint8_t data[2];
    data[0] = (uint8_t)((angle & AS5600_12_BIT_MASK) >> 8);
    data[1] = (uint8_t)angle;
    return AS5600_WriteRegisters(a, AS5600_REGISTER_MANG_HIGH, data, 2U);
}

HAL_StatusTypeDef AS5600_SetPositiveRotationDirection(AS5600_TypeDef *const a,
                                                      const uint8_t dir)
{
    if (a->DirPort == NULL) {
        /* DIR pin not connected to the MCU (hard-wired). Nothing to do. */
        return HAL_OK;
    }
    if (dir == AS5600_DIR_CW) {
        HAL_GPIO_WritePin(a->DirPort, a->DirPin, GPIO_PIN_RESET);
    } else if (dir == AS5600_DIR_CCW) {
        HAL_GPIO_WritePin(a->DirPort, a->DirPin, GPIO_PIN_SET);
    } else {
        return HAL_ERROR; /* Invalid rotation direction specified */
    }
    a->PositiveRotationDirection = dir;
    return HAL_OK;
}

HAL_StatusTypeDef AS5600_SetLowPowerMode(AS5600_TypeDef *const a,
                                         const uint8_t mode)
{
    switch (mode) {
        case AS5600_POWER_MODE_NOM:
            a->confRegister[1] &= (uint8_t)~((1U << 1) | (1U << 0));
            break;
        case AS5600_POWER_MODE_LPM1:
            a->confRegister[1] |= (uint8_t)(1U << 0);
            a->confRegister[1] &= (uint8_t)~(1U << 1);
            break;
        case AS5600_POWER_MODE_LPM2:
            a->confRegister[1] |= (uint8_t)(1U << 1);
            a->confRegister[1] &= (uint8_t)~(1U << 0);
            break;
        case AS5600_POWER_MODE_LPM3:
            a->confRegister[1] |= (uint8_t)((1U << 1) | (1U << 0));
            break;
        default:
            return HAL_ERROR;
    }
    a->LowPowerMode = mode;
    return AS5600_WriteConf(a);
}

HAL_StatusTypeDef AS5600_SetHysteresis(AS5600_TypeDef *const a,
                                       const uint8_t hysteresis)
{
    switch (hysteresis) {
        case AS5600_HYSTERESIS_OFF:
            a->confRegister[1] &= (uint8_t)~((1U << 3) | (1U << 2));
            break;
        case AS5600_HYSTERESIS_1LSB:
            a->confRegister[1] |= (uint8_t)(1U << 2);
            a->confRegister[1] &= (uint8_t)~(1U << 3);
            break;
        case AS5600_HYSTERESIS_2LSB:
            a->confRegister[1] &= (uint8_t)~(1U << 2);
            a->confRegister[1] |= (uint8_t)(1U << 3);
            break;
        case AS5600_HYSTERESIS_3LSB:
            a->confRegister[1] |= (uint8_t)((1U << 3) | (1U << 2));
            break;
        default:
            return HAL_ERROR;
    }
    a->Hysteresis = hysteresis;
    return AS5600_WriteConf(a);
}

HAL_StatusTypeDef AS5600_SetOutputMode(AS5600_TypeDef *const a,
                                       const uint8_t mode, uint8_t freq)
{
    uint8_t pwm = 0U;
    switch (mode) {
        case AS5600_OUTPUT_STAGE_FULL:
            a->confRegister[1] &= (uint8_t)~((1U << 5) | (1U << 4));
            break;
        case AS5600_OUTPUT_STAGE_REDUCED:
            a->confRegister[1] |= (uint8_t)(1U << 4);
            a->confRegister[1] &= (uint8_t)~(1U << 5);
            break;
        case AS5600_OUTPUT_STAGE_PWM:
            a->confRegister[1] &= (uint8_t)~(1U << 4);
            a->confRegister[1] |= (uint8_t)(1U << 5);
            pwm = 1U;
            break;
        default:
            return HAL_ERROR;
    }
    if (pwm) {
        switch (freq) {
            case AS5600_PWM_FREQUENCY_115HZ:
                a->confRegister[1] &= (uint8_t)~((1U << 7) | (1U << 6));
                break;
            case AS5600_PWM_FREQUENCY_230HZ:
                a->confRegister[1] |= (uint8_t)(1U << 6);
                a->confRegister[1] &= (uint8_t)~(1U << 7);
                break;
            case AS5600_PWM_FREQUENCY_460HZ:
                a->confRegister[1] &= (uint8_t)~(1U << 6);
                a->confRegister[1] |= (uint8_t)(1U << 7);
                break;
            case AS5600_PWM_FREQUENCY_920HZ:
                a->confRegister[1] |= (uint8_t)((1U << 7) | (1U << 6));
                break;
            default:
                return HAL_ERROR;
        }
        a->PWMFrequency = freq;
    }
    a->OutputMode = mode;
    return AS5600_WriteConf(a);
}

HAL_StatusTypeDef AS5600_SetSlowFilter(AS5600_TypeDef *const a,
                                       const uint8_t mode)
{
    switch (mode) {
        case AS5600_SLOW_FILTER_16X:
            a->confRegister[0] &= (uint8_t)~((1U << 1) | (1U << 0));
            break;
        case AS5600_SLOW_FILTER_8X:
            a->confRegister[0] |= (uint8_t)(1U << 0);
            a->confRegister[0] &= (uint8_t)~(1U << 1);
            break;
        case AS5600_SLOW_FILTER_4X:
            a->confRegister[0] &= (uint8_t)~(1U << 0);
            a->confRegister[0] |= (uint8_t)(1U << 1);
            break;
        case AS5600_SLOW_FILTER_2X:
            a->confRegister[0] |= (uint8_t)((1U << 1) | (1U << 0));
            break;
        default:
            return HAL_ERROR;
    }
    a->SlowFilter = mode;
    return AS5600_WriteConf(a);
}

HAL_StatusTypeDef AS5600_SetFastFilterThreshold(AS5600_TypeDef *const a,
                                                const uint8_t threshold)
{
    switch (threshold) {
        case AS5600_FAST_FILTER_SLOW_ONLY:
            a->confRegister[0] &= (uint8_t)~((1U << 4) | (1U << 3) | (1U << 2));
            break;
        case AS5600_FAST_FILTER_6LSB:
            a->confRegister[0] &= (uint8_t)~((1U << 4) | (1U << 3));
            a->confRegister[0] |= (uint8_t)(1U << 2);
            break;
        case AS5600_FAST_FILTER_7LSB:
            a->confRegister[0] &= (uint8_t)~((1U << 4) | (1U << 2));
            a->confRegister[0] |= (uint8_t)(1U << 3);
            break;
        case AS5600_FAST_FILTER_9LSB:
            a->confRegister[0] &= (uint8_t)~(1U << 4);
            a->confRegister[0] |= (uint8_t)((1U << 3) | (1U << 2));
            break;
        case AS5600_FAST_FILTER_18LSB:
            a->confRegister[0] &= (uint8_t)~((1U << 3) | (1U << 2));
            a->confRegister[0] |= (uint8_t)(1U << 4);
            break;
        case AS5600_FAST_FILTER_21LSB:
            a->confRegister[0] &= (uint8_t)~(1U << 3);
            a->confRegister[0] |= (uint8_t)((1U << 4) | (1U << 2));
            break;
        case AS5600_FAST_FILTER_24LSB:
            a->confRegister[0] &= (uint8_t)~(1U << 2);
            a->confRegister[0] |= (uint8_t)((1U << 4) | (1U << 3));
            break;
        case AS5600_FAST_FILTER_10LSB:
            a->confRegister[0] |= (uint8_t)((1U << 4) | (1U << 3) | (1U << 2));
            break;
        default:
            return HAL_ERROR;
    }
    a->FastFilterThreshold = threshold;
    return AS5600_WriteConf(a);
}

HAL_StatusTypeDef AS5600_SetWatchdogTimer(AS5600_TypeDef *const a,
                                          const uint8_t mode)
{
    switch (mode) {
        case AS5600_WATCHDOG_OFF:
            a->confRegister[0] &= (uint8_t)~(1U << 5);
            break;
        case AS5600_WATCHDOG_ON:
            a->confRegister[0] |= (uint8_t)(1U << 5);
            break;
        default:
            return HAL_ERROR;
    }
    a->WatchdogTimer = mode;
    return AS5600_WriteConf(a);
}

HAL_StatusTypeDef AS5600_GetRawAngle(AS5600_TypeDef *const a,
                                     uint16_t *const angle)
{
    uint8_t data[2] = {0};
    HAL_StatusTypeDef status =
        AS5600_ReadRegisters(a, AS5600_REGISTER_RAW_ANGLE_HIGH, data, 2U);
    if (status == HAL_OK) {
        *angle = (uint16_t)(((uint16_t)(data[0] & 0x0FU) << 8) | data[1]);
    }
    return status;
}

HAL_StatusTypeDef AS5600_GetAngle(AS5600_TypeDef *const a,
                                  uint16_t *const angle)
{
    uint8_t data[2] = {0};
    HAL_StatusTypeDef status =
        AS5600_ReadRegisters(a, AS5600_REGISTER_ANGLE_HIGH, data, 2U);
    if (status == HAL_OK) {
        *angle = (uint16_t)(((uint16_t)(data[0] & 0x0FU) << 8) | data[1]);
    }
    return status;
}

HAL_StatusTypeDef AS5600_GetAngleDegrees(AS5600_TypeDef *const a,
                                         float *const degrees)
{
    uint16_t raw = 0U;
    HAL_StatusTypeDef status = AS5600_GetAngle(a, &raw);
    if (status == HAL_OK) {
        *degrees = ((float)raw * 360.0f) / 4096.0f;
    }
    return status;
}

HAL_StatusTypeDef AS5600_GetMagnetStatus(AS5600_TypeDef *const a,
                                         uint8_t *const stat)
{
    return AS5600_ReadRegisters(a, AS5600_REGISTER_STATUS, stat, 1U);
}

HAL_StatusTypeDef AS5600_GetAGCSetting(AS5600_TypeDef *const a,
                                       uint8_t *const agc)
{
    return AS5600_ReadRegisters(a, AS5600_REGISTER_AGC, agc, 1U);
}

HAL_StatusTypeDef AS5600_GetCORDICMagnitude(AS5600_TypeDef *const a,
                                            uint16_t *const mag)
{
    uint8_t data[2] = {0};
    /* FIX: original read the ANGLE register here instead of MAGNITUDE. */
    HAL_StatusTypeDef status =
        AS5600_ReadRegisters(a, AS5600_REGISTER_MAGNITUDE_HIGH, data, 2U);
    if (status == HAL_OK) {
        *mag = (uint16_t)(((uint16_t)(data[0] & 0x0FU) << 8) | data[1]);
    }
    return status;
}

HAL_StatusTypeDef AS5600_GetZMCO(AS5600_TypeDef *const a, uint8_t *const zmco)
{
    HAL_StatusTypeDef status =
        AS5600_ReadRegisters(a, AS5600_REGISTER_ZMCO, zmco, 1U);
    if (status == HAL_OK) {
        *zmco &= 0x03U; /* only two LSBs are valid */
    }
    return status;
}
