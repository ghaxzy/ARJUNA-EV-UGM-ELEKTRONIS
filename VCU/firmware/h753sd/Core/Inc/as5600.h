/*************************************************************************
Title:    as5600.h - Driver for AMS AS5600 12-Bit Programmable Contactless
                     Potentiometer (magnetic rotary position sensor)
Original: Nicholas Morrow <nickhudspeth@gmail.com> (STM32F0 version)
Port:     Adapted for STM32H7 series (tested target: STM32H753VIT6)
Software: STM32H7xx_HAL_Driver, CMSIS-CORE
License:  The MIT License (MIT)

CHANGES vs ORIGINAL (STM32F0 -> STM32H7 port):
    - Include changed to "stm32h7xx_hal.h" (auto-detected for other families).
    - All I2C transfers now use BLOCKING HAL calls (HAL_I2C_Mem_Read/Write)
      with a timeout. The original used *_IT (interrupt) calls but read the
      result from a stack buffer immediately, which returns garbage / corrupt
      stack data. Blocking mode is safe and correct here.
    - I2C address is shifted internally (HAL expects 8-bit address).
      Just call AS5600_Init() -- no manual shifting needed.
    - Fixed bit-manipulation bugs in LPM2 and PWM output-stage config
      (original cleared unrelated bits with '&=' instead of '&= ~').
    - Fixed watchdog bit position (bit 5 of CONF high byte per datasheet,
      original wrote bit 6).
    - Fixed AS5600_GetCORDICMagnitude() reading the ANGLE register instead
      of the MAGNITUDE register.
    - Added AS5600_InitStruct() so the driver can be used without dynamic
      allocation (recommended for embedded), AS5600_New() kept for
      compatibility.
    - Added helper AS5600_GetAngleDegrees() and magnet-status helpers.
    - DIR pin is now optional (set DirPort = NULL if the pin is hard-wired).

USAGE (STM32CubeIDE / HAL, blocking mode):

    // I2C1 must be initialized by CubeMX first (hi2c1)
    AS5600_TypeDef as5600;
    AS5600_InitStruct(&as5600);          // load defaults
    as5600.i2cHandle = &hi2c1;
    // Optional DIR pin (comment out if DIR is wired to GND/VDD):
    // as5600.DirPort = GPIOB; as5600.DirPin = GPIO_PIN_0;
    // as5600.PositiveRotationDirection = AS5600_DIR_CW;

    if (AS5600_Init(&as5600) != HAL_OK) {
        Error_Handler();
    }

    uint16_t raw;
    float deg;
    while (1) {
        AS5600_GetAngle(&as5600, &raw);          // 0..4095
        AS5600_GetAngleDegrees(&as5600, &deg);   // 0..359.9
        HAL_Delay(10);
    }

NOTES:
    - Driver does not support permanent OTP burn commands (by design,
      to avoid bricking the sensor accidentally).
    - Analog/PWM output measurement not handled here (I2C only).
    - AS5600 supports I2C up to 1 MHz; 100k/400k from CubeMX works fine.

LICENSE:
    Copyright (C) 2018 Pathogen Systems, Inc. dba Crystal Diagnostics
    MIT License - see original header in source repository.
*************************************************************************/
#ifndef AS5600_H_
#define AS5600_H_

#ifdef __cplusplus
extern "C" {
#endif

/**********************    INCLUDE DIRECTIVES    ***********************/
#include <stdint.h>

#if defined(STM32H7) || defined(STM32H753xx) || defined(STM32H743xx) ||       \
    defined(STM32H750xx) || defined(STM32H7xx)
#include "stm32h7xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#elif defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F0)
#include "stm32f0xx_hal.h"
#elif defined(STM32G4)
#include "stm32g4xx_hal.h"
#else
/* Default target for this port: STM32H753VIT6 */
#include "stm32h7xx_hal.h"
#endif

/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
/* 7-bit I2C address of AS5600. The driver shifts it internally for HAL. */
#define AS5600_SLAVE_ADDRESS            0x36U
#define AS5600_I2C_TIMEOUT_MS           10U   /* blocking transfer timeout */

/* AS5600 Configuration Registers */
#define AS5600_REGISTER_ZMCO            0x00U
#define AS5600_REGISTER_ZPOS_HIGH       0x01U
#define AS5600_REGISTER_ZPOS_LOW        0x02U
#define AS5600_REGISTER_MPOS_HIGH       0x03U
#define AS5600_REGISTER_MPOS_LOW        0x04U
#define AS5600_REGISTER_MANG_HIGH       0x05U
#define AS5600_REGISTER_MANG_LOW        0x06U
#define AS5600_REGISTER_CONF_HIGH       0x07U
#define AS5600_REGISTER_CONF_LOW        0x08U
/* AS5600 Output Registers */
#define AS5600_REGISTER_RAW_ANGLE_HIGH  0x0CU
#define AS5600_REGISTER_RAW_ANGLE_LOW   0x0DU
#define AS5600_REGISTER_ANGLE_HIGH      0x0EU
#define AS5600_REGISTER_ANGLE_LOW       0x0FU
/* AS5600 Status Registers */
#define AS5600_REGISTER_STATUS          0x0BU
#define AS5600_REGISTER_AGC             0x1AU
#define AS5600_REGISTER_MAGNITUDE_HIGH  0x1BU
#define AS5600_REGISTER_MAGNITUDE_LOW   0x1CU
#define AS5600_REGISTER_BURN            0xFFU

/* AS5600 Configuration Settings */
#define AS5600_POWER_MODE_NOM           1
#define AS5600_POWER_MODE_LPM1          2
#define AS5600_POWER_MODE_LPM2          3
#define AS5600_POWER_MODE_LPM3          4
#define AS5600_POWER_MODE_DEFAULT       AS5600_POWER_MODE_NOM

#define AS5600_HYSTERESIS_OFF           1
#define AS5600_HYSTERESIS_1LSB          2
#define AS5600_HYSTERESIS_2LSB          3
#define AS5600_HYSTERESIS_3LSB          4
#define AS5600_HYSTERESIS_DEFAULT       AS5600_HYSTERESIS_OFF

#define AS5600_OUTPUT_STAGE_FULL        1 /* Analog output, GND..VCC        */
#define AS5600_OUTPUT_STAGE_REDUCED     2 /* Analog output, 10%..90% of VCC */
#define AS5600_OUTPUT_STAGE_PWM         3 /* Digital PWM output             */
#define AS5600_OUTPUT_STAGE_DEFAULT     AS5600_OUTPUT_STAGE_FULL

#define AS5600_PWM_FREQUENCY_115HZ      1
#define AS5600_PWM_FREQUENCY_230HZ      2
#define AS5600_PWM_FREQUENCY_460HZ      3
#define AS5600_PWM_FREQUENCY_920HZ      4
#define AS5600_PWM_FREQUENCY_DEFAULT    AS5600_PWM_FREQUENCY_115HZ

#define AS5600_SLOW_FILTER_16X          1
#define AS5600_SLOW_FILTER_8X           2
#define AS5600_SLOW_FILTER_4X           3
#define AS5600_SLOW_FILTER_2X           4
#define AS5600_SLOW_FILTER_DEFAULT      AS5600_SLOW_FILTER_16X

#define AS5600_FAST_FILTER_SLOW_ONLY    1
#define AS5600_FAST_FILTER_6LSB         2
#define AS5600_FAST_FILTER_7LSB         3
#define AS5600_FAST_FILTER_9LSB         4
#define AS5600_FAST_FILTER_18LSB        5
#define AS5600_FAST_FILTER_21LSB        6
#define AS5600_FAST_FILTER_24LSB        7
#define AS5600_FAST_FILTER_10LSB        8
#define AS5600_FAST_FILTER_DEFAULT      AS5600_FAST_FILTER_SLOW_ONLY

#define AS5600_WATCHDOG_OFF             1
#define AS5600_WATCHDOG_ON              2
#define AS5600_WATCHDOG_DEFAULT        AS5600_WATCHDOG_OFF

/* AS5600 Status bits (register 0x0B) */
#define AS5600_AGC_MIN_GAIN_OVERFLOW  ((uint8_t)(1U << 3)) /* MH: field too strong */
#define AS5600_AGC_MAX_GAIN_OVERFLOW  ((uint8_t)(1U << 4)) /* ML: field too weak   */
#define AS5600_MAGNET_DETECTED        ((uint8_t)(1U << 5)) /* MD: magnet detected  */

#define AS5600_DIR_CW                   1
#define AS5600_DIR_CCW                  2

#define AS5600_12_BIT_MASK              ((uint16_t)4095U)

typedef struct {
    I2C_HandleTypeDef *i2cHandle;      /* HAL I2C handle, e.g. &hi2c1       */
    GPIO_TypeDef      *DirPort;        /* Optional DIR pin port (NULL = unused) */
    uint16_t           DirPin;         /* Optional DIR pin                  */
    uint8_t            PositiveRotationDirection; /* AS5600_DIR_CW / _CCW   */
    uint8_t            LowPowerMode;
    uint8_t            Hysteresis;
    uint8_t            OutputMode;
    uint8_t            PWMFrequency;
    uint8_t            SlowFilter;
    uint8_t            FastFilterThreshold;
    uint8_t            WatchdogTimer;

    /* Private */
    uint8_t            confRegister[2]; /* [0]=CONF high (0x07), [1]=CONF low (0x08) */
} AS5600_TypeDef;

/***********************    FUNCTION PROTOTYPES    ***********************/
/* Allocation-free init (recommended): zeroes the struct, then AS5600_Init()
 * fills in defaults for any field left at 0. */
void               AS5600_InitStruct(AS5600_TypeDef *a);
/* Heap-allocated variant, kept for API compatibility with the original. */
AS5600_TypeDef    *AS5600_New(void);

HAL_StatusTypeDef  AS5600_Init(AS5600_TypeDef *a);

HAL_StatusTypeDef  AS5600_SetStartPosition(AS5600_TypeDef *const a,
                                           const uint16_t pos);
HAL_StatusTypeDef  AS5600_SetStopPosition(AS5600_TypeDef *const a,
                                          const uint16_t pos);
HAL_StatusTypeDef  AS5600_SetMaxAngle(AS5600_TypeDef *const a,
                                      const uint16_t angle);

HAL_StatusTypeDef  AS5600_SetPositiveRotationDirection(AS5600_TypeDef *const a,
                                                       const uint8_t dir);

HAL_StatusTypeDef  AS5600_SetLowPowerMode(AS5600_TypeDef *const a,
                                          const uint8_t mode);
HAL_StatusTypeDef  AS5600_SetHysteresis(AS5600_TypeDef *const a,
                                        const uint8_t hysteresis);
HAL_StatusTypeDef  AS5600_SetOutputMode(AS5600_TypeDef *const a,
                                        const uint8_t mode, uint8_t freq);
HAL_StatusTypeDef  AS5600_SetSlowFilter(AS5600_TypeDef *const a,
                                        const uint8_t mode);
HAL_StatusTypeDef  AS5600_SetFastFilterThreshold(AS5600_TypeDef *const a,
                                                 const uint8_t threshold);
HAL_StatusTypeDef  AS5600_SetWatchdogTimer(AS5600_TypeDef *const a,
                                           const uint8_t mode);

HAL_StatusTypeDef  AS5600_GetRawAngle(AS5600_TypeDef *const a,
                                      uint16_t *const angle);
HAL_StatusTypeDef  AS5600_GetAngle(AS5600_TypeDef *const a,
                                   uint16_t *const angle);
HAL_StatusTypeDef  AS5600_GetAngleDegrees(AS5600_TypeDef *const a,
                                          float *const degrees);
HAL_StatusTypeDef  AS5600_GetMagnetStatus(AS5600_TypeDef *const a,
                                          uint8_t *const stat);
HAL_StatusTypeDef  AS5600_GetAGCSetting(AS5600_TypeDef *const a,
                                        uint8_t *const agc);
HAL_StatusTypeDef  AS5600_GetCORDICMagnitude(AS5600_TypeDef *const a,
                                             uint16_t *const mag);
HAL_StatusTypeDef  AS5600_GetZMCO(AS5600_TypeDef *const a,
                                  uint8_t *const zmco);

#ifdef __cplusplus
}
#endif

#endif /* AS5600_H_ */
