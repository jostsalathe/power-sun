/**
  ******************************************************************************
  * @file           : sunimation.h
  * @brief          : Header for sunimation.c file.
  ******************************************************************************
  * @author         : Jost Salathé
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SUNIMATION_H
#define __SUNIMATION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Defines -------------------------------------------------------------------*/
#define SUN_BUF_SIZE 256 //caution! some of the buffer mechanics depend on this being uint8 max
#define SUN_RINGS 6

/* Exported macro ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum sunimationState_t
{
  sunimationOff,
  sunimationStandbyTimeout,
  sunimationStarting,
  sunimationRunning
} sunimationState_t;

typedef struct sunimation_t
{
  // working variables
  sunimationState_t state;
  uint8_t buf[SUN_BUF_SIZE];
  uint8_t bufFront;
  uint8_t standbyTimeoutCounter;

  // configuration variables
  __IO uint32_t* dimmerRegisters[SUN_RINGS];
  uint8_t sunMasterBrightness;
  uint8_t stripesMasterBrightness;
  uint8_t propagationDelay;
  uint8_t standbyBrightness;
  uint8_t standbyIncrement;
  uint8_t standbyTimeout;
  uint8_t startingBrightness;
  uint8_t startingIncrement;
  uint8_t idleBrightness;
  uint8_t idleDecrement;
  uint8_t activeBrightness;
  uint8_t activeIncrement;
  uint8_t stoppingDecrement;
} sunimation_t;

/* Exported function prototypes ----------------------------------------------*/
uint16_t gammaDimmed(uint8_t value, uint8_t brightness);
void sunimationInit(sunimation_t* _this, __IO uint32_t* d0, __IO uint32_t* d1, __IO uint32_t* d2, __IO uint32_t* d3, __IO uint32_t* d4, __IO uint32_t* st);
void sunimationAdvance(sunimation_t* _this, GPIO_PinState isOn, GPIO_PinState isActive);

#ifdef __cplusplus
}
#endif

#endif
