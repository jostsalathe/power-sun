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
  sunimationIdle,
  sunimationActive,
  sunimationStopping
} sunimationState_t;

typedef struct sunimation_t
{
  sunimationState_t state;
  uint8_t buf[SUN_BUF_SIZE];
  uint8_t bufFront;
  __IO uint32_t* dimmerRegisters[SUN_RINGS];
  uint8_t sunMasterBrightness;
  uint8_t stripesMasterBrightness;
  uint8_t onPeakBrightness;
  uint8_t onTimeConstant;
  uint8_t idleBrightness;
  uint8_t activeBrightness;
  uint8_t activeTimeConstant;
  uint8_t propagationDelay;
  uint8_t offTimeConstant;
} sunimation_t;

/* Exported function prototypes ----------------------------------------------*/
void sunimationInit(sunimation_t* _this, uint32_t* d0, uint32_t* d1, uint32_t* d2, uint32_t* d3, uint32_t* d4, uint32_t* st);
void sunimationAdvance(sunimation_t* _this, uint8_t isOn, uint8_t isActive);

#ifdef __cplusplus
}
#endif

#endif __SUNIMATION_H
