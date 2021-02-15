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
#define SUN_BUF_SIZE 256
#define SUN_RINGS 6

/* Exported macro ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct sunimation_t
{
  uint8_t buf[SUN_BUF_SIZE];
  uint8_t bufFront;
  __IO uint32_t* dimmerRegisters[SUN_RINGS];
} sunimation_t;

/* Exported function prototypes ----------------------------------------------*/
void sunimationInit(sunimation_t* _this, uint32_t* d0, uint32_t* d1, uint32_t* d2, uint32_t* d3, uint32_t* d4, uint32_t* st);

#ifdef __cplusplus
}
#endif

#endif __SUNIMATION_H
