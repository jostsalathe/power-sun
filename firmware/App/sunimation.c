/**
  ******************************************************************************
  * @file           : sunimation.c
  * @brief          : This file implements the sun animation.
  ******************************************************************************
  * @author         : Jost Salathé
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sunimation.h"
#include "usbd_cdc_if.h"

/* Private types -------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/
// Gamma brightness lookup table <https://victornpb.github.io/gamma-table-generator>
// gamma = 2.30 steps = 256 range = 0-65535
const uint16_t gamma_lut[256] = {
     0,   0,   1,   2,   5,   8,  12,  17,  23,  30,  38,  47,  58,  70,  83,  97,
   112, 129, 147, 167, 188, 210, 234, 259, 286, 314, 343, 375, 407, 442, 477, 515,
   554, 594, 637, 680, 726, 773, 822, 873, 925, 979,1035,1092,1152,1213,1276,1340,
  1407,1475,1545,1617,1691,1767,1845,1924,2006,2089,2174,2261,2351,2442,2535,2630,
  2727,2826,2927,3030,3135,3242,3351,3462,3575,3690,3808,3927,4049,4172,4298,4426,
  4556,4688,4822,4958,5097,5237,5380,5525,5672,5821,5973,6127,6283,6441,6601,6764,
  6929,7096,7265,7437,7611,7787,7965,8146,8329,8515,8702,8892,9085,9279,9476,9676,
  9877,10081,10288,10496,10707,10921,11137,11355,11576,11799,12024,12252,12482,12715,12950,13188,
  13428,13671,13916,14163,14413,14665,14920,15177,15437,15700,15964,16232,16502,16774,17049,17326,
  17606,17889,18174,18461,18751,19044,19339,19637,19938,20240,20546,20854,21165,21478,21794,22113,
  22434,22758,23084,23413,23745,24079,24416,24756,25098,25443,25791,26141,26494,26850,27208,27569,
  27933,28299,28668,29040,29414,29791,30171,30554,30939,31328,31718,32112,32508,32907,33309,33714,
  34121,34531,34944,35360,35778,36200,36624,37050,37480,37912,38348,38786,39227,39670,40117,40566,
  41018,41473,41931,42392,42855,43322,43791,44263,44738,45216,45696,46180,46666,47156,47648,48143,
  48641,49142,49646,50152,50662,51174,51690,52208,52729,53254,53781,54311,54844,55380,55919,56461,
  57005,57553,58104,58658,59214,59774,60337,60902,61471,62043,62617,63195,63775,64359,64945,65535,
  };


/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void sunimateOff(sunimation_t* _this, GPIO_PinState isOn);
void sunimateStandbyTimeout(sunimation_t* _this, GPIO_PinState isOn);
void sunimateStarting(sunimation_t* _this, GPIO_PinState isOn);
void sunimateRunning(sunimation_t* _this, GPIO_PinState isOn, GPIO_PinState isActive);
uint8_t fade(uint8_t currentValue, uint8_t targetValue, uint8_t increment);
void show(sunimation_t* _this);
void bufPut(sunimation_t* _this, uint8_t value);
uint8_t bufGet(sunimation_t* _this, uint8_t offset);

/* Exported function implementations ------------------------------------------*/
uint16_t gammaDimmed(uint8_t value, uint8_t brightness)
{
  return ((uint32_t) gamma_lut[value]) * brightness / 255;
}

void sunimationInit(sunimation_t* _this, __IO uint32_t* d0, __IO uint32_t* d1, __IO uint32_t* d2, __IO uint32_t* d3, __IO uint32_t* d4, __IO uint32_t* st)
{
  _this->state = sunimationOff;
  for (int i=0; i<SUN_BUF_SIZE; ++i)
  {
    _this->buf[i] = 0;
  }
  _this->bufFront = 0;
  _this->standbyTimeoutCounter = 0;

  _this->dimmerRegisters[0] = d0;
  _this->dimmerRegisters[1] = d1;
  _this->dimmerRegisters[2] = d2;
  _this->dimmerRegisters[3] = d3;
  _this->dimmerRegisters[4] = d4;
  _this->dimmerRegisters[5] = st;
  _this->sunMasterBrightness = 63;
  _this->stripesMasterBrightness = 160;
  _this->propagationDelay = 4;
  _this->standbyBrightness = 20;
  _this->standbyIncrement = 2;
  _this->standbyTimeout = 32;
  _this->startingBrightness = 255;
  _this->startingIncrement = 8;
  _this->idleBrightness = 100;
  _this->idleDecrement = 2;
  _this->activeBrightness = 160;
  _this->activeIncrement = 6;
  _this->stoppingDecrement = 4;
}

void sunimationAdvance(sunimation_t* _this, GPIO_PinState isOn, GPIO_PinState isActive)
{
  switch(_this->state)
  {
  default:
    _this->state = sunimationOff;
    // deliberate fall-through
  case sunimationOff:
    sunimateOff(_this, isOn);
    break;
  case sunimationStandbyTimeout:
    sunimateStandbyTimeout(_this, isOn);
    break;
  case sunimationStarting:
    sunimateStarting(_this, isOn);
    break;
  case sunimationRunning:
    sunimateRunning(_this, isOn, isActive);
    break;
  }
  show(_this);
}

/* Private function implementations -------------------------------------------*/
void sunimateOff(sunimation_t* _this, GPIO_PinState isOn)
{
  uint8_t newValue = fade(bufGet(_this, 0), 0, _this->stoppingDecrement);
  bufPut(_this, newValue);
  if (isOn)
  {
    _this->standbyTimeoutCounter = 0;
    _this->state = sunimationStandbyTimeout;
  }
}

void sunimateStandbyTimeout(sunimation_t* _this, GPIO_PinState isOn)
{
  uint8_t newValue = fade(bufGet(_this, 0), _this->standbyBrightness, _this->standbyIncrement);
  bufPut(_this, newValue);
  if (isOn)
  {
    if (++_this->standbyTimeoutCounter >= _this->standbyTimeout)
    {
      _this->state = sunimationStarting;
    }
  }
  else
  {
    _this->state = sunimationOff;
  }
}

void sunimateStarting(sunimation_t* _this, GPIO_PinState isOn)
{
  uint8_t newValue = fade(bufGet(_this, 0), _this->startingBrightness, _this->startingIncrement);
  bufPut(_this, newValue);
  if (isOn)
  {
    if (newValue >= _this->startingBrightness)
    {
      _this->state = sunimationRunning;
    }
  }
  else
  {
    _this->state = sunimationOff;
  }
}

void sunimateRunning(sunimation_t* _this, GPIO_PinState isOn, GPIO_PinState isActive)
{
  uint8_t targetBrightness;
  uint8_t increment;
  if (isActive)
  {
    targetBrightness = _this->activeBrightness;
    increment = _this->activeIncrement;
  }
  else
  {
    targetBrightness = _this->idleBrightness;
    increment = _this->idleDecrement;
  }
  uint8_t newValue = fade(bufGet(_this, 0), targetBrightness, increment);
  bufPut(_this, newValue);
  if (!isOn)
  {
    _this->state = sunimationOff;
  }
}

uint8_t fade(uint8_t currentValue, uint8_t targetValue, uint8_t increment)
{
  int delta = (int) targetValue;
  delta -= (int) currentValue;

  if (delta > 0)
  {
    if (delta > increment)
    {
      return currentValue + increment;
    }
  }
  else if (delta < 0)
  {
    if ((delta * -1) > increment)
    {
      return currentValue - increment;
    }
  }

  return targetValue;
}

void show(sunimation_t* _this)
{
  uint8_t brightness = _this->sunMasterBrightness;
  for(int i=0; i<SUN_RINGS; ++i)
  {
    if (i == SUN_RINGS - 1)
    {
      brightness = _this->stripesMasterBrightness;
    }
    uint8_t value = bufGet(_this, _this->propagationDelay * i);
    *_this->dimmerRegisters[i] = gammaDimmed(value, brightness);
  }
}

void bufPut(sunimation_t* _this, uint8_t value)
{
  _this->buf[++_this->bufFront] = value;
}

uint8_t bufGet(sunimation_t* _this, uint8_t offset)
{
  uint8_t pos = _this->bufFront - offset;
  return _this->buf[pos];
}

