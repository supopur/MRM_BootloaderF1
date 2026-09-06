#ifndef __STM32F10X_CONF_H
#define __STM32F10X_CONF_H

#include "stm32f10x_can.h"
#include "stm32f10x_crc.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_iwdg.h"
#include "stm32f10x_rcc.h"

#ifdef USE_FULL_ASSERT
#define assert_param(expr) \
((expr) ? (void)0 : 0)
#else
#define assert_param(expr) ((void)0)
#endif

#endif