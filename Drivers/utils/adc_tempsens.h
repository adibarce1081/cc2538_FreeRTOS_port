
#ifndef	_ADC_TEMPSENS_H_
#define	_ADC_TEMPSENS_H_

#include <stdbool.h>
#include <stdint.h>
#include "hw_memmap.h"
#include "gpio.h"
#include "hw_ioc.h"
#include "ioc.h"
#include "interrupt.h"
#include "adc.h"
#include "sys_ctrl.h"
#include "hw_sys_ctrl.h"
#include "systick.h"
#include "uartstdio.h"
#include "hw_cctest.h"
#include "hw_rfcore_xreg.h"
#include <string.h>
#include <stdio.h>

float internal_soc_temperature(void);

#endif
