
/**
 * @ingroup     cpu_samr21
 * @{
 *
 * @file
 * @brief       Dummy ADC driver implementation
 *
 * @author      C.A.
 *
 * @}
 */

#include <stdint.h>
#include "cpu.h"
#include "periph/adc.h"
#include "periph_conf.h"
#define ENABLE_DEBUG    (0)
#include "debug.h"


int adc_init(adc_t dev, adc_precision_t precision)
{ return 0; }

int adc_sample(adc_t dev, int channel)
{ return 111; } 

void adc_poweron(adc_t dev)
{ }

void adc_poweroff(adc_t dev)
{ }

int adc_map(adc_t dev, int value, int min, int max)
{ }

float adc_mapf(adc_t dev, int value, float min, float max)
{ return 0; }

bool adc_syncing(Adc* adc)
{ return false; }

int adc_configure_with_resolution(Adc* adc, uint32_t precision)
{ return 1; }

void adc_clear_status(Adc* adc, uint32_t status_flag)
{ }

uint32_t adc_get_status(Adc* adc)
{ return 0; }
