#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"

#define TYPE_ESP32_SENS "misc.esp32.sens"
#define ESP32_SENS_ADC_CHANNELS 12
#define ESP32_SENS(obj) OBJECT_CHECK(Esp32SensState, (obj), TYPE_ESP32_SENS)

typedef struct Esp32SensState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    uint32_t mem[0x1000 / sizeof(uint32_t)];

    /*
     * Raw 12-bit result returned for each SAR ADC channel on a forced
     * (RTC controller / oneshot) conversion. Defaults to mid-scale (2048).
     * Updated at runtime by the EVCS bridge (hw/xtensa/esp32_evcs_bridge.c).
     */
    uint16_t adc1_value[ESP32_SENS_ADC_CHANNELS];
    uint16_t adc2_value[ESP32_SENS_ADC_CHANNELS];
    uint32_t adc1_reads[ESP32_SENS_ADC_CHANNELS];
    uint32_t adc2_reads[ESP32_SENS_ADC_CHANNELS];
} Esp32SensState;

void esp32_sens_set_adc_value(Esp32SensState *s, int unit, int channel, uint16_t value);

