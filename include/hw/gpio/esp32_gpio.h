#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"

#define TYPE_ESP32_GPIO "esp32.gpio"
#define ESP32_GPIO(obj)             OBJECT_CHECK(Esp32GpioState, (obj), TYPE_ESP32_GPIO)
#define ESP32_GPIO_GET_CLASS(obj)   OBJECT_GET_CLASS(Esp32GpioClass, obj, TYPE_ESP32_GPIO)
#define ESP32_GPIO_CLASS(klass)     OBJECT_CLASS_CHECK(Esp32GpioClass, klass, TYPE_ESP32_GPIO)

#define ESP32_GPIO_MEM_SIZE 0x1000

REG32(GPIO_OUT, 0x0004)
REG32(GPIO_OUT_W1TS, 0x0008)
REG32(GPIO_OUT_W1TC, 0x000c)
REG32(GPIO_OUT1, 0x0010)
REG32(GPIO_OUT1_W1TS, 0x0014)
REG32(GPIO_OUT1_W1TC, 0x0018)

REG32(GPIO_ENABLE, 0x0020)
REG32(GPIO_ENABLE_W1TS, 0x0024)
REG32(GPIO_ENABLE_W1TC, 0x0028)
REG32(GPIO_ENABLE1, 0x002c)
REG32(GPIO_ENABLE1_W1TS, 0x0030)
REG32(GPIO_ENABLE1_W1TC, 0x0034)

REG32(GPIO_STRAP, 0x0038)
REG32(GPIO_IN, 0x003c)
REG32(GPIO_IN1, 0x0040)

REG32(GPIO_STATUS, 0x0044)
REG32(GPIO_STATUS_W1TS, 0x0048)
REG32(GPIO_STATUS_W1TC, 0x004c)
REG32(GPIO_STATUS1, 0x0050)
REG32(GPIO_STATUS1_W1TS, 0x0054)
REG32(GPIO_STATUS1_W1TC, 0x0058)

#define ESP32_STRAP_MODE_FLASH_BOOT 0x12
#define ESP32_STRAP_MODE_UART_BOOT  0x0f

#define ESP_GPIO_MAX_PINS 40

/* IO_MUX register bit definitions */
#define ESP_IOMUX_FUN_PD_BIT    7
#define ESP_IOMUX_FUN_PU_BIT    8

typedef struct Esp32GpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion iomux;
    qemu_irq irq;
    uint32_t strap_mode;
    uint32_t strap_offset;

    /* GPIO output loopback state */
    uint32_t gpio_out;
    uint32_t gpio_enable;
    uint32_t gpio_pullup;   /* bitmask: pins with pull-up enabled */
    uint32_t out_off;       /* GPIO_OUT_REG offset */
    uint32_t out_w1ts_off;  /* GPIO_OUT_W1TS_REG offset */
    uint32_t out_w1tc_off;  /* GPIO_OUT_W1TC_REG offset */
    uint32_t enable_off;    /* GPIO_ENABLE_REG offset */
    uint32_t enable_w1ts_off;
    uint32_t enable_w1tc_off;
    uint32_t in_off;        /* GPIO_IN_REG offset */

    /* IO_MUX state */
    uint32_t iomux_regs[ESP_GPIO_MAX_PINS];
    uint32_t num_gpios;

    /*
     * Classic ESP32 register file, used when a subclass has not configured the
     * offsets above (out_off == 0): a mirror of the whole GPIO block, both pin
     * banks (OUT/OUT1, ENABLE/ENABLE1) and external input levels.
     */
    /*
     * External input levels as seen by GPIO_IN / GPIO_IN1 when the pin is not
     * configured as an output (ENABLE bit = 0).
     *
     * Defaults to all-ones to approximate pull-ups on unconnected inputs,
     * which is important for firmware that samples active-low buttons during
     * startup.
     */
    uint32_t input0;
    uint32_t input1;
    uint32_t mem[ESP32_GPIO_MEM_SIZE / sizeof(uint32_t)];
} Esp32GpioState;

typedef struct Esp32GpioClass {
    SysBusDeviceClass parent_class;
} Esp32GpioClass;
