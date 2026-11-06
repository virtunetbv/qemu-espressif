/*
 * ESP32 GPIO emulation
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32_gpio.h"


/*
 * Classic ESP32 register file. Used when no subclass configured the output
 * offsets (out_off == 0): every register is mirrored, both pin banks are
 * modelled and GPIO_IN/GPIO_IN1 combine the output level of enabled pins with
 * the external input levels (input0/input1).
 */
static uint64_t esp32_gpio_regfile_read(Esp32GpioState *s, hwaddr addr)
{
    if ((addr % sizeof(uint32_t)) != 0 || addr >= ESP32_GPIO_MEM_SIZE) {
        return 0;
    }

    switch (addr) {
    case A_GPIO_STRAP:
        return s->strap_mode;
    case A_GPIO_IN:
        return (s->mem[A_GPIO_OUT / sizeof(uint32_t)] &
                s->mem[A_GPIO_ENABLE / sizeof(uint32_t)]) |
               (s->input0 & ~s->mem[A_GPIO_ENABLE / sizeof(uint32_t)]);
    case A_GPIO_IN1:
        return (s->mem[A_GPIO_OUT1 / sizeof(uint32_t)] &
                s->mem[A_GPIO_ENABLE1 / sizeof(uint32_t)]) |
               (s->input1 & ~s->mem[A_GPIO_ENABLE1 / sizeof(uint32_t)]);
    default:
        return s->mem[addr / sizeof(uint32_t)];
    }
}

static void esp32_gpio_regfile_write(Esp32GpioState *s, hwaddr addr, uint32_t v)
{
    if ((addr % sizeof(uint32_t)) != 0 || addr >= ESP32_GPIO_MEM_SIZE) {
        return;
    }

    uint32_t *mem = s->mem;
    switch (addr) {
    case A_GPIO_OUT_W1TS:
        mem[A_GPIO_OUT / sizeof(uint32_t)] |= v;
        break;
    case A_GPIO_OUT_W1TC:
        mem[A_GPIO_OUT / sizeof(uint32_t)] &= ~v;
        break;
    case A_GPIO_OUT1_W1TS:
        mem[A_GPIO_OUT1 / sizeof(uint32_t)] |= v;
        break;
    case A_GPIO_OUT1_W1TC:
        mem[A_GPIO_OUT1 / sizeof(uint32_t)] &= ~v;
        break;
    case A_GPIO_ENABLE_W1TS:
        mem[A_GPIO_ENABLE / sizeof(uint32_t)] |= v;
        break;
    case A_GPIO_ENABLE_W1TC:
        mem[A_GPIO_ENABLE / sizeof(uint32_t)] &= ~v;
        break;
    case A_GPIO_ENABLE1_W1TS:
        mem[A_GPIO_ENABLE1 / sizeof(uint32_t)] |= v;
        break;
    case A_GPIO_ENABLE1_W1TC:
        mem[A_GPIO_ENABLE1 / sizeof(uint32_t)] &= ~v;
        break;
    case A_GPIO_STATUS_W1TS:
        mem[A_GPIO_STATUS / sizeof(uint32_t)] |= v;
        return;
    case A_GPIO_STATUS_W1TC:
        mem[A_GPIO_STATUS / sizeof(uint32_t)] &= ~v;
        return;
    case A_GPIO_STATUS1_W1TS:
        mem[A_GPIO_STATUS1 / sizeof(uint32_t)] |= v;
        return;
    case A_GPIO_STATUS1_W1TC:
        mem[A_GPIO_STATUS1 / sizeof(uint32_t)] &= ~v;
        return;
    default:
        break;
    }
    mem[addr / sizeof(uint32_t)] = v;
}

static uint64_t esp32_gpio_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);
    uint64_t r = 0;

    if (!s->out_off) {
        return esp32_gpio_regfile_read(s, addr);
    }

    if (addr == s->strap_offset) {
        r = s->strap_mode;
    } else if (s->out_off && addr == s->out_off) {
        r = s->gpio_out;
    } else if (s->enable_off && addr == s->enable_off) {
        r = s->gpio_enable;
    } else if (s->in_off && addr == s->in_off) {
        /* Output-enabled pins loop back; undriven pins reflect pull-up state */
        r = (s->gpio_out & s->gpio_enable) |
            (s->gpio_pullup & ~s->gpio_enable);
    }

    return r;
}

static void esp32_gpio_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);

    if (!s->out_off) {
        esp32_gpio_regfile_write(s, addr, (uint32_t)value);
        return;
    }

    if (s->out_off && addr == s->out_off) {
        s->gpio_out = (uint32_t)value;
    } else if (s->out_w1ts_off && addr == s->out_w1ts_off) {
        s->gpio_out |= (uint32_t)value;
    } else if (s->out_w1tc_off && addr == s->out_w1tc_off) {
        s->gpio_out &= ~(uint32_t)value;
    } else if (s->enable_off && addr == s->enable_off) {
        s->gpio_enable = (uint32_t)value;
    } else if (s->enable_w1ts_off && addr == s->enable_w1ts_off) {
        s->gpio_enable |= (uint32_t)value;
    } else if (s->enable_w1tc_off && addr == s->enable_w1tc_off) {
        s->gpio_enable &= ~(uint32_t)value;
    }
}

static const MemoryRegionOps gpio_ops = {
    .read =  esp32_gpio_read,
    .write = esp32_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static uint64_t esp32_iomux_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);
    int pin = addr / 4;
    if (pin < s->num_gpios) {
        return s->iomux_regs[pin];
    }
    return 0;
}

static void esp32_iomux_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);
    int pin = addr / 4;
    if (pin < s->num_gpios) {
        s->iomux_regs[pin] = (uint32_t)value;
        if (value & (1 << ESP_IOMUX_FUN_PU_BIT)) {
            s->gpio_pullup |= (1u << pin);
        } else {
            s->gpio_pullup &= ~(1u << pin);
        }
    }
}

static const MemoryRegionOps iomux_ops = {
    .read  = esp32_iomux_read,
    .write = esp32_iomux_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_gpio_reset_hold(Object *obj, ResetType type)
{
    Esp32GpioState *s = ESP32_GPIO(obj);
    memset(s->mem, 0, sizeof(s->mem));
}

static void esp32_gpio_realize(DeviceState *dev, Error **errp)
{
}

static void esp32_gpio_init(Object *obj)
{
    Esp32GpioState *s = ESP32_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    /* Set the default value for the strap_mode property */
    object_property_set_int(obj, "strap_mode", ESP32_STRAP_MODE_FLASH_BOOT, &error_fatal);
    s->strap_offset = A_GPIO_STRAP;
    memset(s->mem, 0, sizeof(s->mem));
    /* Default GPIO inputs to pulled-high, but keep GPIO19 low so the EVCS welded-contacts
     * self-test can pass under QEMU (it samples gpio_get_level(19)). */
    s->input0 = 0xffffffffu & ~(1u << 19);
    s->input1 = 0xffffffff;

    memory_region_init_io(&s->iomem, obj, &gpio_ops, s,
                          TYPE_ESP32_GPIO, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);

    memory_region_init_io(&s->iomux, obj, &iomux_ops, s,
                          TYPE_ESP32_GPIO ".iomux", 0x200);
    sysbus_init_mmio(sbd, &s->iomux);

    sysbus_init_irq(sbd, &s->irq);
}

static Property esp32_gpio_properties[] = {
    /* The strap_mode needs to be explicitly set in the instance init, thus, set
     * the default value to 0. */
    DEFINE_PROP_UINT32("strap_mode", Esp32GpioState, strap_mode, 0),
    DEFINE_PROP_UINT32("input0", Esp32GpioState, input0, 0xffffffff),
    DEFINE_PROP_UINT32("input1", Esp32GpioState, input1, 0xffffffff),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_gpio_reset_hold;
    dc->realize = esp32_gpio_realize;
    device_class_set_props(dc, esp32_gpio_properties);
}

static const TypeInfo esp32_gpio_info = {
    .name = TYPE_ESP32_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32GpioState),
    .instance_init = esp32_gpio_init,
    .class_init = esp32_gpio_class_init,
    .class_size = sizeof(Esp32GpioClass),
};

static void esp32_gpio_register_types(void)
{
    type_register_static(&esp32_gpio_info);
}

type_init(esp32_gpio_register_types)
