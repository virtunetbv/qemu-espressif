#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/misc/esp32_ledc.h"

#define ESP32_LEDC_REGS_SIZE (A_LEDC_CONF_REG + 4)

static uint64_t esp32_ledc_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32LEDCState *s = ESP32_LEDC(opaque);
    uint64_t r = addr < sizeof(s->regs) ? s->regs[addr / 4] : 0;
    if (addr < A_LEDC_HSTIMER0_CONF_REG &&
        (addr % 0x14) == 0x10) {
        /* CHn_DUTY_R: current duty, same format as CHn_DUTY (4 fraction bits). */
        r = s->regs[(addr - 0x8) / 4] & 0x1ffffff;
    }
    switch (addr) {
    case A_LEDC_HSTIMER0_CONF_REG ... A_LEDC_LSTIMER3_CONF_REG:
        r = s->timer_conf_reg[(addr - A_LEDC_HSTIMER0_CONF_REG) / 0x8];
        break;

    case A_LEDC_HSCH0_CONF0_REG:
    case A_LEDC_HSCH1_CONF0_REG:
    case A_LEDC_HSCH2_CONF0_REG:
    case A_LEDC_HSCH3_CONF0_REG:
    case A_LEDC_HSCH4_CONF0_REG:
    case A_LEDC_HSCH5_CONF0_REG:
    case A_LEDC_HSCH6_CONF0_REG:
    case A_LEDC_HSCH7_CONF0_REG:
    case A_LEDC_LSCH0_CONF0_REG:
    case A_LEDC_LSCH1_CONF0_REG:
    case A_LEDC_LSCH2_CONF0_REG:
    case A_LEDC_LSCH3_CONF0_REG:
    case A_LEDC_LSCH4_CONF0_REG:
    case A_LEDC_LSCH5_CONF0_REG:
    case A_LEDC_LSCH6_CONF0_REG:
    case A_LEDC_LSCH7_CONF0_REG:
        r = s->channel_conf0_reg[(addr - A_LEDC_HSCH0_CONF0_REG) / 0x14];
        break;
    }
    return r;
}

static uint32_t esp32_ledc_get_percent(Esp32LEDCState *s, uint32_t value, hwaddr addr)
{
    uint32_t duty_val =  (value >> 4) & ((1 << 20) - 1);
    uint32_t duty_res;
    if (((addr - A_LEDC_HSCH0_DUTY_REG) / 0x14) < 8) {
        /* get duty res for the high speed channel from high speed timer */
        duty_res = s->duty_res[(s->channel_conf0_reg[(addr - A_LEDC_HSCH0_DUTY_REG) / 0x14] & ((1 << 2) - 1))];
    } else {
        /* get duty res for the low speed channel from low speed timer */
        duty_res = s->duty_res[((s->channel_conf0_reg[(addr - A_LEDC_HSCH0_DUTY_REG) / 0x14]) & ((1 << 2) - 1)) + 4];
    }
    return duty_res ? (100 * duty_val / ((2 << (duty_res - 1)) - 1)) : 0;
}

static void esp32_ledc_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned int size)
{
    Esp32LEDCState *s = ESP32_LEDC(opaque);
    if (addr < sizeof(s->regs)) {
        uint32_t v = (uint32_t)value;
        if (addr < A_LEDC_HSTIMER0_CONF_REG) {
            const unsigned off = addr % 0x14;
            if (off == 0xc) {
                v &= ~BIT(31);      /* CHn_CONF1.DUTY_START: applied at once, self-clears */
            } else if (off == 0x0 && addr >= A_LEDC_LSCH0_CONF0_REG) {
                v &= ~BIT(4);       /* LSCHn_CONF0.PARA_UP self-clears */
            }
        } else if (addr >= A_LEDC_LSTIMER0_CONF_REG && addr <= A_LEDC_LSTIMER3_CONF_REG &&
                   ((addr - A_LEDC_LSTIMER0_CONF_REG) % 8) == 0) {
            v &= ~BIT(26);          /* LSTIMERn_CONF.PARA_UP self-clears */
        }
        s->regs[addr / 4] = v;
    }
    switch (addr) {
    case A_LEDC_HSTIMER0_CONF_REG ... A_LEDC_LSTIMER3_CONF_REG:
        /* get duty resolution from timer config */
        if (((uint32_t)value & ((1 << 4) - 1)) != 0) {
            s->duty_res[(addr - A_LEDC_HSTIMER0_CONF_REG) / 0x8] = (uint32_t)value & ((1 << 4) - 1);
        }
        s->timer_conf_reg[(addr - A_LEDC_HSTIMER0_CONF_REG) / 0x8] = value;
        break;

    case A_LEDC_HSCH0_CONF0_REG:
    case A_LEDC_HSCH1_CONF0_REG:
    case A_LEDC_HSCH2_CONF0_REG:
    case A_LEDC_HSCH3_CONF0_REG:
    case A_LEDC_HSCH4_CONF0_REG:
    case A_LEDC_HSCH5_CONF0_REG:
    case A_LEDC_HSCH6_CONF0_REG:
    case A_LEDC_HSCH7_CONF0_REG:
    case A_LEDC_LSCH0_CONF0_REG:
    case A_LEDC_LSCH1_CONF0_REG:
    case A_LEDC_LSCH2_CONF0_REG:
    case A_LEDC_LSCH3_CONF0_REG:
    case A_LEDC_LSCH4_CONF0_REG:
    case A_LEDC_LSCH5_CONF0_REG:
    case A_LEDC_LSCH6_CONF0_REG:
    case A_LEDC_LSCH7_CONF0_REG:
        s->channel_conf0_reg[(addr - A_LEDC_HSCH0_CONF0_REG) / 0x14] = value;
        break;

    case A_LEDC_HSCH0_DUTY_REG:
    case A_LEDC_HSCH1_DUTY_REG:
    case A_LEDC_HSCH2_DUTY_REG:
    case A_LEDC_HSCH3_DUTY_REG:
    case A_LEDC_HSCH4_DUTY_REG:
    case A_LEDC_HSCH5_DUTY_REG:
    case A_LEDC_HSCH6_DUTY_REG:
    case A_LEDC_HSCH7_DUTY_REG:
    case A_LEDC_LSCH0_DUTY_REG:
    case A_LEDC_LSCH1_DUTY_REG:
    case A_LEDC_LSCH2_DUTY_REG:
    case A_LEDC_LSCH3_DUTY_REG:
    case A_LEDC_LSCH4_DUTY_REG:
    case A_LEDC_LSCH5_DUTY_REG:
    case A_LEDC_LSCH6_DUTY_REG:
    case A_LEDC_LSCH7_DUTY_REG:
    {
        const unsigned ch = (addr - A_LEDC_HSCH0_DUTY_REG) / 0x14;
        const uint32_t timer = (s->channel_conf0_reg[ch] & 0x3) + (ch < 8 ? 0 : 4);
        const uint32_t res = s->duty_res[timer];
        const uint32_t duty_val = ((uint32_t)value >> 4) & ((1u << 20) - 1);
        s->duty_raw[ch] = duty_val;
        if (res) {
            double pct = 100.0 * (double)duty_val / (double)(1u << res);
            s->duty_percent[ch] = pct > 100.0 ? 100.0 : pct;
        } else {
            s->duty_percent[ch] = 0.0;
        }
        led_set_intensity(&s->led[ch], esp32_ledc_get_percent(s, value, addr));
        break;
    }
    }

}

static const MemoryRegionOps esp32_ledc_ops = {
        .read =  esp32_ledc_read,
        .write = esp32_ledc_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_ledc_realize(DeviceState *dev, Error **errp)
{
    Esp32LEDCState *s = ESP32_LEDC(dev);
    for (int i = 0; i < ESP32_LEDC_CHANNEL_CNT; i++) {
        qdev_realize(DEVICE(&s->led[i]), NULL, &error_fatal);
    }
}

static void esp32_ledc_init(Object *obj)
{
    Esp32LEDCState *s = ESP32_LEDC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    memory_region_init_io(&s->iomem, obj, &esp32_ledc_ops, s,
                          TYPE_ESP32_LEDC, ESP32_LEDC_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    for (int i = 0; i < ESP32_LEDC_CHANNEL_CNT; i++) {
        object_initialize_child(obj, g_strdup_printf("led%d", i + 1), &s->led[i], TYPE_LED);
        s->led[i].color = (char *)"blue";
    }
}

static void esp32_ledc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = esp32_ledc_realize;
}

static const TypeInfo esp32_ledc_info = {
        .name = TYPE_ESP32_LEDC,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Esp32LEDCState),
        .instance_init = esp32_ledc_init,
        .class_init = esp32_ledc_class_init
};

static void esp32_ledc_register_types(void)
{
    type_register_static(&esp32_ledc_info);
}

type_init(esp32_ledc_register_types)
