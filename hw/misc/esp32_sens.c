#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32_sens.h"

/* Only a tiny subset of SENS is needed to unblock typical ESP-IDF ADC flows. */

enum {
    SENS_REG_SIZE = 0x1000,
};

/* SENS_SAR_MEAS_START1_REG (DR_REG_SENS_BASE + 0x54) */
#define SENS_SAR_MEAS_START1      0x54
#define SENS_MEAS1_START_FORCE    BIT(18)
#define SENS_MEAS1_START_SAR      BIT(17)
#define SENS_MEAS1_DONE_SAR       BIT(16)
#define SENS_MEAS1_DATA_MASK      0x0000ffffu
#define SENS_SAR_EN_PAD_SHIFT     19
#define SENS_SAR_EN_PAD_MASK      0xfffu

/* SENS_SAR_MEAS_START2_REG (DR_REG_SENS_BASE + 0x94) */
#define SENS_SAR_MEAS_START2      0x94
#define SENS_MEAS2_START_FORCE    BIT(18)
#define SENS_MEAS2_START_SAR      BIT(17)
#define SENS_MEAS2_DONE_SAR       BIT(16)
#define SENS_MEAS2_DATA_MASK      0x0000ffffu

static bool esp32_sens_debug_enabled(void)
{
    static int enabled = -1;
    if (enabled < 0) {
        enabled = getenv("ESP32_SENS_DEBUG") != NULL;
    }
    return enabled;
}

void esp32_sens_set_adc_value(Esp32SensState *s, int unit, int channel, uint16_t value)
{
    if (channel < 0 || channel >= ESP32_SENS_ADC_CHANNELS) {
        return;
    }
    if (value > 0xfff) {
        value = 0xfff;
    }
    if (unit == 1) {
        s->adc1_value[channel] = value;
    } else if (unit == 2) {
        s->adc2_value[channel] = value;
    }
}

/* Channel selected by the one-hot SARx_EN_PAD field of a MEAS_STARTx write. */
static int esp32_sens_pad_channel(uint32_t reg)
{
    uint32_t pad = (reg >> SENS_SAR_EN_PAD_SHIFT) & SENS_SAR_EN_PAD_MASK;
    if (pad == 0) {
        return -1;
    }
    return ctz32(pad);
}

static uint16_t esp32_sens_sample(Esp32SensState *s, int unit, uint32_t reg)
{
    int ch = esp32_sens_pad_channel(reg);
    uint16_t v = 0x0800u;
    if (ch >= 0 && ch < ESP32_SENS_ADC_CHANNELS) {
        if (unit == 1) {
            v = s->adc1_value[ch];
            s->adc1_reads[ch]++;
        } else {
            v = s->adc2_value[ch];
            s->adc2_reads[ch]++;
        }
    }
    if (esp32_sens_debug_enabled()) {
        printf("esp32_sens: adc%d ch%d -> %u\n", unit, ch, v);
    }
    return v;
}

static uint64_t esp32_sens_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32SensState *s = ESP32_SENS(opaque);

    if (size != 4 || (addr & 0x3) != 0 || addr >= SENS_REG_SIZE) {
        return 0;
    }

    uint32_t r = s->mem[addr / 4];

    if (esp32_sens_debug_enabled()) {
        printf("esp32_sens_read 0x%" HWADDR_PRIx " -> 0x%08" PRIx32 "\n", addr, r);
    }

    return r;
}

static void esp32_sens_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    Esp32SensState *s = ESP32_SENS(opaque);

    if (size != 4 || (addr & 0x3) != 0 || addr >= SENS_REG_SIZE) {
        return;
    }

    if (esp32_sens_debug_enabled()) {
        printf("esp32_sens_write 0x%" HWADDR_PRIx " <- 0x%08" PRIx64 "\n", addr, value);
    }

    switch (addr) {
    case SENS_SAR_MEAS_START1: {
        /*
         * Firmware sets MEAS1_START_SAR and then spins until MEAS1_DONE_SAR is
         * asserted. We don't emulate ADC conversion, so complete immediately.
         */
        const uint32_t prev = s->mem[addr / 4];
        const uint32_t ro_mask = SENS_MEAS1_DONE_SAR | SENS_MEAS1_DATA_MASK;
        uint32_t reg = (prev & ro_mask) | ((uint32_t)value & ~ro_mask);

        const bool start = (value & SENS_MEAS1_START_SAR) != 0;
        if (start) {
            reg &= ~SENS_MEAS1_START_SAR; /* self-clearing */
            reg &= ~SENS_MEAS1_DONE_SAR;

            /* Per-channel injected sample (12-bit), mid-scale by default. */
            reg = (reg & ~SENS_MEAS1_DATA_MASK) | esp32_sens_sample(s, 1, reg);
            reg |= SENS_MEAS1_DONE_SAR;
        }

        s->mem[addr / 4] = reg;
        break;
    }
    case SENS_SAR_MEAS_START2: {
        /*
         * Firmware sets MEAS2_START_SAR and then spins until MEAS2_DONE_SAR is
         * asserted. We don't emulate ADC conversion, so complete immediately.
         */
        const uint32_t prev = s->mem[addr / 4];
        const uint32_t ro_mask = SENS_MEAS2_DONE_SAR | SENS_MEAS2_DATA_MASK;
        uint32_t reg = (prev & ro_mask) | ((uint32_t)value & ~ro_mask);

        const bool start = (value & SENS_MEAS2_START_SAR) != 0;
        if (start) {
            reg &= ~SENS_MEAS2_START_SAR; /* self-clearing */
            reg &= ~SENS_MEAS2_DONE_SAR;

            /* Per-channel injected sample (12-bit), mid-scale by default. */
            reg = (reg & ~SENS_MEAS2_DATA_MASK) | esp32_sens_sample(s, 2, reg);
            reg |= SENS_MEAS2_DONE_SAR;
        }

        s->mem[addr / 4] = reg;
        break;
    }
    default:
        s->mem[addr / 4] = (uint32_t)value;
        break;
    }
}

static const MemoryRegionOps esp32_sens_ops = {
    .read = esp32_sens_read,
    .write = esp32_sens_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_sens_init(Object *obj)
{
    Esp32SensState *s = ESP32_SENS(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_sens_ops, s, TYPE_ESP32_SENS, SENS_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    memset(s->mem, 0, sizeof(s->mem));
    for (int i = 0; i < ESP32_SENS_ADC_CHANNELS; i++) {
        s->adc1_value[i] = 0x0800u;
        s->adc2_value[i] = 0x0800u;
    }
}

static const TypeInfo esp32_sens_info = {
    .name = TYPE_ESP32_SENS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32SensState),
    .instance_init = esp32_sens_init,
};

static void esp32_sens_register_types(void)
{
    type_register_static(&esp32_sens_info);
}

type_init(esp32_sens_register_types)
