/*
 * EVCS hardware bridge for the ESP32 machine.
 *
 * Exchanges JSON lines with an external "hardware" sidecar (simulated car and
 * meter, or a real charger) over a QEMU chardev, so the Victron EVCS firmware
 * sees plausible ADC and GPIO inputs and its outputs can drive the sidecar.
 *
 * In  (sidecar -> QEMU), one object per line:
 *   {"type":"sensor_snapshot", "adc_raw":{"adc1_0":2048,...},
 *    "digital_inputs":{"19":false,...}, ...}
 * Out (QEMU -> sidecar), on change and at least once per second:
 *   {"type":"firmware_output","seq":N,"timestamp_ms":T,
 *    "gpio_outputs":{"0":false,...},"gpio_output_enables":{"0":false,...},
 *    "ledc_percent":{"0":0.0,...}}
 *
 * The chardev is taken from "-chardev ...,id=evcsbridge". When that is absent
 * and EVCS_BRIDGE_SOCKET is set, a reconnecting Unix socket client to that
 * path is created.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qapi/qmp/qdict.h"
#include "qapi/qmp/qnum.h"
#include "qapi/qmp/qbool.h"
#include "qapi/qmp/qjson.h"
#include "qapi/qmp/qobject.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "hw/xtensa/esp32.h"
#include "hw/xtensa/esp32_evcs_bridge.h"

#define EVCS_BRIDGE_CHARDEV_ID "evcsbridge"
#define EVCS_BRIDGE_TICK_MS 20
#define EVCS_BRIDGE_HEARTBEAT_MS 1000
#define EVCS_BRIDGE_MAX_LINE (64 * 1024)
#define EVCS_GPIO_COUNT 40

typedef struct EvcsBridge {
    Esp32SocState *soc;
    CharBackend chr;
    QEMUTimer *timer;
    GString *rx;
    GString *last_payload;
    int64_t last_sent_ms;
    uint64_t seq;
    uint64_t snapshots;
    bool connected;
    bool trace;
} EvcsBridge;

static EvcsBridge *evcs_bridge;

static bool parse_bool_obj(QObject *o, bool *out)
{
    QBool *b = qobject_to(QBool, o);
    if (b) {
        *out = qbool_get_bool(b);
        return true;
    }
    QNum *n = qobject_to(QNum, o);
    if (n) {
        *out = qnum_get_double(n) != 0.0;
        return true;
    }
    return false;
}

static void evcs_bridge_apply_snapshot(EvcsBridge *b, QDict *d)
{
    Esp32SocState *s = b->soc;
    QDict *adc = qdict_get_qdict(d, "adc_raw");
    if (adc) {
        for (const QDictEntry *e = qdict_first(adc); e; e = qdict_next(adc, e)) {
            int unit, ch;
            QNum *n = qobject_to(QNum, qdict_entry_value(e));
            if (!n || sscanf(qdict_entry_key(e), "adc%d_%d", &unit, &ch) != 2) {
                continue;
            }
            double v = qnum_get_double(n);
            if (v < 0) {
                v = 0;
            }
            esp32_sens_set_adc_value(&s->sens, unit, ch, (uint16_t)(v > 4095 ? 4095 : v));
        }
    }
    QDict *din = qdict_get_qdict(d, "digital_inputs");
    if (din) {
        for (const QDictEntry *e = qdict_first(din); e; e = qdict_next(din, e)) {
            int pin;
            bool level;
            if (sscanf(qdict_entry_key(e), "%d", &pin) != 1 || pin < 0 || pin >= EVCS_GPIO_COUNT) {
                continue;
            }
            if (!parse_bool_obj(qdict_entry_value(e), &level)) {
                continue;
            }
            uint32_t *word = pin < 32 ? &s->gpio.input0 : &s->gpio.input1;
            uint32_t bit = 1u << (pin & 31);
            if (level) {
                *word |= bit;
            } else {
                *word &= ~bit;
            }
        }
    }
    b->snapshots++;
}

static void evcs_bridge_handle_line(EvcsBridge *b, const char *line)
{
    Error *err = NULL;
    if (!*line) {
        return;
    }
    QObject *obj = qobject_from_json(line, &err);
    if (!obj) {
        if (b->trace) {
            warn_report("evcs-bridge: bad JSON line: %s", error_get_pretty(err));
        }
        error_free(err);
        return;
    }
    QDict *d = qobject_to(QDict, obj);
    if (d) {
        const char *type = qdict_get_try_str(d, "type");
        if (type && strcmp(type, "sensor_snapshot") == 0) {
            evcs_bridge_apply_snapshot(b, d);
        }
    }
    qobject_unref(obj);
}

static int evcs_bridge_can_read(void *opaque)
{
    return 4096;
}

static void evcs_bridge_read(void *opaque, const uint8_t *buf, int size)
{
    EvcsBridge *b = opaque;
    for (int i = 0; i < size; i++) {
        if (buf[i] == '\n') {
            evcs_bridge_handle_line(b, b->rx->str);
            g_string_truncate(b->rx, 0);
        } else if (b->rx->len < EVCS_BRIDGE_MAX_LINE) {
            g_string_append_c(b->rx, (char)buf[i]);
        }
    }
}

static GString *evcs_bridge_build_payload(EvcsBridge *b)
{
    Esp32SocState *s = b->soc;
    GString *p = g_string_new(NULL);
    const uint32_t out[2] = { s->gpio.mem[A_GPIO_OUT / 4], s->gpio.mem[A_GPIO_OUT1 / 4] };
    const uint32_t en[2] = { s->gpio.mem[A_GPIO_ENABLE / 4], s->gpio.mem[A_GPIO_ENABLE1 / 4] };

    g_string_append(p, "\"gpio_outputs\":{");
    for (int pin = 0; pin < EVCS_GPIO_COUNT; pin++) {
        g_string_append_printf(p, "%s\"%d\":%s", pin ? "," : "", pin,
                               (out[pin / 32] >> (pin & 31)) & 1 ? "true" : "false");
    }
    g_string_append(p, "},\"gpio_output_enables\":{");
    for (int pin = 0; pin < EVCS_GPIO_COUNT; pin++) {
        g_string_append_printf(p, "%s\"%d\":%s", pin ? "," : "", pin,
                               (en[pin / 32] >> (pin & 31)) & 1 ? "true" : "false");
    }
    g_string_append(p, "},\"ledc_percent\":{");
    for (int ch = 0; ch < ESP32_LEDC_CHANNEL_CNT; ch++) {
        g_string_append_printf(p, "%s\"%d\":%.3f", ch ? "," : "", ch, s->ledc.duty_percent[ch]);
    }
    g_string_append(p, "}");
    return p;
}

static void evcs_bridge_send(EvcsBridge *b, bool force)
{
    if (!b->connected) {
        return;
    }
    int64_t now = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);
    GString *payload = evcs_bridge_build_payload(b);
    bool changed = !b->last_payload || !g_string_equal(payload, b->last_payload);
    if (!force && !changed && now - b->last_sent_ms < EVCS_BRIDGE_HEARTBEAT_MS) {
        g_string_free(payload, true);
        return;
    }
    b->seq++;
    int64_t wall_ms = g_get_real_time() / 1000;
    g_autofree char *line = g_strdup_printf(
        "{\"type\":\"firmware_output\",\"seq\":%" PRIu64 ",\"timestamp_ms\":%" PRId64
        ",\"guest_clock_ms\":%" PRId64 ",%s}\n",
        b->seq, wall_ms, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), payload->str);
    qemu_chr_fe_write_all(&b->chr, (const uint8_t *)line, strlen(line));
    b->last_sent_ms = now;
    if (b->last_payload) {
        g_string_free(b->last_payload, true);
    }
    b->last_payload = payload;
}

static void evcs_bridge_tick(void *opaque)
{
    EvcsBridge *b = opaque;
    evcs_bridge_send(b, false);
    timer_mod(b->timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + EVCS_BRIDGE_TICK_MS);
}

static void evcs_bridge_event(void *opaque, QEMUChrEvent event)
{
    EvcsBridge *b = opaque;
    switch (event) {
    case CHR_EVENT_OPENED:
        b->connected = true;
        g_string_truncate(b->rx, 0);
        info_report("evcs-bridge: sidecar connected");
        evcs_bridge_send(b, true);
        break;
    case CHR_EVENT_CLOSED:
        b->connected = false;
        info_report("evcs-bridge: sidecar disconnected");
        break;
    default:
        break;
    }
}

void esp32_evcs_bridge_init(Esp32SocState *s)
{
    Chardev *chr = qemu_chr_find(EVCS_BRIDGE_CHARDEV_ID);
    if (!chr) {
        const char *path = getenv("EVCS_BRIDGE_SOCKET");
        if (!path || !*path) {
            return;
        }
        g_autofree char *spec = g_strdup_printf("unix:%s,reconnect-ms=500", path);
        chr = qemu_chr_new(EVCS_BRIDGE_CHARDEV_ID, spec, NULL);
        if (!chr) {
            error_report("evcs-bridge: cannot create chardev for %s", path);
            return;
        }
    }

    EvcsBridge *b = g_new0(EvcsBridge, 1);
    b->soc = s;
    b->rx = g_string_new(NULL);
    b->trace = getenv("EVCS_BRIDGE_TRACE") != NULL;
    if (!qemu_chr_fe_init(&b->chr, chr, &error_fatal)) {
        g_free(b);
        return;
    }
    qemu_chr_fe_set_handlers(&b->chr, evcs_bridge_can_read, evcs_bridge_read,
                             evcs_bridge_event, NULL, b, NULL, true);
    b->timer = timer_new_ms(QEMU_CLOCK_REALTIME, evcs_bridge_tick, b);
    timer_mod(b->timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + EVCS_BRIDGE_TICK_MS);
    evcs_bridge = b;
    info_report("evcs-bridge: attached to chardev '%s'", chr->label);
}
