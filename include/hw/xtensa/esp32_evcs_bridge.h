#pragma once

#include "hw/xtensa/esp32.h"

/* Attach the EVCS hardware bridge (no-op unless configured). */
void esp32_evcs_bridge_init(Esp32SocState *s);
