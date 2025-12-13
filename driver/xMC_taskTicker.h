#pragma once

#include "platform/driver_core/MCADCPWM3P/MCADCPWM3P_timeBase.h"

#define XMC_TICK_FREQ_HZ XMC_TICKFREQ_HZ
#define XMC_TICK_PERIOD_MS (1000.0f / XMC_TICK_HZ)
#define XMC_TICK_PERIOD_US (1000000.0f / XMC_TICK_HZ)