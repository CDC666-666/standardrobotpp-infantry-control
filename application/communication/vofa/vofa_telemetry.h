#ifndef VOFA_TELEMETRY_H
#define VOFA_TELEMETRY_H

#include "robot_param.h"
#include <stdint.h>

#if VOFA_ENABLE
void vofa_telemetry_update(uint32_t now);
#endif

#endif
