#ifndef VOFA_CHASSIS_H
#define VOFA_CHASSIS_H

#include "chassis_omni.h"

#if VOFA_ENABLE && CHASSIS_TYPE == CHASSIS_OMNI_WHEEL
void vofa_chassis_update(const Chassis_s *state);
#else
static inline void vofa_chassis_update(const Chassis_s *state) { (void)state; }

#endif

#endif
