#ifndef VOFA_SHOOT_H
#define VOFA_SHOOT_H

#include "shoot_fric_trigger.h"

#if VOFA_ENABLE && SHOOT_TYPE == SHOOT_FRIC_TRIGGER
void vofa_shoot_update(const Shoot_s *state);
#else
static inline void vofa_shoot_update(const Shoot_s *state) { (void)state; }

#endif

#endif
