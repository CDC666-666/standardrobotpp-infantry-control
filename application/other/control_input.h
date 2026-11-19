#ifndef CONTROL_INPUT_H
#define CONTROL_INPUT_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CONTROL_INPUT_NONE,
    CONTROL_INPUT_DT7,
    CONTROL_INPUT_SBUS
} ControlInputSource_e;

/* A task owns its copy. Never change the receiver's RC_ctrl_t from a control task. */
typedef struct {
    int16_t ch[5];
    uint8_t sw[2];
    int16_t mouse_x, mouse_y, mouse_z;
    bool mouse_left, mouse_right;
    uint16_t keyboard;
    uint32_t sequence;
    uint32_t update_time_ms;
    ControlInputSource_e source;
    bool valid;
} ControlInput_t;

typedef struct {
    bool down;
    bool enabled;
} ControlButton_t;

/* Keep tracking a held key while disabled; enabling alone must not create an edge. */
static inline bool ControlButtonPressed(ControlButton_t *button, bool down, bool enabled)
{
    bool pressed = enabled && button->enabled && down && !button->down;
    button->down = down;
    button->enabled = enabled;
    return pressed;
}

void ControlInputRead(ControlInput_t *input);

#endif
