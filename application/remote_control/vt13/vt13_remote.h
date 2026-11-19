#ifndef VT13_REMOTE_H
#define VT13_REMOTE_H

#include <stdbool.h>
#include <stdint.h>

#define VT13_FRAME_SIZE 21U
#define VT13_INPUT_TIMEOUT_MS 100U

/* 保留遥控量的原始数值，不映射为 DT7 双拨杆或车辆命令。 */
typedef struct {
    uint16_t channel[4], wheel;
    uint8_t mode_switch; /* C=0，N=1，S=2。 */
    bool pause, function_left, function_right, trigger;
    int16_t mouse_x, mouse_y, mouse_z;
    uint8_t mouse_left, mouse_right, mouse_middle;
    uint16_t keyboard;
    uint32_t sequence, update_time_ms;
    bool valid;
    float normalized[4]; /* 与 channel 同序，范围 [-1, 1]，中位为 0。 */
} Vt13Input;

typedef struct {
    uint32_t bytes, valid_frames, crc_errors, invalid_frames;
} Vt13ParserStats;

typedef struct {
    Vt13Input input;
    Vt13ParserStats stats;
    uint8_t buffer[VT13_FRAME_SIZE], length;
    uint32_t last_byte_ms;
    bool have_bytes;
} Vt13Parser;

/* 纯协议层；调用方负责串行喂入数据，以及读取时的并发保护。 */
uint16_t Vt13Crc16(const uint8_t *data, uint16_t length);
void Vt13ParserInit(Vt13Parser *parser);
void Vt13ParserInvalidate(Vt13Parser *parser);
void Vt13ParserFeed(Vt13Parser *parser, const uint8_t *data, uint16_t length, uint32_t now_ms);
void Vt13ParserRead(const Vt13Parser *parser, Vt13Input *out, uint32_t now_ms);
#endif
