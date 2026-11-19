#include "vt13_remote.h"

#include <string.h>

#define VT13_CHANNEL_MIN 364U
#define VT13_CHANNEL_CENTER 1024U
#define VT13_CHANNEL_MAX 1684U

/* 按官网 Example_Code_for_Data_Frame_and_Validation.c：反射多项式 0x8408，
 * 初值 0xffff、无最终异或，帧尾低字节在前。PDF 的 CCITT-FALSE 文案与示例冲突。
 * 来源：https://rm-static.djicdn.com/tem/17348/Example_Code_for_Data_Frame_and_Validation.c
 */
uint16_t Vt13Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xffffU;
    for (uint16_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8U; ++bit)
            crc = (uint16_t)((crc >> 1) ^ ((crc & 1U) ? 0x8408U : 0U));
    }
    return crc;
}

static uint16_t Vt13Bits(const uint8_t *data, uint16_t offset, uint8_t count)
{
    uint16_t value = 0;
    for (uint8_t i = 0; i < count; ++i)
        value |= (uint16_t)(((data[(offset + i) / 8U] >> ((offset + i) % 8U)) & 1U) << i);
    return value;
}

static int16_t Vt13SignedWord(const uint8_t *data)
{
    uint16_t value = (uint16_t)(data[0] | ((uint16_t)data[1] << 8));
    return value <= 0x7fffU ? (int16_t)value : (int16_t)((int32_t)value - 65536);
}

static void Vt13ClearControls(Vt13Input *input)
{
    uint32_t sequence = input->sequence, time = input->update_time_ms;
    memset(input, 0, sizeof(*input));
    for (uint8_t i = 0; i < 4U; ++i) input->channel[i] = VT13_CHANNEL_CENTER;
    input->wheel = VT13_CHANNEL_CENTER;
    input->sequence = sequence;
    input->update_time_ms = time;
}

void Vt13ParserInit(Vt13Parser *parser)
{
    memset(parser, 0, sizeof(*parser));
    Vt13ClearControls(&parser->input);
}

void Vt13ParserInvalidate(Vt13Parser *parser)
{
    parser->length = 0;
    parser->have_bytes = false;
    Vt13ClearControls(&parser->input);
}

static bool Vt13Decode(Vt13Parser *parser, uint32_t now_ms)
{
    Vt13Input next = {0};
    const uint8_t *data = parser->buffer;
    for (uint8_t i = 0; i < 4U; ++i) {
        next.channel[i] = Vt13Bits(data, (uint16_t)(16U + 11U * i), 11U);
        if (next.channel[i] < VT13_CHANNEL_MIN || next.channel[i] > VT13_CHANNEL_MAX) return false;
        next.normalized[i] = ((float)next.channel[i] - (float)VT13_CHANNEL_CENTER) /
                             (float)(VT13_CHANNEL_MAX - VT13_CHANNEL_CENTER);
    }
    next.mode_switch = (uint8_t)Vt13Bits(data, 60U, 2U);
    next.pause = Vt13Bits(data, 62U, 1U) != 0;
    next.function_left = Vt13Bits(data, 63U, 1U) != 0;
    next.function_right = Vt13Bits(data, 64U, 1U) != 0;
    next.wheel = Vt13Bits(data, 65U, 11U);
    next.trigger = Vt13Bits(data, 76U, 1U) != 0;
    next.mouse_x = Vt13SignedWord(&data[10]);
    next.mouse_y = Vt13SignedWord(&data[12]);
    next.mouse_z = Vt13SignedWord(&data[14]);
    next.mouse_left = data[16] & 3U;
    next.mouse_right = (data[16] >> 2) & 3U;
    next.mouse_middle = (data[16] >> 4) & 3U;
    next.keyboard = (uint16_t)(data[17] | ((uint16_t)data[18] << 8));
    if (next.mode_switch > 2U || next.wheel < VT13_CHANNEL_MIN || next.wheel > VT13_CHANNEL_MAX ||
        next.mouse_left > 1U || next.mouse_right > 1U || next.mouse_middle > 1U) return false;
    next.sequence = parser->input.sequence + 1U;
    next.update_time_ms = now_ms;
    next.valid = true;
    parser->input = next;
    ++parser->stats.valid_frames;
    return true;
}

void Vt13ParserFeed(Vt13Parser *parser, const uint8_t *data, uint16_t length, uint32_t now_ms)
{
    if (length == 0U || data == NULL) return;
    /* 不把长时间断流前后的残片拼成一帧。 */
    if (parser->have_bytes && now_ms - parser->last_byte_ms >= VT13_INPUT_TIMEOUT_MS)
        parser->length = 0;
    parser->last_byte_ms = now_ms;
    parser->have_bytes = true;
    parser->stats.bytes += length;
    for (uint16_t i = 0; i < length; ++i) {
        parser->buffer[parser->length++] = data[i];
        for (;;) {
            if (parser->length == 0U) break;
            if (parser->buffer[0] == 0xa9U &&
                (parser->length == 1U || parser->buffer[1] == 0x53U)) {
                if (parser->length < VT13_FRAME_SIZE) break;
                uint16_t expected = Vt13Crc16(parser->buffer, VT13_FRAME_SIZE - 2U);
                uint16_t received = (uint16_t)(parser->buffer[19] | ((uint16_t)parser->buffer[20] << 8));
                if (expected == received) {
                    if (!Vt13Decode(parser, now_ms)) ++parser->stats.invalid_frames;
                    else { parser->length = 0; break; }
                } else ++parser->stats.crc_errors;
            }
            /* 错帧逐字节重新找头，保留内嵌或跨缓冲的下一个帧头。 */
            --parser->length;
            memmove(parser->buffer, parser->buffer + 1, parser->length);
        }
    }
}

void Vt13ParserRead(const Vt13Parser *parser, Vt13Input *out, uint32_t now_ms)
{
    *out = parser->input;
    if (!out->valid || now_ms - out->update_time_ms >= VT13_INPUT_TIMEOUT_MS)
        Vt13ClearControls(out);
}
