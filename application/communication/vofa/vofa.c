#include "vofa.h"
#include "vofa_port.h"
#include <string.h>

volatile VofaStats vofa_stats;

#if VOFA_ENABLE
typedef char vofa_float_must_be_32_bits[(sizeof(float) == 4) ? 1 : -1];

// Explicitly selected into RW_IRAM1 by vofa.sct, never CCM (0x10000000).
static uint8_t tx_buffer[VOFA_FRAME_BYTES(VOFA_MAX_CHANNELS)]
    __attribute__((section("VOFA_DMA"), aligned(4)));
static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
static uint32_t started_at, deadline_ms, previous_start;
static uint8_t have_previous, resync_frame, recovery_attempted;

static VofaSendResult StartFrame(uint16_t length, uint8_t resync)
{
    VofaPortResult result;
    uint32_t key = VofaPortLock();
    started_at = VofaPortNow();
    deadline_ms = ((uint32_t)length * 10000UL + VOFA_BAUDRATE - 1UL) / VOFA_BAUDRATE + 10UL;
    resync_frame = resync;
    vofa_stats.state = VOFA_SENDING;
    VofaPortUnlock(key);

    result = VofaPortStart(tx_buffer, length);
    key = VofaPortLock();
    if (result != VOFA_PORT_OK) {
        ++vofa_stats.start_failed;
        if (result == VOFA_PORT_BUSY) ++vofa_stats.busy_skipped;
        vofa_stats.state = VOFA_RECOVER_PENDING;
        VofaPortUnlock(key);
        return result == VOFA_PORT_BUSY ? VOFA_SEND_BUSY : VOFA_SEND_FAULT;
    }
    // Do not overwrite state here: a completion/error IRQ may already have run.
    if (!resync) {
        if (have_previous) vofa_stats.last_interval_ms = started_at - previous_start;
        previous_start = started_at;
        have_previous = 1;
    }
    VofaPortUnlock(key);
    return VOFA_SEND_STARTED;
}

void VofaInit(void)
{
    // Startup only, before the first transmission. No runtime buffer reset.
    memset((void *)&vofa_stats, 0, sizeof(vofa_stats));
    have_previous = resync_frame = recovery_attempted = 0;
    vofa_stats.state = VofaPortInit(tx_buffer, sizeof(tx_buffer)) ? VOFA_IDLE : VOFA_FAULT;
}

VofaSendResult VofaTrySend(const float *values, uint16_t count)
{
    uint16_t i;
    uint32_t bits, key = VofaPortLock();
    if (vofa_stats.state == VOFA_FAULT || vofa_stats.state == VOFA_STOPPED) {
        VofaPortUnlock(key);
        return VOFA_SEND_FAULT;
    }
    if (vofa_stats.state != VOFA_IDLE) {
        ++vofa_stats.busy_skipped;
        VofaPortUnlock(key);
        return VOFA_SEND_BUSY;
    }
    vofa_stats.state = VOFA_PREPARING;
    VofaPortUnlock(key);

    // Even an unexpected mismatch between HAL/hardware and our state must not
    // overwrite memory that DMA might still be reading.
    if (!VofaPortReady()) {
        key = VofaPortLock();
        ++vofa_stats.busy_skipped;
        vofa_stats.state = VOFA_RECOVER_PENDING;
        VofaPortUnlock(key);
        return VOFA_SEND_BUSY;
    }

    if (!values || count < 1 || count > VOFA_MAX_CHANNELS || !VOFA_BANDWIDTH_OK(count)) goto invalid;
    for (i = 0; i < count; ++i) {
        memcpy(&bits, &values[i], sizeof(bits));
        // Reject all NaN/Inf without depending on compiler fast-math behavior.
        if ((bits & 0x7f800000UL) == 0x7f800000UL) goto invalid;
    }
    memcpy(tx_buffer, values, count * sizeof(float));
    memcpy(tx_buffer + count * sizeof(float), tail, sizeof(tail));
    return StartFrame((uint16_t)VOFA_FRAME_BYTES(count), 0);

invalid:
    key = VofaPortLock();
    ++vofa_stats.invalid_data;
    vofa_stats.state = VOFA_IDLE;
    VofaPortUnlock(key);
    return VOFA_SEND_INVALID;
}

void VofaOnTxComplete(void)
{
    uint32_t key = VofaPortLock();
    if (vofa_stats.state == VOFA_SENDING) {
        if (!resync_frame) {
            ++vofa_stats.completed;
            recovery_attempted = 0; // A complete normal frame ends this fault episode.
        }
        vofa_stats.state = VOFA_IDLE;
    }
    VofaPortUnlock(key);
}

void VofaOnError(uint32_t error)
{
    uint32_t key = VofaPortLock();
    ++vofa_stats.transfer_errors;
    vofa_stats.last_error = error;
    if (vofa_stats.state == VOFA_SENDING) vofa_stats.state = VOFA_RECOVER_PENDING;
    VofaPortUnlock(key);
}

void VofaService(void)
{
    uint8_t stop_after_cleanup;
    uint32_t key = VofaPortLock();
    if (vofa_stats.state == VOFA_SENDING && VofaPortNow() - started_at > deadline_ms) {
        ++vofa_stats.timeouts;
        vofa_stats.state = VOFA_RECOVER_PENDING;
    }
    if (vofa_stats.state != VOFA_RECOVER_PENDING) {
        VofaPortUnlock(key);
        return;
    }
    stop_after_cleanup = recovery_attempted;
    recovery_attempted = 1;
    vofa_stats.state = VOFA_RECOVERING;
    VofaPortUnlock(key);

    // No HAL calls or polling while interrupts are globally disabled.
    if (!VofaPortRecover() || stop_after_cleanup) {
        ++vofa_stats.recovery_failed;
        vofa_stats.state = VOFA_FAULT;
        return;
    }
    ++vofa_stats.recoveries;
    // Terminate a possibly partial frame before the next normal sample.
    // The receiver must discard data around the fault/re-sync boundary.
    memcpy(tx_buffer, tail, sizeof(tail));
    (void)StartFrame(sizeof(tail), 1);
}

void VofaRecordMissedPeriods(uint32_t count) { vofa_stats.missed_periods += count; }
#else
void VofaInit(void) { }
void VofaService(void) { }
VofaSendResult VofaTrySend(const float *v, uint16_t n) { (void)v; (void)n; return VOFA_SEND_FAULT; }
void VofaOnTxComplete(void) { }
void VofaOnError(uint32_t e) { (void)e; }
void VofaRecordMissedPeriods(uint32_t n) { (void)n; }
#endif
