#ifndef VOFA_H
#define VOFA_H

#include <stdint.h>
#include "robot_param.h"

#define VOFA_MAX_CHANNELS 16U
#define VOFA_FRAME_BYTES(n) (4UL * ((n) + 1UL))
#define VOFA_BANDWIDTH_OK(n) \
    (VOFA_FRAME_BYTES(n) * 10ULL * 1000ULL * 100ULL <= VOFA_BAUDRATE * VOFA_PERIOD_MS * 70ULL)

typedef enum {
    VOFA_SEND_STARTED, VOFA_SEND_BUSY, VOFA_SEND_INVALID, VOFA_SEND_FAULT
} VofaSendResult;

typedef enum {
    VOFA_STOPPED, VOFA_IDLE, VOFA_PREPARING, VOFA_SENDING,
    VOFA_RECOVER_PENDING, VOFA_RECOVERING, VOFA_FAULT
} VofaState;

typedef struct {
    uint32_t completed, busy_skipped, start_failed, transfer_errors;
    uint32_t timeouts, invalid_data, missed_periods, recoveries;
    uint32_t recovery_failed, last_interval_ms, last_error;
    VofaState state;
} VofaStats;

// Debugger-visible; not automatically added to the wire protocol.
extern volatile VofaStats vofa_stats;

// Task-only, single owner: the communication task.
void VofaInit(void);
void VofaService(void);
VofaSendResult VofaTrySend(const float *values, uint16_t count);
void VofaRecordMissedPeriods(uint32_t count);
// Called only by the USART1 HAL callbacks.
void VofaOnTxComplete(void);
void VofaOnError(uint32_t error);

#if VOFA_ENABLE
// Each expression is evaluated once. VofaTrySend copies before returning;
// this local array is NEVER passed directly to DMA. AC5/C99 compatible checks.
#define VOFA_SEND(...) do { \
    const float vofa_values_[] = { __VA_ARGS__ }; \
    enum { vofa_count_ = sizeof(vofa_values_) / sizeof(vofa_values_[0]) }; \
    typedef char vofa_count_check_[(vofa_count_ >= 1 && vofa_count_ <= VOFA_MAX_CHANNELS) ? 1 : -1]; \
    typedef char vofa_bandwidth_check_[VOFA_BANDWIDTH_OK(vofa_count_) ? 1 : -1]; \
    (void)sizeof(vofa_count_check_); \
    (void)sizeof(vofa_bandwidth_check_); \
    (void)VofaTrySend(vofa_values_, (uint16_t)vofa_count_); \
} while (0)
#else
#define VOFA_SEND(...) ((void)0)
#endif

#endif
