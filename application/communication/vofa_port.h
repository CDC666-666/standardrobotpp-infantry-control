#ifndef VOFA_PORT_H
#define VOFA_PORT_H
#include <stdint.h>

typedef enum { VOFA_PORT_OK, VOFA_PORT_BUSY, VOFA_PORT_ERROR } VofaPortResult;
uint32_t VofaPortNow(void);
uint32_t VofaPortLock(void);
void VofaPortUnlock(uint32_t key);
int VofaPortInit(const void *buffer, uint16_t capacity);
int VofaPortReady(void);
VofaPortResult VofaPortStart(uint8_t *buffer, uint16_t length);
// Bounded, task-context recovery. Returns only after DMA no longer owns memory.
int VofaPortRecover(void);
#endif
