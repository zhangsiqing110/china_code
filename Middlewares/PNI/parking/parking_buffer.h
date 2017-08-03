#ifndef PNI_PARKING_BUFFER_H
#define PNI_PARKING_BUFFER_H

#include <stdint.h>
#include "parking.h"

#define ENABLE_PARKING_BUFFER_DEBUG (0)

/* MUST be power of two! */
#define PNI_PARKING_BUFFER_COUNT (32U)

typedef struct pni_parking_buffer {
    uint32_t in;
    uint32_t out;
    PNI_ParkingPacket data[PNI_PARKING_BUFFER_COUNT];
} PNI_ParkingBuffer;

typedef struct pni_parking_buffer_iface {
    void (*push)(void *self, PNI_ParkingPacket *pkt);
    void (*pop)(void *self, PNI_ParkingPacket *pkt);
    void (*get)(void *self, PNI_ParkingPacket *pkt, uint32_t index);
    int  (*len)(void *self);
    int  (*is_empty)(void *self);
    int  (*is_full)(void *self);
    void (*adv_in)(void *self, uint32_t count);
    void (*adv_out)(void *self, uint32_t count);

#if ENABLE_PARKING_BUFFER_DEBUG
    void (*debug)(void *self);
#endif /* ENABLE_PARKING_BUFFER_DEBUG */

    PNI_ParkingBuffer buffer;
} PNI_ParkingBufferIface;

void pni_parking_buffer_init(PNI_ParkingBufferIface *iface);

#endif /* PNI_PARKING_BUFFER_H */
