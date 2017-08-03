#include <stdint.h>
#include "parking.h"
#include "parking_buffer.h"

#if ENABLE_PARKING_BUFFER_DEBUG
#include <stdio.h>
#include "pni_config.h"
#endif /* ENABLE_PARKING_BUFFER_DEBUG */

static void parking_buffer_push(void *self, PNI_ParkingPacket *pkt)
{
    PNI_ParkingBufferIface *iface = self;
    uint32_t index = iface->buffer.in++ & (PNI_PARKING_BUFFER_COUNT - 1);

    iface->buffer.data[index]= *pkt;

#if ENABLE_PARKING_BUFFER_DEBUG
    iface->debug(self);
#endif /* ENABLE_PARKING_BUFFER_DEBUG */
}

static void parking_buffer_pop(void *self, PNI_ParkingPacket *pkt)
{
    PNI_ParkingBufferIface *iface = self;
    uint32_t index = iface->buffer.out++ & (PNI_PARKING_BUFFER_COUNT - 1);

    *pkt = iface->buffer.data[index];

#if ENABLE_PARKING_BUFFER_DEBUG
    iface->debug(self);
#endif /* ENABLE_PARKING_BUFFER_DEBUG */
}

static void parking_buffer_get(void *self, PNI_ParkingPacket *pkt,
        uint32_t pos)
{
    PNI_ParkingBufferIface *iface = self;
    uint32_t index = (iface->buffer.out + pos)
            & (PNI_PARKING_BUFFER_COUNT - 1);

    *pkt = iface->buffer.data[index];
}

static int parking_buffer_len(void *self)
{
    PNI_ParkingBufferIface *iface = self;
    return (iface->buffer.in - iface->buffer.out);
}

static int parking_buffer_is_empty(void *self)
{
    PNI_ParkingBufferIface *iface = self;
    return (iface->len(iface) == 0);
}

static int parking_buffer_is_full(void *self)
{
    PNI_ParkingBufferIface *iface = self;
    return (iface->len(iface) == PNI_PARKING_BUFFER_COUNT);
}

static void parking_buffer_adv_in(void *self, uint32_t count)
{
    PNI_ParkingBufferIface *iface = self;
    iface->buffer.in += count;
}

static void parking_buffer_adv_out(void *self, uint32_t count)
{
    PNI_ParkingBufferIface *iface = self;
    iface->buffer.out += count;
}

#if ENABLE_PARKING_BUFFER_DEBUG
static void parking_buffer_debug(void *self)
{
    PNI_ParkingBufferIface *iface = self;
    PNI_PRINTF("[ParkingBuffer] in: %u, out: %u, len: %u\r\n",
            iface->buffer.in, iface->buffer.out, iface->len(iface));
}
#endif /* ENABLE_PARKING_BUFFER_DEBUG */

void pni_parking_buffer_init(PNI_ParkingBufferIface *iface)
{
    iface->push = &parking_buffer_push;
    iface->pop = &parking_buffer_pop;
    iface->get = &parking_buffer_get;
    iface->len = &parking_buffer_len;
    iface->is_empty = &parking_buffer_is_empty;
    iface->is_full = &parking_buffer_is_full;
    iface->adv_in = &parking_buffer_adv_in;
    iface->adv_out = &parking_buffer_adv_out;

#if ENABLE_PARKING_BUFFER_DEBUG
    iface->debug = &parking_buffer_debug;
#endif /* ENABLE_PARKING_BUFFER_DEBUG */

    iface->buffer.in = iface->buffer.out = 0;
}

