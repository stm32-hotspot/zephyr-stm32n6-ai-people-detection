/**
 ******************************************************************************
 * @file    aiPbIO.c
 * @author  STMicroelectronics
 * @brief   Low Level ProtoBuffer IO functions for COM stack
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 - 2026 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software is licensed under terms that can be found in the LICENSE file in
 * the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/*
 * Description:
 *
 * Low-Level IO functions to port the nano PB stack
 *
 * History:
 *  - v1.0 - initial version. Based on a split of the original aiTestUtility.c
 *           (v1.4) file previous (aiValidation v3.2)
 */

/* System headers */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include <aiPbIO.h>

#include <aiTestUtility.h>

#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <stm32msg.pb.h>

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif


#if (defined(STM32N657xx) || defined(STM32H7P5xx))
#include "app_config.h"
#endif


#define _PACKET_HEADER_SIZE sizeof(uint16_t)

#define _PACKET_PAYLOAD_IN_SIZE  ((EnumLowLevelIO)EnumLowLevelIO_IO_IN_PACKET_SIZE)
#define _PACKET_PAYLOAD_OUT_SIZE ((EnumLowLevelIO)EnumLowLevelIO_IO_OUT_PACKET_SIZE)


#if defined(USE_USB_CDC_CLASS) && USE_USB_CDC_CLASS == 1

#if defined(USE_USB_PACKET_SIZE)

#if USE_USB_PACKET_SIZE > 4094
/* see also  _MAX_USB_USER_ELEM for ST USB stack in aiTestUtility.c file */
#error USE_USB_PACKET_SIZE should be inferior to 4096.
#endif

#define _PACKET_PAYLOAD_IN_SIZE_LARGE  (USE_USB_PACKET_SIZE)
#define _PACKET_PAYLOAD_OUT_SIZE_LARGE (USE_USB_PACKET_SIZE)

#else /* USE_USB_CDC_CLASS */

#define _PACKET_PAYLOAD_IN_SIZE_LARGE  ((EnumLowLevelIO)EnumLowLevelIO_IO_IN_PACKET_SIZE_LARGE)
#define _PACKET_PAYLOAD_OUT_SIZE_LARGE ((EnumLowLevelIO)EnumLowLevelIO_IO_OUT_PACKET_SIZE_LARGE)

#endif

#else /* USE_USB_CDC_CLASS */

#define _PACKET_PAYLOAD_IN_SIZE_LARGE  _PACKET_PAYLOAD_IN_SIZE
#define _PACKET_PAYLOAD_OUT_SIZE_LARGE _PACKET_PAYLOAD_OUT_SIZE

#endif /* ! USE_USB_CDC_CLASS */


static uint16_t packet_size = _PACKET_PAYLOAD_IN_SIZE;

uint16_t pb_io_get_packet_size(uint16_t *max_size)
{
  if (max_size)
    *max_size = _PACKET_PAYLOAD_IN_SIZE_LARGE;
  return packet_size;
}

void pb_io_set_packet_size(uint16_t val)
{
  packet_size = val;
}

void pb_io_reset_packet_size(void)
{
  packet_size = _PACKET_PAYLOAD_IN_SIZE;
}


static struct o_packet {
  uint16_t pw;
  uint8_t payload[_PACKET_PAYLOAD_IN_SIZE_LARGE];
} o_packet;

static bool write_packet(void) {
  return ioRawWriteBuffer((uint8_t *)&o_packet, packet_size + _PACKET_HEADER_SIZE);
}

void pb_io_flush_ostream(void)
{
  uint8_t tmp = (o_packet.pw >> 8) & 0xFF;
  o_packet.pw = (o_packet.pw << 8) | tmp;
  *(uint8_t *)&o_packet.pw |= EnumLowLevelIO_IO_HEADER_EOM_FLAG; /* Indicate last packet */
  write_packet();
  o_packet.pw = 0;
}

static bool write_callback(pb_ostream_t *stream, const uint8_t *buf,
    size_t count)
{
  bool res = true;
  uint8_t *pr = (uint8_t *)buf;

  UNUSED(stream);

  while (count) {
    for (; o_packet.pw < packet_size && count; o_packet.pw++) {
      o_packet.payload[o_packet.pw] = *pr;
      pr++;
      count--;
    }
    if (o_packet.pw == packet_size) {
      uint8_t tmp = (o_packet.pw >> 8) & 0xFF;
      o_packet.pw = (o_packet.pw << 8) | tmp;
      res = write_packet();
      o_packet.pw = 0;
    }
  }

  return res;
}


static struct i_packet {
  uint16_t pr;
  uint8_t payload[_PACKET_PAYLOAD_OUT_SIZE_LARGE];
} i_packet;

static int i_ridx = 0;

static bool read_packet(void) {

  bool res = ioRawReadBuffer((uint8_t *)&i_packet,
      packet_size + _PACKET_HEADER_SIZE);
  i_ridx = 0;

  uint8_t tmp = (i_packet.pr >> 8) & 0xFF;
  i_packet.pr = (i_packet.pr << 8) | tmp;

  return res;
}

void pb_io_flush_istream(void)
{
  i_packet.pr = 0xFFFF;
  i_ridx = 0;
}

static bool read_callback(pb_istream_t *stream, uint8_t *buf, size_t count)
{
  bool res = true;
  uint8_t *pw = (uint8_t *)buf;

  UNUSED(stream);

  if (count == 0)
    return true;

  if (i_packet.pr == 0xFFFF)
    res = read_packet();

  if (res == false)
    return res;


  while (count) {
    for (; i_packet.pr > 0 && count; i_packet.pr--) {
      *pw = i_packet.payload[i_ridx];
      pw++;
      count--;
      i_ridx++;
    }
    if (count && i_packet.pr  == 0) {
      uint8_t sync = 0xAA;
      ioRawWriteBuffer(&sync, 1);
      read_packet();
    }
  }

  return res;
}

pb_ostream_t pb_io_ostream(int fd)
{
#ifndef PB_NO_ERRMSG
  pb_ostream_t stream = {&write_callback, (void*)(intptr_t)fd, SIZE_MAX, 0, NULL};
#else
  pb_ostream_t stream = {&write_callback, (void*)(intptr_t)fd, SIZE_MAX, 0};
#endif
  return stream;
}

pb_istream_t pb_io_istream(int fd)
{
#ifndef PB_NO_ERRMSG
  pb_istream_t stream = {&read_callback, (void*)(intptr_t)fd, SIZE_MAX, NULL};
#else
  pb_istream_t stream = {&read_callback, (void*)(intptr_t)fd, SIZE_MAX};
#endif
  return stream;
}

int pb_io_stream_init(void)
{
  ioRawDisableLLWrite();
  return 0;
}
