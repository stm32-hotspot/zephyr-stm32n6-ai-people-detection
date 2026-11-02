/**
 ******************************************************************************
 * @file    aiPbIO.h
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

#ifndef _AI_PB_IO_H_
#define _AI_PB_IO_H_

#include <pb.h>

int pb_io_stream_init(void);

void pb_io_flush_ostream(void);
void pb_io_flush_istream(void);

pb_ostream_t pb_io_ostream(int fd);
pb_istream_t pb_io_istream(int fd);

uint16_t pb_io_get_packet_size(uint16_t *max_size);
void pb_io_set_packet_size(uint16_t val);
void pb_io_reset_packet_size(void);

#endif /* _AI_PB_IO_H_ */
