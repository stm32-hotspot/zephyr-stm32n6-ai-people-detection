/**
 ******************************************************************************
 * @file    aiPbMemRWservices.c
 * @author  STMicroelectronics
 * @brief   AI Pb services to read/write in memory
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

#include <aiPbMemRWServices.h>


/** Computes BSD checksum if a given buffer. Same as 'sum -r' on Unix
 */
static int bsdChecksum(uint8_t* buffer, int length)
{
  int i;
  int ch;                       /* Each character read. */
  int checksum = 0;             /* The checksum mod 2^16. */

  for(i=0; i<length; i++){
    ch = buffer[i];
    checksum = (checksum >> 1) + ((checksum & 1) << 15);
    checksum += ch;
    checksum &= 0xffff;       /* Keep it within bounds. */
  }
  return checksum;
}

static bool memory_valid_addr_range(uint32_t base_addr, uint32_t size, int mode)
{
  (void)base_addr;
  (void)size;
  (void)mode;
  return true;
}

static void memory_checksum(uintptr_t base_addr, uint32_t size, uint32_t *hash)
{
  if ((size == 0) && hash)
    *hash = ~0UL;

  *hash = bsdChecksum((uint8_t *)base_addr, size);
}

static bool memory_write(uintptr_t src, uintptr_t dest, uint32_t size, uint32_t *hash)
{
  memcpy((void *)dest, (void *)src, size);
  *hash = bsdChecksum((uint8_t *)dest, size);
  return true;
}

void aiPbCmdRWMemory(const reqMsg *req, respMsg *resp, void *param)
{
  uint32_t addr = req->param;
  uint32_t size = req->opt;
  uint32_t res = 0;

  static uint32_t buffer[1024 / 4];
#if defined(AI_PB_TEST) && (AI_PB_TEST == 1)
  static uint32_t test[1024 / 4];

  if (aiPbTestModeEnabled())
    addr = (uint32_t)test;
#endif

  if (!memory_valid_addr_range(addr, size, (int)param - EnumCmd_CMD_MEMORY_READ)) {
    aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
        res, EnumError_E_INVALID_PARAM);
  }

  if ((uint32_t)param == EnumCmd_CMD_MEMORY_CHECKSUM) {
    memory_checksum(addr, size, &res);
    aiPbMgrSendAck(req, resp, EnumState_S_DONE,
        res, EnumError_E_NONE);
  }
  else if ((uint32_t)param == EnumCmd_CMD_MEMORY_WRITE) {
    aiPbData data = {0, size>1024?1024:size, (uintptr_t)buffer, 0};
    aiPbMgrSendAck(req, resp, (size == 0)?EnumState_S_DONE:EnumState_S_WAITING,
          data.size, EnumError_E_NONE);
    while (size) {
      aiPbMgrReceiveData(&data);
      if (data.nb_read > data.size) {
        aiPbMgrSendAck(req, resp, EnumState_S_ERROR, data.nb_read - data.size,
            EnumError_E_MEM_OVERFLOW);
      }
#if defined(AI_PB_TEST) && (AI_PB_TEST == 1)
      memory_write((uintptr_t)buffer, (uint32_t)test, data.nb_read, &res);
#else
      memory_write((uintptr_t)buffer, addr, data.nb_read, &res);
#endif
      size -= data.nb_read;
      addr += data.nb_read;
      aiPbMgrSendAck(req, resp, (size == 0)?EnumState_S_DONE:EnumState_S_WAITING, res,
          EnumError_E_NONE);
      data.size = size>1024?1024:size;
      data.addr = (uintptr_t)buffer;
    }
  }
  else { // EnumCmd_CMD_MEMORY_READ
    aiPbData data = {0, size>1024?1024:size, (uintptr_t)addr, 0};
    aiPbMgrSendAck(req, resp, (size == 0)?EnumState_S_DONE:EnumState_S_PROCESSING,
          data.size, EnumError_E_NONE);
    while (size) {
      data.addr = addr;
      size -= data.size;
      addr += data.size;
      aiPbMgrSendData(req, resp, (size == 0)?EnumState_S_DONE:EnumState_S_PROCESSING, &data);
      data.size = size>1024?1024:size;
    };
  }
}
