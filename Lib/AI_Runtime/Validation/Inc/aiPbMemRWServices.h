/**
 ******************************************************************************
 * @file    aiPbMemRWservices.h
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

#ifndef _AI_PB_MEM_RW_SERVICES_
#define _AI_PB_MEM_RW_SERVICES_

#include <aiPbMgr.h>

void aiPbCmdRWMemory(const reqMsg *req, respMsg *resp, void *param);

#define AI_PB_MEMORY_RW_SERVICES()\
    { EnumCmd_CMD_MEMORY_READ, &aiPbCmdRWMemory, (void *)EnumCmd_CMD_MEMORY_READ },\
    { EnumCmd_CMD_MEMORY_WRITE, &aiPbCmdRWMemory, (void* )EnumCmd_CMD_MEMORY_WRITE },\
    { EnumCmd_CMD_MEMORY_CHECKSUM, &aiPbCmdRWMemory, (void *)EnumCmd_CMD_MEMORY_CHECKSUM }

#endif /* _AI_PB_MEM_RW_SERVICES_ */
