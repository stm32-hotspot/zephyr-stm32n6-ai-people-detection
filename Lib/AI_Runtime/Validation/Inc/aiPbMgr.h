/**
 ******************************************************************************
 * @file    aiPbMgr.h
 * @author  STMicroelectronics
 * @brief   Helper function for AI ProtoBuffer support
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

#ifndef _AI_PB_MGR_H_
#define _AI_PB_MGR_H_

#include <stdint.h>

#include "pb.h"
#include "stm32msg.pb.h"

#ifndef AI_PB_TEST
#define AI_PB_TEST 0
#endif

#ifdef __cplusplus
extern "C" {
#endif


/*---------------------------------------------------------------------------*/
/* Nanopb stack initialization & manager                                     */
/*---------------------------------------------------------------------------*/

typedef struct _aiPbCmdFunc {
        EnumCmd cmd;
        void (*process)(const reqMsg *req, respMsg *resp, void *param);
        void *param;
} aiPbCmdFunc;

void aiPbMgrInit(const aiPbCmdFunc *funcs);
int aiPbMgrWaitAndProcess(void);

void aiPbCmdSync(const reqMsg *req, respMsg *resp, void *param);
#define AI_PB_CMD_SYNC(par) { EnumCmd_CMD_SYNC, &aiPbCmdSync, (par) }

void aiPbCmdDisconnect(const reqMsg *req, respMsg *resp, void *param);
#define AI_PB_CMD_DISCONNECT(par) { EnumCmd_CMD_DISCONNECT, &aiPbCmdDisconnect, NULL }

#define AI_PB_CMD_END      { (EnumCmd)0, NULL, NULL }

#if defined(AI_PB_TEST) && (AI_PB_TEST == 1)
void aiPbTestCmd(const reqMsg *req, respMsg *resp, void *param);
#define AI_PB_CMD_TEST(par) { EnumCmd_CMD_TEST, &aiPbTestCmd, (par) }

bool aiPbTestModeEnabled(void);
#endif


/*---------------------------------------------------------------------------*/
/* Nanopb services                                                           */
/*---------------------------------------------------------------------------*/

/* built-in services */

void aiPbMgrSendResp(const reqMsg *req, respMsg *resp, EnumState state);

void aiPbMgrSendAck(const reqMsg *req, respMsg *resp,
                    EnumState state, uint32_t param, EnumError error);

bool aiPbMgrSendLog(const reqMsg *req, respMsg *resp,
        EnumState state, uint32_t lvl, const char *str);

bool aiPbMgrSendLogV2(EnumState state, uint32_t lvl, const char *str);

bool aiPbMgrWaitAck(void);

typedef struct aiPbData {
  uint32_t  type;
  uint32_t  size;
  uintptr_t addr;
  uint32_t  nb_read;
} aiPbData;

bool aiPbMgrReceiveData(aiPbData *data);

bool aiPbMgrSendData(const reqMsg *req, respMsg *resp, EnumState state,
                     aiPbData *data);

typedef struct aiOpPerf {
  float duration;        /* duration in ms */
  uint32_t counter_type; /* */
  uint32_t counter_n;    /* */
  uint32_t *counters;    /* */
  int32_t stack_usage;   /* */
  int32_t  heap_usage;   /* */
} aiOpPerf;

bool aiPbMgrSendOperator(const reqMsg *req, respMsg *resp,
                         EnumState state, const char *name,
                         const uint32_t type, const uint32_t id,
                         aiOpPerf *perf);

/* low-level services */

struct _encode_uint32 {
  size_t size;
  void *data;
  uint32_t offset;
};

struct _encode_tensor_desc {
  void (*cb)(size_t index, void* data, aiTensorDescMsg* msg,
             struct _encode_uint32 *array_u32);
  size_t size;
  void* data;
};

bool encode_tensor_desc(pb_ostream_t *stream, const pb_field_t *field,
                        void * const *arg);
bool encode_uint32(pb_ostream_t *stream, const pb_field_t *field,
                   void * const *arg);
bool encode_data_cb(pb_ostream_t *stream, const pb_field_t *field,
                    void * const *arg);

void aiPbStrCopy(const char *src, char *dst, uint32_t max);


uint32_t aiPbTensorFormat(EnumDataFmtType data_fmt_type, bool sign_bit, int bits, int fbits);


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _AI_PB_MGR_H_ */
