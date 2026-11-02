/**
 ******************************************************************************
 * @file    aiPbMgr.c
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

/*
 *
 * History:
 *  - v3.0: ProtoBuffer MSG definition 3.0
 */

#include <aiPbMgr.h>
#include <aiPbIO.h>
#include <stm32msg.pb.h>

#include <pb_encode.h>
#include <pb_decode.h>

#define _NO_DEVICE_PORT_FUNC         /* to import only the specific TOOLS macros, no device port functions are defined */
#include <ai_device_adaptor.h>

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

/*---------------------------------------------------------------------------*/
/* Nanopb stack initialization & manager                                     */
/*---------------------------------------------------------------------------*/

typedef enum _pbMmgState {
  PB_MGR_NOT_INITIALIZED = 0,
  PB_MGR_READY = 1,
  PB_MGR_ON_GOING = 2,
} pbMgrState;

static struct pbContextMgr {
  pb_istream_t input;
  pb_ostream_t output;
  const aiPbCmdFunc *funcs;
  uint32_t  n_func;
  reqMsg  req;
  respMsg resp;
  pbMgrState state;
} pbContextMgr;

void aiPbMgrInit(const aiPbCmdFunc *funcs)
{
  const aiPbCmdFunc *cfunc;

  memset(&pbContextMgr, 0, sizeof(struct pbContextMgr));

  pb_io_stream_init();

  pbContextMgr.input = pb_io_istream(0);
  pbContextMgr.output = pb_io_ostream(0);

  pbContextMgr.n_func = 0;
  pbContextMgr.funcs = NULL;

  if (funcs) {
    cfunc = funcs;
    while (cfunc->process) {
      pbContextMgr.n_func++;
      cfunc++;
    }
    pbContextMgr.funcs = funcs;
  }

  pbContextMgr.state = PB_MGR_READY;
}

int aiPbMgrWaitAndProcess(void)
{
  uint32_t idx;
  const aiPbCmdFunc *cfunc;

  pb_io_flush_istream();
  if (pb_decode_delimited(&pbContextMgr.input, reqMsg_fields, &(pbContextMgr.req))) {
    pb_io_flush_istream();
    pbContextMgr.state = PB_MGR_ON_GOING;
    for (idx = 0; idx < pbContextMgr.n_func; idx++) {
      cfunc = &pbContextMgr.funcs[idx];
      if (cfunc->cmd == pbContextMgr.req.cmd) {
        cfunc->process(&(pbContextMgr.req), &(pbContextMgr.resp), cfunc->param);
        break;
      }
    }
    if (idx == pbContextMgr.n_func) {
      aiPbMgrSendAck(&(pbContextMgr.req), &(pbContextMgr.resp), EnumState_S_ERROR,
          EnumError_E_INVALID_PARAM, EnumError_E_INVALID_PARAM);
    }
    pbContextMgr.state = PB_MGR_READY;
  }

  pb_io_flush_istream();

  return 0;
}

/*---------------------------------------------------------------------------*/
/* Nanopb Services                                                           */
/*---------------------------------------------------------------------------*/


uint32_t aiPbTensorFormat(EnumDataFmtType data_fmt_type, bool sign_bit, int bits, int fbits)
{
/* format is aligned with the ai_platform.h definition */

#define _AI_PB_BUFFER_FMT_PACK(value_, mask_, bits_) \
	  ( ((value_) & (mask_)) << (bits_) )

#define _AI_PB_BUFFER_FMT_SET_BITS(bits_) \
		_AI_PB_BUFFER_FMT_PACK((bits_), 0x7F, 7)

#define _AI_PB_BUFFER_FMT_SET_FBITS(fbits_) \
  _AI_PB_BUFFER_FMT_PACK((fbits_)+64, 0x7F, 0)

#define _AI_PB_BUFFER_FMT_SET(type_id_, sign_bit_, float_bit_, bits_, fbits_) \
		_AI_PB_BUFFER_FMT_PACK(float_bit_, 0x1, 24) | \
		_AI_PB_BUFFER_FMT_PACK(sign_bit_, 0x1, 23) | \
		_AI_PB_BUFFER_FMT_PACK(0, 0x3, 21) | \
		_AI_PB_BUFFER_FMT_PACK(type_id_, 0xF, 17) | \
		_AI_PB_BUFFER_FMT_PACK(0, 0x7, 14) | \
    _AI_PB_BUFFER_FMT_SET_BITS(bits_) | \
    _AI_PB_BUFFER_FMT_SET_FBITS(fbits_)

  uint32_t fmt = 0;
  switch (data_fmt_type) {
    case EnumDataFmtType_DATA_FMT_TYPE_NONE:
      fmt = _AI_PB_BUFFER_FMT_SET(EnumDataFmtType_DATA_FMT_TYPE_NONE, 0, 0, 0, 0);
      break;
    case EnumDataFmtType_DATA_FMT_TYPE_FLOAT:
      fmt = _AI_PB_BUFFER_FMT_SET(EnumDataFmtType_DATA_FMT_TYPE_FLOAT, 1, 0, 32, 0);
      break;
    case EnumDataFmtType_DATA_FMT_TYPE_INTEGER:
      fmt = _AI_PB_BUFFER_FMT_SET(EnumDataFmtType_DATA_FMT_TYPE_INTEGER, sign_bit, 0, bits, fbits);
      break;
    case EnumDataFmtType_DATA_FMT_TYPE_BOOL:
      fmt = _AI_PB_BUFFER_FMT_SET(EnumDataFmtType_DATA_FMT_TYPE_BOOL, 0, 0, 8, 0);
      break;
    case EnumDataFmtType_DATA_FMT_TYPE_FXP:
      fmt = _AI_PB_BUFFER_FMT_SET(EnumDataFmtType_DATA_FMT_TYPE_FXP, sign_bit, 0, bits, fbits);
  }
  return fmt;
}

void aiPbStrCopy(const char *src, char *dst, uint32_t max)
{
  const char undef[] = "UNDEFINED";
  size_t l = strlen(src);

  if (l >= max)
    l = max - 1;

  if (!dst)
    return;

  if (src && l) {
    memcpy(dst, src, l); // copy characters from string
    dst[l] = 0; // ensure to have NULL-terminator
  }
  else
    memcpy(dst, undef, strlen(undef) + 1);
}

/*
 * Callback function to encode array of uint32
 */
bool encode_uint32(pb_ostream_t *stream, const pb_field_t *field,
                   void * const *arg)
{
  struct _encode_uint32* array = (struct _encode_uint32 *)*arg;
  int offset = (array->offset == 0)?4:array->offset;
  for (size_t i = 0; i < array->size; i++)
  {
    if (!pb_encode_tag_for_field(stream, field))
      return false;
    uint32_t c_val = 0;
    if (offset == 2)
      c_val = *(uint16_t*)((uint8_t *)array->data + i * offset);
    else
      c_val = *(uint32_t*)((uint8_t *)array->data + i * offset);
    if (!pb_encode_varint(stream, (intptr_t)(c_val)))
      return false;
  }
  return true;
}

/*
 * Callback function to encode multiple aiTensorDescMsg
 */
bool encode_tensor_desc(pb_ostream_t *stream, const pb_field_t *field,
                        void * const *arg)
{
  struct _encode_tensor_desc* tensors = (struct _encode_tensor_desc *)*arg;

  for (size_t i = 0; i < tensors->size; i++)
  {
    aiTensorDescMsg msg = aiTensorDescMsg_init_zero;
    struct _encode_uint32 array_u32;
    tensors->cb(i, tensors->data, &msg, &array_u32);

    msg.dims.funcs.encode = encode_uint32;
    msg.dims.arg = &array_u32;

    if (!pb_encode_tag_for_field(stream, field))
      return false;

    if (!pb_encode_submessage(stream, aiTensorDescMsg_fields, &msg))
      return false;
  }
  return true;
}

/*---------------------------------------------------------------------------*/

/*
 * Local callback function to decode datas field from aiDataMsg msg
 */
static bool decode_data_cb(pb_istream_t *stream, const pb_field_t *field,
    void **arg)
{
  UNUSED(field);
  aiPbData *data = (aiPbData *)*arg;

  int maxr = data->size;
  size_t itsize = 1;
  uint8_t *pw = (uint8_t *)data->addr;
  data->nb_read = 0;

  /* Read data */
  while (stream->bytes_left) {
    uint64_t number;
    if (!pb_read(stream, (pb_byte_t *)&number, itsize))
      return false;
    if ((pw) && (maxr > 0)) /* additional data are skipped */
    {
      *pw = *(uint8_t *)&number;
      pw += itsize;
      maxr--;
    }
    data->nb_read += itsize;
  }

  return true;
}

/*
 * Local callback function to encode datas field from aiDataMsg msg
 */
bool encode_data_cb(pb_ostream_t *stream, const pb_field_t *field,
    void * const *arg)
{
  aiPbData *data = (aiPbData *)*arg;

  pb_byte_t *pr = (pb_byte_t *)data->addr;

  if (!pb_encode_tag_for_field(stream, field))
    return false;

  if (!pb_encode_string(stream, pr, data->size))
    return false;

  return true;
}


/*---------------------------------------------------------------------------*/

void aiPbMgrSendResp(const reqMsg *req, respMsg *resp,
    EnumState state)
{
  resp->reqid = req->reqid;
  resp->state = state;
  pb_encode(&pbContextMgr.output, respMsg_fields, resp);
  pb_io_flush_ostream();
}

void aiPbMgrSendAck(const reqMsg *req, respMsg *resp,
    EnumState state, uint32_t param, EnumError error)
{
  resp->which_payload = respMsg_ack_tag;
  resp->payload.ack.param = param;
  resp->payload.ack.error = error;
  aiPbMgrSendResp(req, resp, state);
}

bool aiPbMgrWaitAck(void)
{
  bool res;
  ackMsg ack = ackMsg_init_default;
  res = pb_decode_delimited(&pbContextMgr.input, ackMsg_fields, &ack);
  pb_io_flush_istream();
  return res;
}

bool aiPbMgrSendLog(const reqMsg *req, respMsg *resp,
    EnumState state, uint32_t lvl, const char *str)
{
  bool res;
  ackMsg ack = ackMsg_init_default;

  size_t len = strlen(str);

  resp->which_payload = respMsg_log_tag;
  resp->payload.log.level = lvl;
  if (len >= sizeof(resp->payload.log.str))
    len = sizeof(resp->payload.log.str) - 1;

  memcpy(&resp->payload.log.str[0], str, len+1);

  aiPbMgrSendResp(req, resp, state);

  res = pb_decode_delimited(&pbContextMgr.input, ackMsg_fields, &ack);
  pb_io_flush_istream();
  return res;
}

bool aiPbMgrSendLogV2(EnumState state, uint32_t lvl, const char *str)
{
  bool res;
  ackMsg ack = ackMsg_init_default;

  if (pbContextMgr.state != PB_MGR_ON_GOING)
    return false;

  pbContextMgr.resp.which_payload = respMsg_log_tag;
  pbContextMgr.resp.payload.log.level = lvl;

  aiPbStrCopy(str, &pbContextMgr.resp.payload.log.str[0],
      (uint32_t)sizeof(pbContextMgr.resp.payload.log.str));

  aiPbMgrSendResp(&(pbContextMgr.req), &(pbContextMgr.resp), state);

  res = pb_decode_delimited(&pbContextMgr.input, ackMsg_fields, &ack);
  pb_io_flush_istream();
  return res;
}

bool aiPbMgrReceiveData(aiPbData *data)
{
  aiDataMsg msg;

  msg.datas.funcs.decode = &decode_data_cb;
  msg.datas.arg = (void *)data;

  /* Waiting and decoding aiDataMsg message */
  pb_decode_delimited(&pbContextMgr.input, aiDataMsg_fields, &msg);
  pb_io_flush_istream();

  data->type = msg.type;
  data->addr = msg.addr;

  return true;
}

bool aiPbMgrSendData(const reqMsg *req, respMsg *resp, EnumState state, aiPbData *data)
{
  resp->which_payload = respMsg_data_tag;
  resp->payload.data.type = data->type;
  resp->payload.data.addr = (uint32_t)data->addr;
  resp->payload.data.size = data->size;

  resp->payload.data.datas.funcs.encode = &encode_data_cb;
  resp->payload.data.datas.arg = (void *)data;

  aiPbMgrSendResp(req, resp, state);

  if (state == EnumState_S_PROCESSING)
    return aiPbMgrWaitAck();
  return true;
}


bool aiPbMgrSendOperator(const reqMsg *req, respMsg *resp,
    EnumState state, const char *name, const uint32_t type, const uint32_t id,
    aiOpPerf *perf)
{
  struct _encode_uint32 array;
  resp->which_payload = respMsg_op_tag;
  if (name)
    aiPbStrCopy(name, &resp->payload.op.name[0], sizeof(resp->payload.op.name));
  else
    resp->payload.op.name[0] = 0;
  resp->payload.op.type = type;
  resp->payload.op.id = id;

  if (perf) {
    resp->payload.op.stack_used = perf->stack_usage;
    resp->payload.op.heap_used = perf->heap_usage;
    resp->payload.op.duration = perf->duration;
    resp->payload.op.counter_type = perf->counter_type;
    if (perf->counters) {
      array.size = perf->counter_n;
	  array.data = (void *)perf->counters;
	  array.offset = 4;
      resp->payload.op.counters.funcs.encode = encode_uint32;
      resp->payload.op.counters.arg = &array;
    } else {
      resp->payload.op.counters.funcs.encode = NULL;
    }
  }
  else {
    resp->payload.op.duration = 0.0f;
    resp->payload.op.counter_type = 0;
    resp->payload.op.counters.funcs.encode = NULL;
  }

  aiPbMgrSendResp(req, resp, state);
  // aiPbMgrWaitAck();

  return true;
}


#undef _ARM_TOOLS_ID

#if defined(_IS_AC6_COMPILER) && _IS_AC6_COMPILER
#define _ARM_TOOLS_ID       EnumTools_AI_MDK_6
#endif

#if defined(_IS_GCC_COMPILER) && _IS_GCC_COMPILER
#define _ARM_TOOLS_ID       EnumTools_AI_GCC
#endif

#if defined(_IS_IAR_COMPILER) && _IS_IAR_COMPILER
#define _ARM_TOOLS_ID       EnumTools_AI_IAR
#endif

#if defined(_IS_AC5_COMPILER) && _IS_AC5_COMPILER
#define _ARM_TOOLS_ID       EnumTools_AI_MDK_5
#endif

#if defined(_IS_HTC_COMPILER) && _IS_HTC_COMPILER
#define _ARM_TOOLS_ID       EnumTools_AI_HTC
#endif

#if defined(_IS_GHS_COMPILER) && _IS_GHS_COMPILER
#define _ARM_TOOLS_ID       EnumTools_AI_GHS
#endif

void aiPbCmdDisconnect(const reqMsg *req, respMsg *resp, void *param)
{
  UNUSED(req);
  UNUSED(resp);
  UNUSED(param);
  pb_io_reset_packet_size();
}

void aiPbCmdSync(const reqMsg *req, respMsg *resp, void *param)
{
  resp->which_payload = respMsg_sync_tag;
  resp->payload.sync.version =
      EnumVersion_P_VERSION_MAJOR << 8 |
      EnumVersion_P_VERSION_MINOR;

  resp->payload.sync.capability = 0;


#if defined(AI_PB_TEST) && (AI_PB_TEST == 1)
  resp->payload.sync.capability |= EnumCapability_CAP_SELF_TEST;
#endif

#if defined(USE_USB_CDC_CLASS) && USE_USB_CDC_CLASS == 1
  resp->payload.sync.capability |= EnumCapability_CAP_USBC;
#endif
  /*
   * .rtid field (32b) (also reported with model info msg)
   *  b7..b0   8b : RT ID (EnumAiRuntime value)
   *  b15..b8  8b : API variant (EnumAiApiRuntime value)
   *  b23..b16 8b : TOOL-CHAIN ID (EnumTools value)
   */

  resp->payload.sync.rtid = ((uint32_t)param >> 16) & 0xFFFF;
  resp->payload.sync.capability |= ((uint32_t)param & 0xFFFF);

  resp->payload.sync.rtid |= (_ARM_TOOLS_ID << EnumTools_AI_TOOLS_POS);

  aiPbMgrSendResp(req, resp, EnumState_S_IDLE);
}


/*---------------------------------------------------------------------------*/

#if defined(AI_PB_TEST) && (AI_PB_TEST == 1)

static bool _test_mode_is_enabled = false;


bool aiPbTestModeEnabled(void) {
  return _test_mode_is_enabled;
}

#define _TEST_DATA_SIZE (1024)

MEM_ALIGNED(4)
static uint8_t _data[_TEST_DATA_SIZE];

/* https://rosettacode.org/wiki/CRC-32#Python */
static uint32_t test_rc_crc32(uint32_t crc, const char *buf, size_t len)
{
  static uint32_t table[256];
  static int have_table = 0;
  uint32_t rem;
  uint8_t octet;
  int i, j;
  const char *p, *q;

  /* This check is not thread safe; there is no mutex. */
  if (have_table == 0) {
    /* Calculate CRC table. */
    for (i = 0; i < 256; i++) {
      rem = i;  /* remainder from polynomial division */
      for (j = 0; j < 8; j++) {
        if (rem & 1) {
          rem >>= 1;
          rem ^= 0xedb88320;
        } else
          rem >>= 1;
      }
      table[i] = rem;
    }
    have_table = 1;
  }

  crc = ~crc;
  q = buf + len;
  for (p = buf; p < q; p++) {
    octet = *p;  /* Cast to unsigned octet. */
    crc = (crc >> 8) ^ table[(crc & 0xff) ^ octet];
  }
  return ~crc;
}

static uint32_t test_tensor_size(uint32_t *shape, uint32_t size)
{
  uint32_t s = 1;
  for (uint32_t i=0; i < size; i++) {
    s *= shape[i];
  }
  return s;
}

/* TEST CMD */
void aiPbTestCmd(const reqMsg *req, respMsg *resp, void *param)
{
  uint32_t test_id = req->param & 0xFFFF;
  uint32_t opt = req->opt;
  uint32_t extra = req->param >> 16;

  if (test_id == 0)
  {
    /* Send simple ACK: S_DONE/E_NONE */
    aiPbMgrSendAck(req, resp, EnumState_S_DONE, (uint32_t)aiPbTestModeEnabled(), EnumError_E_NONE);
  }
  else if (test_id == 1)
  {
    /* Send Simple ACK: invalid param */
    aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
              EnumError_E_INVALID_PARAM, EnumError_E_INVALID_PARAM);
  }
  else if (test_id == 2)
  {
      const char *str = "Hello.. ";
      for (int i=0; i < 5; i++) {
        aiPbMgrSendLog(req, resp, EnumState_S_DONE, i + 1, str);
      }
      aiPbMgrSendAck(req, resp, EnumState_S_DONE, req->param,
          EnumError_E_NONE);
  }
  else if (test_id == 3)
  {
    /* time out / no answer */
  }
  else if (test_id == 4)
  {
    /* send(S_PROCESSING/E_NONE)->send(S_DONE/E_NONE) sequence */
    aiPbMgrSendAck(req, resp, EnumState_S_PROCESSING, ~opt,
        EnumError_E_NONE);
    // HAL_Delay(10);
    aiPbMgrSendAck(req, resp, EnumState_S_DONE, req->param,
        EnumError_E_NONE);
  }
  else if (test_id == 10)
  {
    /* Send simple ACK with expected tensor format description */
    uint32_t fmt = 0;
    if (opt == 0) // _FLOAT
      fmt = aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_FLOAT, 1, 32, 0);
    else if (opt == 1) // _U1
      fmt = aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_INTEGER, false, 1, 0);
    else if (opt == 2) // _S1
      fmt = aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_INTEGER, true, 1, 0);
    else if (opt == 3) // _U8
      fmt = aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_INTEGER, false, 8, 0);
    else if (opt == 4)  // _S8
      fmt = aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_INTEGER, true, 8, 0);
    else if (opt == 20) // _BOOL
      fmt = aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_BOOL, true, 0, 0);
    else if (opt == 21) // _NONE
      fmt = aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_NONE, 0, 8, 0);
    else if (opt == 10) // Q8
      fmt = aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_INTEGER, true, 8, 7);
    else if (opt == 11) // _UQ8
      fmt = aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_INTEGER, false, 8, 7);
    else if (opt == 12) // _Q15
      fmt = aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_INTEGER, true, 16, 15);
    else if (opt == 13) // _UQ15
      fmt = aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_INTEGER, false, 16, 15);
    else if (opt == 30) // _Q7.4
      fmt = aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_INTEGER, true, 8, 4);
    else if (opt == 31) // _UQ16.5
      fmt = aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_INTEGER, false, 16, 5);
    else if (opt == 32) // _Q31.13
      fmt = aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_INTEGER, true, 32, 13);

    aiPbMgrSendAck(req, resp, EnumState_S_DONE, fmt, EnumError_E_NONE);
  }
  else if (test_id == 20)
  {
    /* Send simple resp.aiOperatorMsg */
    uint32_t counters[] = {5, 7};
    aiOpPerf perf = {
        2.5f, 1, 2, counters, 0, 0
    };
    if (opt == 0)
      aiPbMgrSendOperator(req, resp, EnumState_S_DONE, req->name, 34, 55, NULL);
    if (opt == 1) {
      perf.counters = NULL;
      perf.counter_type = 7;
      perf.counter_n = 0;
      aiPbMgrSendOperator(req, resp, EnumState_S_DONE, req->name, 34, 55, &perf);
    }
    if (opt == 2) {
      perf.counter_type = 2;
      aiPbMgrSendOperator(req, resp, EnumState_S_DONE, req->name, 34, 55, &perf);
    }
  }
  else if (test_id == 30)
  {
    /* Receive data aiDataMsg */
    if (extra <= _TEST_DATA_SIZE) {
        aiPbMgrSendAck(req, resp, EnumState_S_WAITING, extra, EnumError_E_NONE);
        aiPbData data = { 0, extra, (uintptr_t)_data, 0};
        aiPbMgrReceiveData(&data);
        uint32_t crc = test_rc_crc32(0, (char *)_data, extra);
        if (opt == 0) {
          aiPbMgrSendAck(req, resp, EnumState_S_DONE, crc, EnumError_E_NONE);
        } else {
          aiPbMgrSendAck(req, resp, EnumState_S_PROCESSING, crc, EnumError_E_NONE);
          aiPbMgrWaitAck();
        }
    }
    else
      aiPbMgrSendAck(req, resp, EnumState_S_ERROR, extra, EnumError_E_INVALID_SIZE);
  }
  else if (test_id == 40)
  {
    /* Send data aiDataMsg */
    if (extra <= _TEST_DATA_SIZE) {
      aiPbMgrSendAck(req, resp, EnumState_S_WAITING, extra, EnumError_E_NONE);
      aiPbData data = { opt, extra, (uintptr_t)_data, 0};
      memset(_data, opt, extra);
      aiPbMgrSendData(req, resp, EnumState_S_PROCESSING, &data);
      uint32_t crc = test_rc_crc32(0, (char *)_data, extra);
      aiPbMgrSendAck(req, resp, EnumState_S_DONE, crc, EnumError_E_NONE);
    }
    else
      aiPbMgrSendAck(req, resp, EnumState_S_ERROR, extra, EnumError_E_INVALID_SIZE);
  }
  else if (test_id == 50)
  {
    /* Receive/Send data aiDataMsg */
    if (extra <= _TEST_DATA_SIZE) {
      memset(_data, 0, _TEST_DATA_SIZE);
      aiPbMgrSendAck(req, resp, EnumState_S_WAITING, extra, EnumError_E_NONE);
      aiPbData data = { 0, extra, (uintptr_t)_data, 0};

      // receive buffer (! data.addr and data.type are updated)
      aiPbMgrReceiveData(&data);
      uint32_t crc = test_rc_crc32(0, (char *)_data, extra);
      aiPbMgrSendAck(req, resp, EnumState_S_PROCESSING, crc, EnumError_E_NONE);

      // send buffer
      data.size = extra;
      data.addr = (uintptr_t)_data;
      data.nb_read = 0;
      data.type = opt;
      aiPbMgrSendData(req, resp, EnumState_S_PROCESSING, &data);
      aiPbMgrSendAck(req, resp, EnumState_S_DONE, crc, EnumError_E_NONE);
    }
    else
      aiPbMgrSendAck(req, resp, EnumState_S_ERROR, extra, EnumError_E_INVALID_SIZE);
  }
  else if (test_id == 100)
  {
    struct conf {
      char *name;
      uint32_t *shape;
      uint32_t shape_size;
      uint32_t fmt;
      uint32_t elem_size;
      float scale;
      int32_t zp;
    };

    uint32_t fmt0 = aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_INTEGER, true, 8, 0);
    uint32_t fmt1 = aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_INTEGER, true, 32, 13);
    uint32_t fmt2 = aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_FLOAT, true, 32, 0);
    uint32_t fmt3 = aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_INTEGER, false, 8, 0);
    uint32_t shape0[] = { 1, 3, 3, 5 };
    uint32_t shape1[] = { 1, 5};

    struct conf confs[] = {
        { "conf0", shape0, 4, fmt0, 1, 1.0/256.0, -123 },
        { "conf1", shape1, 2, fmt1, 4, 0.0, 0 },
        { "conf2", shape0, 4, fmt1, 0, 0.0, 0 },
        { "conf3", shape1, 2, fmt2, 4, 0.0, 0 },
        { "conf4", shape0, 4, fmt3, 1, 1.0/256.0, 0 },
    };
    /* Send aiTensorMsg */
    uint32_t conf = extra<5?extra:0;
    memset(_data, 0, _TEST_DATA_SIZE);

    uint32_t nb_elem = test_tensor_size(confs[conf].shape, confs[conf].shape_size);

    uint8_t *pw = (uint8_t *)_data;
    for (uint32_t i=0; i<nb_elem; i++) {
      *pw = opt;
      pw += confs[conf].elem_size;
    }

    aiPbMgrSendAck(req, resp, EnumState_S_WAITING, conf, EnumError_E_NONE);

    /* Build the PB message */
    resp->which_payload = respMsg_tensor_tag;

    /*-- Flags field */
    resp->payload.tensor.desc.flags = EnumTensorFlag_TENSOR_FLAG_LAST;
    if (confs[conf].elem_size == 0)
      resp->payload.tensor.desc.flags |= EnumTensorFlag_TENSOR_FLAG_NO_DATA;

    /*-- Tensor desc field */
    struct _encode_uint32 array_u32;
    array_u32.size = confs[conf].shape_size;
    array_u32.data = (uint32_t *)confs[conf].shape;
    array_u32.offset = sizeof(confs[conf].shape[0]);

    aiPbStrCopy(confs[conf].name,
                (char *)&resp->payload.tensor.desc.name,
                sizeof(resp->payload.tensor.desc.name));
    resp->payload.tensor.desc.format = confs[conf].fmt;

    resp->payload.tensor.desc.n_dims = EnumShapeFmt_F_SHAPE_FMT_BHWC << 24 | array_u32.size;
    resp->payload.tensor.desc.size = nb_elem;

    resp->payload.tensor.desc.dims.funcs.encode = encode_uint32;
    resp->payload.tensor.desc.dims.arg = &array_u32;

    resp->payload.tensor.desc.scale = confs[conf].scale;
    resp->payload.tensor.desc.zeropoint = confs[conf].zp;

    /*-- Data field */
    resp->payload.tensor.data.addr = (uint32_t)_data;
    resp->payload.tensor.data.size = nb_elem * confs[conf].elem_size;

    struct aiPbData data = { 0, resp->payload.tensor.data.size, resp->payload.tensor.data.addr, 0};
    resp->payload.tensor.data.datas.funcs.encode = &encode_data_cb;
    resp->payload.tensor.data.datas.arg = (void *)&data;

    /* Send the PB message */
    aiPbMgrSendResp(req, resp, EnumState_S_DONE);

    aiPbMgrSendAck(req, resp, EnumState_S_DONE, 0, EnumError_E_NONE);
  }
  else if (test_id == 500)
  {
    _test_mode_is_enabled = _test_mode_is_enabled?false:true;
    aiPbMgrSendAck(req, resp, EnumState_S_DONE, (uint32_t)aiPbTestModeEnabled(), EnumError_E_NONE);
  }
  else
  {
    /* Send simple ACK: S_DONE/E_NONE */
    aiPbMgrSendAck(req, resp, EnumState_S_DONE, (uint32_t)aiPbTestModeEnabled(), EnumError_E_GENERIC);
  }
}

#endif  /* defined(AI_PB_TEST) */
