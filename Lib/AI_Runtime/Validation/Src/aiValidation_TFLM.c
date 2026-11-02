/**
 ******************************************************************************
 * @file    aiValidation_TFLM.c
 * @author  STMicroelectronics
 * @brief   AI Validation application (entry points) - TFLM runtime
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

/* Description
 *
 * - Entry points for the AI validation, TFLM runtime
 *
 * History:
 *  - v1.0 - Initial version. Use the X-CUBE-AI protocol vX.X
 *           Emulate X-CUBE-AI interface
 *  - v1.1 - minor - let irq enabled if USB CDC is used
 *  - v2.0 - align code for TFLM 2.5.0
 *           add observer support to upload time by layer with or w/o data
 *  - v2.1 - Use the fix cycle count overflow support
 *  - v2.2 - map TFLM log error message on aiPbMgrSendLog service
 *  - v2.3 - Add support to use SYSTICK only (remove direct call to DWT fcts)
 *  - v3.0 - align code with TFLM from tflite-micro repo and new ai_buffer definition
 *           86c8d52 - Fix erroneous write from EXPAND_DIMS to an array that can be in read-only region (#649)
 *           USE_OBSERVER can be not used with the current TFLM runtime to dump the intermediate tensors
 *           Only IO_ONLY mode can be used.
 *  - v3.1 - Add support for IO tensor with shape > 4 (up to 6)
 *  - v4.0 - migration to PB msg interface 3.0
 *           align code with ai_stm32_adpaptor.h file (remove direct call of HAL_xx fcts)
 *  - v4.1 - migration to PB msg interface 3.1
 *  - v5.0 - alignment with new tfm_c interface 5.0
 */

/* System headers */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdarg.h>

#if !defined(UNUSED)
#define UNUSED(x) ((void)(x))
#endif

#define USE_OBSERVER         1

#define USE_CORE_CLOCK_ONLY  0 /* 1: remove usage of the HAL_GetTick() to evaluate the number of CPU clock. Only the Core
                                *    DWT IP is used. HAL_Tick() is requested to avoid an overflow with the DWT clock counter
                                *    (32b register) - USE_SYSTICK_ONLY should be set to 0.
                                */
#define USE_SYSTICK_ONLY     0 /* 1: use only the SysTick to evaluate the time-stamps (for Cortex-m0 based device, this define is forced) */

/* APP Header files */
#include <aiValidation.h>
#include <aiTestUtility.h>
#include <aiTestHelper.h>
#include <aiPbMgr.h>

/* TFLM C header files */
#include <tflm_c.h>

#include "network_tflite_data.h"


#define _AI_RUNTIME_ID EnumAiRuntime_AI_RT_TFLM


#define TFLM_NETWORK_NUMBER_MAX (1)


#if defined(USE_OBSERVER) && USE_OBSERVER == 1
// #define _CAP (void *)(EnumCapability_CAP_OBSERVER | (_AI_RUNTIME_ID << 16))
#define _CAP (void *)(_AI_RUNTIME_ID << 16)
#else
#define _CAP (void *)(_AI_RUNTIME_ID << 16)
#endif


/* -----------------------------------------------------------------------------
 * TEST-related definitions
 * -----------------------------------------------------------------------------
 */

#define _APP_VERSION_MAJOR_  (0x04)
#define _APP_VERSION_MINOR_  (0x01)
#define _APP_VERSION_        ((_APP_VERSION_MAJOR_ << 8) | _APP_VERSION_MINOR_)

#define _APP_NAME_           "AI Validation TFLM"

/*
 * Execution context - Only one instance is supported
 */

struct tflm_context {
  char *name;
  uint32_t hdl;
  uint32_t n_inputs;
  uint32_t n_outputs;
  struct tflm_c_tensor_info *inputs;
  struct tflm_c_tensor_info *outputs;
  struct tflm_c_tensor_info  arena;
  struct tflm_c_tensor_info  tflite;
  int error;
  bool debug;
} net_exec_ctx[TFLM_NETWORK_NUMBER_MAX] = {0};


#if defined(TFLM_NETWORK_NAME)
static char* _model_name = TFLM_NETWORK_NAME;
#else
static char* _model_name = "network";
#endif


/* -----------------------------------------------------------------------------
 * Helper functions
 * -----------------------------------------------------------------------------
 */

static void _prepare_tensor_infos(struct tflm_context *ctx)
{
  if (!ctx || !ctx->hdl)
    return;

  ctx->n_inputs = tflm_c_inputs_size(ctx->hdl);
  ctx->n_outputs = tflm_c_outputs_size(ctx->hdl);

  ctx->inputs = malloc(ctx->n_inputs * sizeof(struct tflm_c_tensor_info));
  if (!ctx->inputs)
    return;
  ctx->outputs = malloc(ctx->n_outputs * sizeof(struct tflm_c_tensor_info));
  if (!ctx->outputs) {
    free(ctx->inputs);
    return;
  }

  for (uint32_t idx=0; idx<ctx->n_inputs; idx++) {
    tflm_c_input(ctx->hdl, idx, &ctx->inputs[idx]);
  }

  for (uint32_t idx=0; idx<ctx->n_outputs; idx++) {
    tflm_c_output(ctx->hdl, idx, &ctx->outputs[idx]);
  }

  memset(&ctx->arena, 0, sizeof(struct tflm_c_tensor_info));
  memset(&ctx->tflite, 0, sizeof(struct tflm_c_tensor_info));
  ctx->arena.bytes = (size_t)tflm_c_arena_used_bytes(ctx->hdl);
  ctx->arena.type = kTfLiteUInt8;
  ctx->arena.shape.size = 1;
  ctx->arena.shape.data[0] = ctx->arena.bytes;
  ctx->tflite.bytes = (size_t)g_tflm_network_model_data_len;
  ctx->tflite.type = kTfLiteUInt8;
  ctx->tflite.shape.size = 1;
  ctx->tflite.shape.data[0] = ctx->tflite.bytes;
}

static uint32_t _version_to_uint32(const struct tflm_c_version *version)
{
  return version->major << 24 | version->minor << 16 | version->patch << 8 | version->schema;
}

static uint32_t _get_buffer_size(const struct tflm_c_tensor_info *info)
{
  return info->bytes;
}

static uint32_t _get_element_size(const struct tflm_c_tensor_info *info)
{
  switch (info->type) {
    case kTfLiteUInt32:
    case kTfLiteInt32:
    case kTfLiteFloat32: return 4;
    case kTfLiteFloat16:
    case kTfLiteInt16: return 2;
    case kTfLiteUInt8:
    case kTfLiteInt8:
    case kTfLiteBool: return 1;
    default: return 0;
  }
}

static uint32_t set_ai_buffer_format(TfLiteType tflm_type)
{
  switch (tflm_type) {
    case kTfLiteFloat32: return aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_FLOAT, true, 32, 0);
    case kTfLiteFloat16: return aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_FLOAT, true, 16, 0);
    case kTfLiteUInt8: return aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_INTEGER, false, 8, 0);
    case kTfLiteInt8: return aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_INTEGER, true, 8, 0);
    case kTfLiteInt16: return aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_INTEGER, true, 16, 0);
    case kTfLiteUInt32: return aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_INTEGER, false, 32, 0);
    case kTfLiteInt32: return aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_INTEGER, true, 32, 0);
    case kTfLiteBool: return aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_BOOL, false, 1, 0);
    default: return 0;
  }
}

struct _data_tensor_desc {
  struct tflm_context *ctx;
  struct tflm_c_tensor_info *info;
  uint32_t flags;
};

static void _fill_tensor_desc(aiTensorDescMsg* msg, struct _encode_uint32 *array_u32,
    const struct tflm_c_tensor_info *info, uint32_t flags)
{
  array_u32->size = info->shape.size;
  array_u32->data = (void *)info->shape.data;
  array_u32->offset = 4;

  msg->name[0] = 0;
  msg->format = set_ai_buffer_format(info->type);
  msg->flags = flags;

  msg->size = 1;
  for (uint32_t i=0; i<info->shape.size; i++)
    msg->size *= info->shape.data[i];

  msg->n_dims = EnumShapeFmt_F_SHAPE_FMT_UND << 24 | array_u32->size;

  if (info->scale) {
    msg->scale = info->scale;
    msg->zeropoint = info->zero_point;
  } else {
    msg->scale = 0.0;
    msg->zeropoint = 0;
  }

  msg->addr = (uint32_t)info->data;
}

static void _encode_to_tensor_desc_cb(size_t index,
    void* data, aiTensorDescMsg* msg,
    struct _encode_uint32 *array_u32)
{
  struct _data_tensor_desc *desc = (struct _data_tensor_desc *)data;
  struct tflm_c_tensor_info *info = &desc->info[index];

  _fill_tensor_desc(msg, array_u32, info, desc->flags);
}

static void send_model_info(const reqMsg *req, respMsg *resp,
    EnumState state, struct tflm_context *ctx)
{
  uint32_t flags;
  struct tflm_c_version ver;

  const char *_compile_date_time = __DATE__ " " __TIME__;

  tflm_c_rt_version(&ver);

  resp->which_payload = respMsg_minfo_tag;

  memset(&resp->payload.minfo, 0, sizeof(aiModelInfoMsg));

  aiPbStrCopy(_model_name, &resp->payload.minfo.name[0],
      sizeof(resp->payload.minfo.name));

  /* rtid definition - see rtid field (32b) in stm32msg.proto file.
   32b: <31 extra(8b) 24> <23 tools(8b) 16> <15 api(8b) 8> <7 rt(8b) 0>
   */

  resp->payload.minfo.rtid = _AI_RUNTIME_ID | ver.tools_id << EnumTools_AI_TOOLS_POS;

  resp->payload.minfo.rtid |= (4 << 24);  /* no macc info */


  aiPbStrCopy(ver.desc, &resp->payload.minfo.runtime_desc[0],
      sizeof(resp->payload.minfo.runtime_desc));

  uint32_to_str(ver.build, &resp->payload.minfo.runtime_desc[strlen(ver.desc)],
                sizeof(resp->payload.minfo.runtime_desc) - strlen(ver.desc) - 1);

  aiPbStrCopy(_compile_date_time, &resp->payload.minfo.compile_datetime[0],
      sizeof(resp->payload.minfo.compile_datetime));

  resp->payload.minfo.runtime_version = _version_to_uint32(&ver);
  resp->payload.minfo.tool_version = _version_to_uint32(&ver);

  resp->payload.minfo.n_macc = (uint64_t)0;
  resp->payload.minfo.n_nodes = (uint32_t)tflm_c_operators_size(ctx->hdl);

  flags = EnumTensorFlag_TENSOR_FLAG_INPUT | EnumTensorFlag_TENSOR_FLAG_IN_MEMPOOL;
  struct _data_tensor_desc tensor_desc_ins = {ctx, ctx->inputs, flags};
  struct _encode_tensor_desc tensor_ins = {
      &_encode_to_tensor_desc_cb, ctx->n_inputs, &tensor_desc_ins};
  resp->payload.minfo.n_inputs = ctx->n_inputs;
  resp->payload.minfo.inputs.funcs.encode = encode_tensor_desc;
  resp->payload.minfo.inputs.arg = (void *)&tensor_ins;

  flags = EnumTensorFlag_TENSOR_FLAG_OUTPUT | EnumTensorFlag_TENSOR_FLAG_IN_MEMPOOL;
  struct _data_tensor_desc tensor_desc_outs = {ctx, ctx->outputs, flags};
  struct _encode_tensor_desc tensor_outs = {
      &_encode_to_tensor_desc_cb, ctx->n_outputs, &tensor_desc_outs};
  resp->payload.minfo.n_outputs = ctx->n_outputs;
  resp->payload.minfo.outputs.funcs.encode = encode_tensor_desc;
  resp->payload.minfo.outputs.arg = (void *)&tensor_outs;

  flags = EnumTensorFlag_TENSOR_FLAG_MEMPOOL;
  struct _data_tensor_desc tensor_desc_acts = {ctx, &ctx->arena, flags};
  struct _encode_tensor_desc tensor_acts = {
      &_encode_to_tensor_desc_cb, 1, &tensor_desc_acts };
  resp->payload.minfo.n_activations = 1;
  resp->payload.minfo.activations.funcs.encode = encode_tensor_desc;
  resp->payload.minfo.activations.arg = (void *)&tensor_acts;

  struct _data_tensor_desc tensor_desc_w = {ctx, &ctx->tflite, flags};
  struct _encode_tensor_desc tensor_w = {
      &_encode_to_tensor_desc_cb, 1, &tensor_desc_w };
  resp->payload.minfo.n_params = 1;
  resp->payload.minfo.params.funcs.encode = encode_tensor_desc;
  resp->payload.minfo.params.arg = (void *)&tensor_w;

  aiPbMgrSendResp(req, resp, state);
}

static bool receive_ai_data(const reqMsg *req, respMsg *resp,
    EnumState state, struct tflm_c_tensor_info *buffer, const bool const_value, const bool direct_write)
{
  bool res = true;
  uint32_t temp;
  aiPbData data = { 0, _get_buffer_size(buffer), (uintptr_t)buffer->data, 0};

  if ((const_value) || (direct_write))
    data.size = _get_element_size(buffer);
  if (direct_write)
    data.addr = (uintptr_t)&temp;

  aiPbMgrReceiveData(&data);

  /* Send ACK and wait ACK (or send ACK only if error) */
  if (data.nb_read != data.size) {
    aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
        data.nb_read,
        EnumError_E_INVALID_SIZE);
    res = false;
  }
  else {

  if ((const_value) && (!direct_write))/* broadcast the value */
    {
      const size_t el_s = data.size;
      const uintptr_t r_ptr = (uintptr_t)buffer->data;
      uintptr_t w_ptr = r_ptr + el_s;
      for (size_t pos = 1; pos < _get_buffer_size(buffer) / el_s; pos++)
      {
        memcpy((void *)w_ptr, (void *)r_ptr, el_s);
        w_ptr += el_s;
      }
    }

    aiPbMgrSendAck(req, resp, state, data.size, EnumError_E_NONE);
    if ((state == EnumState_S_WAITING) ||
        (state == EnumState_S_PROCESSING))
      aiPbMgrWaitAck();
  }

  return res;
}

static bool send_ai_io_tensor(const reqMsg *req, respMsg *resp,
    EnumState state, const struct tflm_c_tensor_info *buffer,
    const uint32_t flags)
{
  struct _encode_uint32 array_u32;

  /* Build the PB message */
  resp->which_payload = respMsg_tensor_tag;

  /*-- Tensor desc field */
  _fill_tensor_desc(&resp->payload.tensor.desc, &array_u32, buffer, flags);

  resp->payload.tensor.desc.dims.funcs.encode = encode_uint32;
  resp->payload.tensor.desc.dims.arg = &array_u32;

  /*-- Data field */
  resp->payload.tensor.data.addr = (uint32_t)buffer->data;
  if (flags & EnumTensorFlag_TENSOR_FLAG_NO_DATA) {
    resp->payload.tensor.data.size = 0;
  } else {
    resp->payload.tensor.data.size = _get_buffer_size(buffer);
  }
  struct aiPbData data = { 0, resp->payload.tensor.data.size, resp->payload.tensor.data.addr, 0};
  resp->payload.tensor.data.datas.funcs.encode = &encode_data_cb;
  resp->payload.tensor.data.datas.arg = (void *)&data;

  /* Send the PB message */
  aiPbMgrSendResp(req, resp, state);

  return true;

#if 0
  /* Waiting ACK */
  if (state == EnumState_S_PROCESSING)
    return aiPbMgrWaitAck();
  else
    return true;
#endif
}


/* -----------------------------------------------------------------------------
 * PB_LC_PRINT/STAT functions
 * -----------------------------------------------------------------------------
 */

#if defined(HAS_DEDICATED_PRINT_PORT) && HAS_DEDICATED_PRINT_PORT == 1
#define PB_LC_PRINT(debug, fmt, ...) LC_PRINT(fmt, ##__VA_ARGS__)
#define PB_LC_STAT(_cat, _sub_cat, fmt, ...) LC_PRINT("s:" _cat ":" _sub_cat ":" fmt, ##__VA_ARGS__)
#else

#define _PRINT_BUFFER_SIZE  256

static char _print_buffer[_PRINT_BUFFER_SIZE];

void _print_debug(bool debug, const char* fmt, ...);

void _print_debug(bool debug, const char* fmt, ...)
{
  va_list ap;
  size_t s;

  if (!debug)
    return;

  va_start(ap, fmt);
  s = LC_VSNPRINT(_print_buffer, _PRINT_BUFFER_SIZE - 1, fmt, ap);
  va_end(ap);
  while (s) {
    if ((_print_buffer[s] == '\n') || (_print_buffer[s] == '\r'))
      _print_buffer[s] = 0;
    s--;
  }
  _print_buffer[_PRINT_BUFFER_SIZE - 1] = 0;
  aiPbMgrSendLogV2(EnumState_S_WAITING, 1, &_print_buffer[0]);
}

#define PB_LC_PRINT(debug, fmt, ...)         _print_debug(debug, fmt, ##__VA_ARGS__)
#define PB_LC_STAT(_cat, _sub_cat, fmt, ...) _print_debug(true, "s:" _cat ":" _sub_cat ":" fmt "\r\n", ##__VA_ARGS__)
#endif


/* -----------------------------------------------------------------------------
 * object definition/declaration for AI-related execution context
 * -----------------------------------------------------------------------------
 */

/* Local activations buffer */
MEM_ALIGNED(16)
static uint8_t tensor_arena[TFLM_NETWORK_TENSOR_AREA_SIZE+32];

extern UART_HandleTypeDef UartHandle;

static bool _direct_write_uart = true;

#ifdef __cplusplus
extern "C"
{
#endif

static void _tfm_c_debug_log_puts(const char* buff)
{
  size_t count = strlen(buff);

  if ((count == 0) || (count > 256))
    return;

  if (_direct_write_uart) {
    HAL_UART_Transmit(&UartHandle, (uint8_t *)buff, count, HAL_MAX_DELAY);
  } else {
    char *pr = (char *)buff;
    if (buff && pr[0] != '\r') {
      pr[count] = 0;
      aiPbMgrSendLogV2(EnumState_S_ERROR, 0, pr);
    }
  }
}

#ifdef __cplusplus
}
#endif


void log_tensor(struct tflm_c_tensor_info* t_info, int idx)
{
  LC_PRINT(" - %d:%s:%d:(", idx,
      tflm_c_TfLiteTypeGetName(t_info->type), (int)t_info->bytes);
  for (size_t i=0; i<t_info->shape.size; i++) {
    LC_PRINT("%d", t_info->shape.data[i]);
    if (i==(t_info->shape.size-1))
      LC_PRINT(")");
    else
      LC_PRINT("x");
  }
  if (t_info->scale)
    LC_PRINT(":s=%f:zp=%d\r\n", (double)t_info->scale, t_info->zero_point);
  else
    LC_PRINT("\r\n");
}

/* -----------------------------------------------------------------------------
 * PER LAYER and call-back support
 * -----------------------------------------------------------------------------
 */

#if defined(USE_OBSERVER) && USE_OBSERVER == 1

static uint64_t _tfm_c_current_time_ticks_cb(int mode)
{
  UNUSED(mode);
  return cyclesCounterEnd();
}

static int _tflm_c_observer_node_cb(
  const void* cookie,
  const uint32_t flags,
  const struct tflm_c_node* node)
{
  struct tflm_context *ctx = (struct tflm_context *)cookie;
  if (ctx && node) {
    PB_LC_STAT("node", "end", "%d:%s:%ld:%f:%d:%d:%d",
        node->node_info.idx,
        node->node_info.name,
        node->node_info.dur,
        (double)dwtCyclesToFloatMs(node->node_info.dur),
        node->node_info.builtin_code,
        node->node_info.n_inputs,
        node->node_info.n_outputs);
  }
  return 0;
}

#endif

/* -----------------------------------------------------------------------------
 * AI-related functions and execution context
 * -----------------------------------------------------------------------------
 */

static struct tflm_context *aiExecCtx(const char *nn_name, int pos)
{
  struct tflm_context *cur = NULL;

  if (!nn_name)
    return NULL;

  if (!nn_name[0]) {
    if ((pos >= 0) && (pos < TFLM_NETWORK_NUMBER_MAX) && net_exec_ctx[pos].hdl)
      cur = &net_exec_ctx[pos];
  } else {
    int idx;
    for (idx=0; idx < TFLM_NETWORK_NUMBER_MAX; idx++) {
      cur = &net_exec_ctx[idx];
      if (cur->hdl &&
          (strlen(cur->name) == strlen(nn_name)) &&
          (strncmp(cur->name, nn_name,
              strlen(cur->name)) == 0)) {
        break;
      }
      cur = NULL;
    }
  }
  return cur;
}


static int aiBootstrap(struct tflm_context *ctx)
{
  TfLiteStatus res;
  struct tflm_c_version ver;

  /* Creating an instance of the network ------------------------- */
  LC_PRINT("\r\nInstancing the network (TFLM)..\r\n");

  /* TFLm runtime expects that the tensor arena is aligned on 16-bytes */
  uint32_t uaddr = (uint32_t)tensor_arena;
  uaddr = (uaddr + (16 - 1)) & (uint32_t)(-16);  // Round up to 16-byte boundary

  MON_ALLOC_RESET();
  MON_ALLOC_ENABLE();

  ctx->error = 0;

  tfm_c_register_log_cb(_tfm_c_debug_log_puts);

  res = tflm_c_create(g_tflm_network_model_data, (uint8_t*)uaddr,
          TFLM_NETWORK_TENSOR_AREA_SIZE, &ctx->hdl);

  MON_ALLOC_DISABLE();

  if (res != kTfLiteOk) {
    ctx->hdl = 0;
    ctx->error = -1;
    return -1;
  }

  ctx->name = _model_name;
  _prepare_tensor_infos(ctx);

  tflm_c_rt_version(&ver);

  LC_PRINT(" TFLM version       : %d.%d.%d\r\n", (int)ver.major, (int)ver.minor, (int)ver.patch);
  LC_PRINT(" Network name       : %s\r\n", _model_name);
  LC_PRINT(" TFLite file        : 0x%08x (%d bytes)\r\n", (int)g_tflm_network_model_data, (int)g_tflm_network_model_data_len);
  LC_PRINT(" Arena location     : 0x%08x\r\n", (int)uaddr);
  LC_PRINT(" Opcode size        : %d\r\n", (int)tflm_c_operator_codes_size(ctx->hdl));
  LC_PRINT(" Operator size      : %d\r\n", (int)tflm_c_operators_size(ctx->hdl));
  LC_PRINT(" Tensor size        : %d\r\n", (int)tflm_c_tensors_size(ctx->hdl));
  LC_PRINT(" Allocated size     : %d / %d\r\n", (int)tflm_c_arena_used_bytes(ctx->hdl),
      TFLM_NETWORK_TENSOR_AREA_SIZE);
  LC_PRINT(" Inputs size        : %d\r\n", (int)tflm_c_inputs_size(ctx->hdl));
  for (uint32_t i=0; i<ctx->n_inputs; i++) {
    log_tensor(&ctx->inputs[i], i);
  }
  LC_PRINT(" Outputs size       : %d\r\n", (int)tflm_c_outputs_size(ctx->hdl));
  for (uint32_t i=0; i<ctx->n_outputs; i++) {
    log_tensor(&ctx->outputs[i], i);
  }

#if defined(_APP_HEAP_MONITOR_) && _APP_HEAP_MONITOR_ == 1
  LC_PRINT(" Used heap          : %d bytes (max=%d bytes) (for c-wrapper %s)\r\n",
      MON_ALLOC_USED(), MON_ALLOC_MAX_USED(), TFLM_C_VERSION_STR);
#endif

  return 0;
}

static void aiDone(struct tflm_context *ctx)
{
  /* Releasing the instance(s) ------------------------------------- */
  LC_PRINT("Releasing the instance...\r\n");

  if (ctx->hdl != 0) {
    free(ctx->inputs);
    free(ctx->outputs);
    tflm_c_destroy(ctx->hdl);
    ctx->hdl = 0;
  }
}

static int aiInit(void)
{
  int res;

  net_exec_ctx[0].hdl = 0;
  res = aiBootstrap(&net_exec_ctx[0]);

  return res;
}

static void aiDeInit(void)
{
  aiDone(&net_exec_ctx[0]);
}


/* -----------------------------------------------------------------------------
 * Specific test APP commands
 * -----------------------------------------------------------------------------
 */

void aiPbCmdSysInfo(const reqMsg *req, respMsg *resp, void *param)
{
  UNUSED(param);
  struct mcu_conf conf;
  struct _encode_uint32 array_u32;

  getSysConf(&conf);

  resp->which_payload = respMsg_sinfo_tag;

  resp->payload.sinfo.devid = conf.devid;
  resp->payload.sinfo.sclock = conf.sclk;
  resp->payload.sinfo.hclock = conf.hclk;
  resp->payload.sinfo.cache = conf.conf;
  resp->payload.sinfo.com_param = 0;

#if defined(HAS_EXTRA_CONF) && HAS_EXTRA_CONF > 0
  array_u32.size = HAS_EXTRA_CONF;
  array_u32.offset = 4;
  array_u32.data = &conf.extra[0];
#else
  array_u32.size = 0;
  array_u32.offset = 4;
  array_u32.data = NULL;
#endif

  resp->payload.sinfo.extra.funcs.encode = encode_uint32;
  resp->payload.sinfo.extra.arg = &array_u32;

  aiPbMgrSendResp(req, resp, EnumState_S_IDLE);
}


void aiPbCmdNNInfo(const reqMsg *req, respMsg *resp, void *param)
{
  struct tflm_context *ctx;
  UNUSED(param);

  ctx = aiExecCtx(req->name, req->param);

  if (ctx) {
    struct mcu_conf conf;
    getSysConf(&conf);
    PB_LC_STAT("config", "version", "1.0");
    PB_LC_STAT("config", "dev_id", "0x%X", conf.devid);
    PB_LC_STAT("config", "mcu_freq", "%d", conf.sclk);
    PB_LC_STAT("config", "sys_freq", "%d", conf.hclk);

    send_model_info(req, resp, EnumState_S_IDLE, ctx);
  }
  else {
    if (req->param >= TFLM_NETWORK_NUMBER_MAX)
      aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
          EnumError_E_INVALID_PARAM, EnumError_E_INVALID_PARAM);
    else
      aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
          ctx->error, EnumError_E_GENERIC);
  }
}


void aiPbCmdNNRun(const reqMsg *req, respMsg *resp, void *param)
{
  TfLiteStatus tflm_res;
  uint64_t tend;
  bool res;
  bool cb_is_registered = false;
  UNUSED(param);
  uint32_t tick;

#if defined(USE_OBSERVER) && USE_OBSERVER == 1
  struct tflm_c_profile_info profile_info;
  struct tflm_c_observer_options cb_options = {
      .notify = &_tflm_c_observer_node_cb,
      .get_time = &_tfm_c_current_time_ticks_cb,
      .cookie = NULL,
      .flags = 0
  };
#endif

  struct tflm_context *ctx = aiExecCtx(req->name, req->param);

  /* 0 - Check if requested c-name model is available -------------- */
  if (ctx == 0) {
    aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
        EnumError_E_INVALID_PARAM, EnumError_E_INVALID_PARAM);
    return;
  }

  ctx->debug = req->param & EnumRunParam_P_RUN_CONF_DEBUG?true:false;
  const bool per_layer = req->param & EnumRunParam_P_RUN_MODE_PER_LAYER?true:false;
  const bool with_data = req->param & EnumRunParam_P_RUN_MODE_PER_LAYER_WITH_DATA?true:false;
  const bool const_value = req->param & EnumRunParam_P_RUN_CONF_CONST_VALUE?true:false;
  const bool direct_write = req->param & EnumRunParam_P_RUN_CONF_DIRECT_WRITE?true:false;

  PB_LC_PRINT(ctx->debug, "RUN: opt=0x%x, param=0x%x, cb=%d/%d (s=%d, dw=%d)\r\n",
      req->opt, req->param,
      per_layer, with_data,
      const_value, direct_write);

  PB_LC_PRINT(ctx->debug, "RUN: Waiting data..\r\n");

  /* 1 - Send a ACK (ready to receive a tensor) -------------------- */
  aiPbMgrSendAck(req, resp, EnumState_S_WAITING, ctx->inputs[0].bytes,
      EnumError_E_NONE);

  /* 2 - Receive all input tensors --------------------------------- */
  tick = port_hal_get_tick();
  for (uint32_t i = 0; i < ctx->n_inputs; i++) {
    /* upload a buffer */
    EnumState state = EnumState_S_WAITING;
    if ((i + 1) == ctx->n_inputs)
      state = EnumState_S_PROCESSING;
    res = receive_ai_data(req, resp, state, &ctx->inputs[i], const_value, direct_write);
    if (res != true)
      return;
  }

  /* 3 - Processing ------------------------------------------------ */
  tick = port_hal_get_tick() - tick;
  PB_LC_PRINT(ctx->debug, "RUN: Processing.. (%ld ticks to download %d input(s))\r\n", tick, ctx->n_inputs);

#if defined(USE_OBSERVER) && USE_OBSERVER == 1
  if (ctx->debug) {
     cb_options.cookie = ctx;
     if (tflm_c_observer_register(ctx->hdl, &cb_options) == kTfLiteOk) {
       cb_is_registered = true;
       tflm_c_observer_start(ctx->hdl);
     }
  }
#endif

  cyclesCounterStart();
  tflm_res = tflm_c_invoke(ctx->hdl);
  tend = cyclesCounterEnd();

#if defined(USE_OBSERVER) && USE_OBSERVER == 1
  if (ctx->debug && cb_is_registered) {
    tflm_c_observer_info(ctx->hdl, &profile_info);
    tflm_c_observer_unregister(ctx->hdl, &cb_options);
    PB_LC_PRINT(ctx->debug, "RUNN:CB: n_evts=%d, %d, %fms\r\n", profile_info.n_events, profile_info.n_invoks,
        (double)dwtCyclesToFloatMs(profile_info.node_dur));
    tend = profile_info.node_dur;
  }
#endif

  if (tflm_res != kTfLiteOk) {
    aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
        EnumError_E_GENERIC, EnumError_E_GENERIC);
    return;
  }

  /* 4 - Send basic report (optional) ------------------------------ */
  aiOpPerf perf = {dwtCyclesToFloatMs(tend), 0,  2, (uint32_t *)&tend, -1, 0};
  perf.counter_type = EnumCounterFormat_COUNTER_FMT_64B << EnumCounterFormat_COUNTER_FMT_POS;
  perf.counter_type |= EnumCounterType_COUNTER_TYPE_CPU;
  aiPbMgrSendOperator(req, resp, EnumState_S_PROCESSING, ctx->name, 0, 0, &perf);

  PB_LC_PRINT(ctx->debug, "RUN: send output tensors\r\n");
  /* 5 - Send all output tensors ----------------------------------- */
  for (uint32_t i = 0; i < ctx->n_outputs; i++) {
    EnumState state = EnumState_S_PROCESSING;
    uint32_t flags =  EnumTensorFlag_TENSOR_FLAG_OUTPUT;
    if (req->param & EnumRunParam_P_RUN_MODE_PERF) {
      flags |= EnumTensorFlag_TENSOR_FLAG_NO_DATA;
    }
    if ((i + 1) == ctx->n_outputs) {
      state = EnumState_S_DONE;
      flags |= EnumTensorFlag_TENSOR_FLAG_LAST;
    }
    send_ai_io_tensor(req, resp, state, &ctx->outputs[i], flags);
  }
}

static aiPbCmdFunc pbCmdFuncTab[] = {
    AI_PB_CMD_SYNC(_CAP),
    AI_PB_CMD_DISCONNECT(),
    { EnumCmd_CMD_SYS_INFO, &aiPbCmdSysInfo, NULL },
    { EnumCmd_CMD_NETWORK_INFO, &aiPbCmdNNInfo, NULL },
    { EnumCmd_CMD_NETWORK_RUN, &aiPbCmdNNRun, NULL },
#if defined(AI_PB_TEST) && AI_PB_TEST == 1
    AI_PB_CMD_TEST(NULL),
#endif
    AI_PB_CMD_END,
};


/* -----------------------------------------------------------------------------
 * Exported/Public functions
 * -----------------------------------------------------------------------------
 */

int aiValidationInit(void)
{
  LC_PRINT("\r\n#\r\n");
  LC_PRINT("# %s %d.%d\r\n", _APP_NAME_ , _APP_VERSION_MAJOR_, _APP_VERSION_MINOR_);
  LC_PRINT("#\r\n");

  systemSettingLog();

  cyclesCounterInit();

  return 0;
}

int aiValidationProcess(void)
{
  int r;

  r = aiInit();
  if (r) {
    LC_PRINT("\r\nE:  aiInit() r=%d\r\n", r);
    port_hal_delay(2000);
    return r;
  } else {
    LC_PRINT("\r\n");
    LC_PRINT("-------------------------------------------\r\n");
    LC_PRINT("| READY to receive a CMD from the HOST... |\r\n");
    LC_PRINT("-------------------------------------------\r\n");
    LC_PRINT("\r\n");
    LC_PRINT("# Note: At this point, default ASCII-base terminal should be closed\r\n");
    LC_PRINT("# and a serial COM interface should be used\r\n");
    LC_PRINT("# (i.e. Python ai_runner module). Protocol version = %d.%d\r\n",
        EnumVersion_P_VERSION_MAJOR,
        EnumVersion_P_VERSION_MINOR);
  }

  _direct_write_uart = false;
  aiPbMgrInit(pbCmdFuncTab);

  do {
    r = aiPbMgrWaitAndProcess();
  } while (r==0);

  return r;
}

void aiValidationDeInit(void)
{
  LC_PRINT("\r\n");
  aiDeInit();
  LC_PRINT("bye bye ...\r\n");
}

#include <stdint.h>

__attribute__((noreturn))
void __aeabi_assert(const char *expr, const char *file, int line)
{
    (void)expr;
    (void)file;
    (void)line;

#ifdef DEBUG
    __asm volatile ("bkpt #0");
#endif

    for (;;) {
        __asm volatile ("wfi");
    }
}
