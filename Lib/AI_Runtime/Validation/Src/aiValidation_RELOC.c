/**
 ******************************************************************************
 * @file    aiValidation.c
 * @author  STMicroelectronics
 * @brief   AI Validation application - Relocatable network
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
 * - Entry points) for the AI validation of a relocatable network object.
 *   Support for a simple relocatable network (no multiple network support).
 *
 * History:
 *  - v1.0 - Initial version. Based on aiValidation v5.0
 *  - v1.1 - minor - let irq enabled if USB CDC is used
 *  - v1.2 - Use the fix cycle count overflow support
 *  - v1.3 - Add support to use SYSTICK only (remove direct call to DWT fcts)
 *  - v2.0 - Update to support fragmented activations/weights buffer
 *           activations and io buffers are fully handled by app_x-cube-ai.c/h files
 *           Align code with the new ai_buffer struct definition
 *  - v2.1 - Add support for IO tensor with shape > 4 (up to 6)
 *  - v3.0 - migration to PB msg interface 3.0
 *           align code with ai_stm32_adpaptor.h file (remove direct call of HAL_xx fcts)
 *  - v7.1 - migration to PB msg interface 3.1
 */


#if !defined(USE_OBSERVER)
#define USE_OBSERVER         1 /* 0: remove the registration of the user CB to evaluate the inference time by layer */
#endif

#define USER_REL_COPY_MODE   0

#if defined(USE_OBSERVER) && USE_OBSERVER == 1

#ifndef HAS_INSPECTOR
#define HAS_INSPECTOR
#endif

#ifdef HAS_INSPECTOR
#define HAS_OBSERVER
#endif

#endif


#define USE_CORE_CLOCK_ONLY  0 /* 1: remove usage of the HAL_GetTick() to evaluate the number of CPU clock. Only the Core
                                *    DWT IP is used. HAL_Tick() is requested to avoid an overflow with the DWT clock counter
                                *    (32b register) - USE_SYSTICK_ONLY should be set to 0.
                                */
#define USE_SYSTICK_ONLY     0 /* 1: use only the SysTick to evaluate the time-stamps (for Cortex-m0 based device, this define is forced)
                                *    (see aiTestUtility.h file)
                                */


/* System headers */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

/* APP Header files */
#include <aiValidation.h>
#include <aiTestUtility.h>
#include <aiTestHelper.h>
#include <aiPbMgr.h>



/* AI header files */
#include <ai_platform.h>
#include <core_datatypes.h>   /* AI_PLATFORM_RUNTIME_xxx definition */
#include <ai_datatypes_internal.h>
#include <core_common.h>      /* for GET_TENSOR_LIST_OUT().. definition */
#include <core_private.h>      /* for GET_TENSOR_LIST_OUT().. definition */

#include <ai_reloc_network.h>

#define _AI_RUNTIME_ID EnumAiRuntime_AI_RT_STM_AI | EnumAiApiRuntime_AI_RT_API_RELOC << EnumAiApiRuntime_AI_RT_API_POS

#if defined(HAS_OBSERVER)
#define _CAP (void *)(EnumCapability_CAP_OBSERVER | (_AI_RUNTIME_ID << 16))
#else
#define _CAP (void *)(_AI_RUNTIME_ID << 16)
#endif

/* Include the image of the relocatable network */
#include <network_img_rel.h>

#if !defined(UNUSED)
#define UNUSED(_p) AI_UNUSED((_p))
#endif

/* -----------------------------------------------------------------------------
 * TEST-related definitions
 * -----------------------------------------------------------------------------
 */

/* APP configuration 0: disabled 1: enabled */
#define _APP_DEBUG_         			0

#define _APP_VERSION_MAJOR_     (0x07)
#define _APP_VERSION_MINOR_     (0x01)
#define _APP_VERSION_   ((_APP_VERSION_MAJOR_ << 8) | _APP_VERSION_MINOR_)

#define _APP_NAME_   "AI Validation (RELOC)"

#if AI_MNETWORK_NUMBER > 1 && !defined(AI_NETWORK_MODEL_NAME)
#error Only ONE network is supported (default c-name)
#endif


/* -----------------------------------------------------------------------------
 * object definition/declaration for AI-related execution context
 * -----------------------------------------------------------------------------
 */

#ifdef HAS_OBSERVER
struct ai_network_user_obs_ctx {
  bool is_enabled;                /* indicate if the feature is enabled */
  ai_u32 n_cb_in;                 /* indicate the number of the entry cb (debug) */
  ai_u32 n_cb_out;                /* indicate the number of the exit cb (debug) */
  const reqMsg *creq;             /* reference of the current PB request */
  respMsg *cresp;                 /* reference of the current PB response */
  bool no_data;                   /* indicate that the data of the tensor should be not up-loaded */
  uint64_t tcom;                  /* number of cycles to up-load the data by layer (COM) */
  uint64_t tnodes;                /* number of cycles to execute the operators (including nn.init)
                                     nn.done is excluded but added by the adjust function */
  ai_observer_exec_ctx plt_ctx;   /* internal AI platform execution context for the observer
                                     requested to avoid dynamic allocation during the the registration */
};

struct ai_network_user_obs_ctx  net_obs_ctx; /* execution the models is serialized,
                                                only one context is requested */

#endif /* HAS_OBSERVER */

struct ai_network_exec_ctx {
  ai_handle handle;
  ai_network_report report;
  bool inputs_in_activations;
  bool outputs_in_activations;

  bool observer_is_enabled;       /* indicate if the observer is enabled */
  bool emit_intermediate_data;    /* indicate that the data from the intermediate tensors can be dumped/uploaded */
  bool simple_value;              /* indicate that only the first value has been provided and should be broadcasted
                                     to the whole input tensor */
  bool direct_write;              /* indicate that the buffer is written by an external agent */
  bool debug;
  float init_time;
  float install_time;
#ifdef HAS_OBSERVER
  struct ai_network_user_obs_ctx *obs_ctx;
#endif /* HAS_OBSERVER */
} net_exec_ctx[1] = {0};


/* RT Network buffer to relocatable network instance */
#if defined(USER_REL_COPY_MODE) && USER_REL_COPY_MODE == 1
AI_ALIGNED(32)
uint8_t reloc_ram[AI_NETWORK_RELOC_RAM_SIZE_COPY];
#else
AI_ALIGNED(32)
uint8_t reloc_ram[AI_NETWORK_RELOC_RAM_SIZE_XIP];
#endif


/* -----------------------------------------------------------------------------
 * Helper functions
 * -----------------------------------------------------------------------------
 */

static size_t _get_buffer_size(const ai_buffer* buffer)
{
  const ai_u32 batch_ = AI_BUFFER_SHAPE_ELEM(buffer, AI_SHAPE_BATCH);
  return (size_t)AI_BUFFER_BYTE_SIZE(AI_BUFFER_SIZE(buffer) * batch_, buffer->format);
}

static size_t _get_buffer_element_size(const ai_buffer* buffer)
{
  const ai_u32 batch_ = AI_BUFFER_SHAPE_ELEM(buffer, AI_SHAPE_BATCH);
  return (size_t)AI_BUFFER_SIZE(buffer) * batch_;
}

static size_t _get_element_size(const ai_buffer* buffer)
{
  const ai_bool is_binary = (AI_BUFFER_FMT_SAME(AI_BUFFER_FORMAT(buffer), AI_BUFFER_FORMAT_S1) ||
      AI_BUFFER_FMT_SAME(AI_BUFFER_FORMAT(buffer), AI_BUFFER_FORMAT_U1));
  if (is_binary)
    return 4;

  return (size_t)AI_BUFFER_BYTE_SIZE(1, buffer->format);
}

static uint32_t _ai_version_to_uint32(const ai_platform_version *version)
{
  return version->major << 24 | version->minor << 16 | version->micro << 8 | version->reserved;
}

struct _data_tensor_desc {
  const ai_buffer *buffer;
  uint32_t flags;
  float  scale;
  int32_t zero_point;
};

struct _mempool_attr_desc {
  const char* name;
  uint32_t  size;
  uintptr_t addr;
};

/* ---- Protobuf IO port adaptations ---- */

static uint32_t _shape_to_n_dims(const ai_buffer* buff)
{
  uint32_t n_dims;
  uint8_t shape_fmt;
  if (buff->shape.type == AI_SHAPE_BCWH)
    shape_fmt = EnumShapeFmt_F_SHAPE_FMT_BCWH;
  else
    shape_fmt = EnumShapeFmt_F_SHAPE_FMT_UND;
  n_dims = shape_fmt << EnumShapeFmt_F_SHAPE_FMT_POS | buff->shape.size;
  return n_dims;
}


static void fill_tensor_desc_msg(const ai_buffer *buff,
                                 aiTensorDescMsg* msg,
                                 struct _encode_uint32 *array_u32,
                                 uint32_t flags,
                                 float scale,
                                 int32_t zero_point
                                 )
{
  array_u32->size = buff->shape.size;
  array_u32->data = (uint32_t *)buff->shape.data;
  array_u32->offset = sizeof(buff->shape.data[0]);

  msg->name[0] = 0;
  msg->format = (uint32_t)buff->format;
  msg->flags = flags;

  msg->n_dims = _shape_to_n_dims(buff);

  msg->size = _get_buffer_element_size(buff);

  const ai_buffer_meta_info *meta_info = AI_BUFFER_META_INFO(buff);

  msg->scale = scale;
  msg->zeropoint = zero_point;
  if (AI_BUFFER_META_INFO_INTQ(meta_info)) {
    msg->scale = AI_BUFFER_META_INFO_INTQ_GET_SCALE(meta_info, 0);
    msg->zeropoint = AI_BUFFER_META_INFO_INTQ_GET_ZEROPOINT(meta_info, 0);
  }

  msg->addr = (uint32_t)buff->data;
}

static void encode_ai_buffer_to_tensor_desc(size_t index, void* data, aiTensorDescMsg* msg,
    struct _encode_uint32 *array_u32)
{
  struct _data_tensor_desc *info = (struct _data_tensor_desc *)data;
  ai_buffer *buff = &((ai_buffer *)(info->buffer))[index];

  fill_tensor_desc_msg(buff, msg, array_u32, info->flags, info->scale, info->zero_point);
}


static void encode_mempool_to_tensor_desc(size_t index, void* data,
                                         aiTensorDescMsg* msg,
                                         struct _encode_uint32 *array_u32)
{
  struct _mempool_attr_desc *info = (struct _mempool_attr_desc *)data;

  (void)index;

  array_u32->size = 1;
  array_u32->data = (void *)&info->size;
  array_u32->offset = 4;

  if (info->name)
    aiPbStrCopy(info->name, &msg->name[0],
        sizeof(msg->name));
  else
    msg->name[0] = 0;
  msg->format = AI_BUFFER_FORMAT_U8;
  msg->size = info->size;
  msg->n_dims = EnumShapeFmt_F_SHAPE_FMT_BCWH << EnumShapeFmt_F_SHAPE_FMT_POS | array_u32->size;
  msg->scale = 0.0;
  msg->zeropoint = 0;
  msg->addr = (uint32_t)info->addr;
  msg->flags = EnumTensorFlag_TENSOR_FLAG_MEMPOOL;
}

static uint32_t _stai_compiler_id_to(stai_compiler_id id)
{
  if (id == STAI_COMPILER_ID_GCC) {
    return EnumTools_AI_GCC;
  }
  else if (id == STAI_COMPILER_ID_GHS) {
    return EnumTools_AI_GHS;
  }
  else if  (id == STAI_COMPILER_ID_HIGHTECH) {
    return EnumTools_AI_HTC;
  }
  else if  (id == STAI_COMPILER_ID_GCC) {
    return EnumTools_AI_GCC;
  }
  else if  (id == STAI_COMPILER_ID_IAR) {
    return EnumTools_AI_IAR;
  }
  else if  (id == STAI_COMPILER_ID_KEIL_AC6) {
    return EnumTools_AI_MDK_6;
  }
  else if  (id == STAI_COMPILER_ID_KEIL) {
    return EnumTools_AI_MDK_5;
  }
  return STAI_COMPILER_ID_NONE;
}

static void send_model_info(const reqMsg *req, respMsg *resp,
    EnumState state, const ai_network_report *nn,
    bool inputs_in_acts, bool outputs_in_acts)
{
  uint32_t flags;
  resp->which_payload = respMsg_minfo_tag;

  struct ai_network_exec_ctx *ctx;

  // ctx = aiExecCtx(req->name, req->param);
  ctx = &net_exec_ctx[0];

  stai_runtime_info rt_info;
  stai_runtime_get_info(&rt_info);

  memset(&resp->payload.minfo, 0, sizeof(aiModelInfoMsg));

  aiPbStrCopy(nn->model_name, &resp->payload.minfo.name[0],
      sizeof(resp->payload.minfo.name));
  aiPbStrCopy(nn->model_signature, &resp->payload.minfo.signature[0],
      sizeof(resp->payload.minfo.signature));
  aiPbStrCopy(nn->compile_datetime, &resp->payload.minfo.compile_datetime[0],
      sizeof(resp->payload.minfo.compile_datetime));

  resp->payload.minfo.rtid = _AI_RUNTIME_ID | _stai_compiler_id_to(rt_info.compiler_id) << EnumTools_AI_TOOLS_POS;
  aiPbStrCopy(rt_info.compiler_desc, &resp->payload.minfo.runtime_desc[0],
      sizeof(resp->payload.minfo.runtime_desc));

  uint32_to_str(rt_info.runtime_build, &resp->payload.minfo.runtime_desc[strlen(rt_info.compiler_desc)],
                sizeof(resp->payload.minfo.runtime_desc) - strlen(rt_info.compiler_desc) - 1);

  /* return lib/runtime version of the compiled network_runtime lib */
  resp->payload.minfo.runtime_version = _ai_version_to_uint32((const ai_platform_version *)&rt_info.runtime_version);

  /* return tools version used to compile the model */
  resp->payload.minfo.tool_version = _ai_version_to_uint32(&nn->tool_version);

  resp->payload.minfo.n_macc = (uint64_t)nn->n_macc;
  resp->payload.minfo.n_nodes = nn->n_nodes;

  resp->payload.minfo.n_init_time = ctx->init_time;
  resp->payload.minfo.n_install_time = ctx->install_time;

  flags = EnumTensorFlag_TENSOR_FLAG_INPUT;
  if (inputs_in_acts)
    flags |= EnumTensorFlag_TENSOR_FLAG_IN_MEMPOOL;

  struct _data_tensor_desc tensor_desc_ins = {&nn->inputs[0], flags, 0.0, 0};
  struct _encode_tensor_desc tensor_ins = {
      &encode_ai_buffer_to_tensor_desc, nn->n_inputs, &tensor_desc_ins };
  resp->payload.minfo.n_inputs = nn->n_inputs;
  resp->payload.minfo.inputs.funcs.encode = encode_tensor_desc;
  resp->payload.minfo.inputs.arg = (void *)&tensor_ins;

  flags = EnumTensorFlag_TENSOR_FLAG_OUTPUT;
  if (outputs_in_acts)
    flags |= EnumTensorFlag_TENSOR_FLAG_IN_MEMPOOL;
				
  struct _data_tensor_desc tensor_desc_outs = {&nn->outputs[0], flags, 0.0, 0};
  struct _encode_tensor_desc tensor_outs = {
      &encode_ai_buffer_to_tensor_desc, nn->n_outputs, &tensor_desc_outs };
  resp->payload.minfo.n_outputs = nn->n_outputs;
  resp->payload.minfo.outputs.funcs.encode = encode_tensor_desc;
  resp->payload.minfo.outputs.arg = (void *)&tensor_outs;

  ai_size size_acts = 0;
  if (nn->map_activations.size) {
    for (int i=0; i<nn->map_activations.size; i++)
      size_acts += nn->map_activations.buffer[i].size;
   }
  struct _mempool_attr_desc tensor_desc_acts = {"acts", size_acts, 0};
  struct _encode_tensor_desc tensor_acts = {
      &encode_mempool_to_tensor_desc, 1, &tensor_desc_acts };
  resp->payload.minfo.n_activations = 1;
  resp->payload.minfo.activations.funcs.encode = encode_tensor_desc;
  resp->payload.minfo.activations.arg = (void *)&tensor_acts;

  ai_size size_params = 0;
  if (nn->map_weights.size) {
    for (int i=0; i<nn->map_weights.size; i++)
      size_params += nn->map_weights.buffer[i].size;
  }
  struct _mempool_attr_desc tensor_desc_w = {"params", size_params, 0};
  struct _encode_tensor_desc tensor_w = {
      &encode_mempool_to_tensor_desc, 1, &tensor_desc_w };
  resp->payload.minfo.n_params = 1;
  resp->payload.minfo.params.funcs.encode = encode_tensor_desc;
  resp->payload.minfo.params.arg = (void *)&tensor_w;

  aiPbMgrSendResp(req, resp, state);
}

static bool receive_ai_data(const reqMsg *req, respMsg *resp,
    EnumState state, ai_buffer *buffer, const bool simple_value,
    const bool direct_write)
{
  bool res = true;
  uint32_t temp;
  aiPbData data = { 0, _get_buffer_size(buffer), (uintptr_t)buffer->data, 0};

  if ((simple_value) || (direct_write))
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

  if ((simple_value) && (!direct_write))/* broadcast the value */
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
    EnumState state, const ai_buffer *buffer,
    const uint32_t flags,
    float scale, int32_t zero_point)
{
  struct _encode_uint32 array_u32;

  /* Build the PB message */
  resp->which_payload = respMsg_tensor_tag;

  /*-- Flags field */
  // resp->payload.tensor.flags = flags;

  /*-- Tensor desc field */
  fill_tensor_desc_msg(buffer, &resp->payload.tensor.desc, &array_u32, flags, scale, zero_point);
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

#if defined(HAS_DEDICATED_PRINT_PORT) && HAS_DEDICATED_PRINT_PORT == 1
#define PB_LC_PRINT(debug, fmt, ...) LC_PRINT(fmt, ##__VA_ARGS__)
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

#define PB_LC_PRINT(debug, fmt, ...) _print_debug(debug, fmt, ##__VA_ARGS__)
#endif

/* -----------------------------------------------------------------------------
 * Observer-related functions
 * -----------------------------------------------------------------------------
 */

#ifdef HAS_OBSERVER
static ai_u32 aiOnExecNode_cb(const ai_handle cookie,
    const ai_u32 flags,
    const ai_observer_node *node) {

  struct ai_network_exec_ctx *ctx = (struct ai_network_exec_ctx*)cookie;
  struct ai_network_user_obs_ctx  *obs_ctx = ctx->obs_ctx;

  volatile uint64_t ts = cyclesCounterEnd(); /* time stamp to mark the entry */

  if (flags & AI_OBSERVER_PRE_EVT) {
    obs_ctx->n_cb_in++;
    if (flags & AI_OBSERVER_FIRST_EVT)
      obs_ctx->tnodes = ts;
  } else if (flags & AI_OBSERVER_POST_EVT) {
    uint32_t type;
    ai_tensor_list *tl;

    cyclesCounterStart();
    /* "ts" here indicates the execution time of the
     * operator because the dwt cycle CPU counter has been
     * reset by the entry cb.
     */
    obs_ctx->tnodes += ts;
    obs_ctx->n_cb_out++;

    type = (EnumOperatorFlag_OPERATOR_FLAG_INTERNAL << 24);
    if (flags & AI_OBSERVER_LAST_EVT)
      type |= (EnumOperatorFlag_OPERATOR_FLAG_LAST << 24);
    type |= (node->type & (ai_u16)0x7FFF);

    aiOpPerf perf = {dwtCyclesToFloatMs(ts), 0,  2, (uint32_t *)&ts, -1, -1};
    perf.counter_type = EnumCounterFormat_COUNTER_FMT_64B << EnumCounterFormat_COUNTER_FMT_POS;
    perf.counter_type |= EnumCounterType_COUNTER_TYPE_CPU;

    aiPbMgrSendOperator(obs_ctx->creq, obs_ctx->cresp, EnumState_S_PROCESSING,
        NULL, type, node->id, &perf);

    tl = GET_TENSOR_LIST_OUT(node->tensors);
    AI_FOR_EACH_TENSOR_LIST_DO(i, t, tl) {
      ai_float scale = AI_TENSOR_INTEGER_GET_SCALE(t, 0);
      ai_i32 zero_point = 0;

      if (AI_TENSOR_FMT_GET_SIGN(t))
        zero_point = AI_TENSOR_INTEGER_GET_ZEROPOINT_I8(t, 0);
      else
        zero_point = AI_TENSOR_INTEGER_GET_ZEROPOINT_U8(t, 0);

      const ai_buffer_format fmt = AI_TENSOR_GET_FMT(t);
      const ai_shape *shape = AI_TENSOR_SHAPE(t);  /* Note that = ai_buffer_shape */

      ai_buffer buffer =
          AI_BUFFER_INIT(
            AI_FLAG_NONE,                                       /* flags */
            fmt,                                                /* format */
            AI_BUFFER_SHAPE_INIT_FROM_ARRAY(shape->type,
                                            shape->size,
                                            shape->data),       /* shape */
            AI_TENSOR_SIZE(t),                                  /* size */
            NULL,                                               /* meta info */
            AI_TENSOR_ARRAY_GET_DATA_ADDR(t));                  /* data */

      uint32_t tens_flags = EnumTensorFlag_TENSOR_FLAG_INTERNAL;
      if (i == (GET_TENSOR_LIST_SIZE(tl) - 1U))
        tens_flags |= EnumTensorFlag_TENSOR_FLAG_LAST;
      if (obs_ctx->no_data)
        tens_flags |= EnumTensorFlag_TENSOR_FLAG_NO_DATA;

      send_ai_io_tensor(obs_ctx->creq, obs_ctx->cresp, EnumState_S_PROCESSING,
          &buffer, tens_flags, scale, zero_point);
    }
    obs_ctx->tcom += cyclesCounterEnd();
  }

  cyclesCounterStart();
  return 0;
}
#endif


static uint64_t aiObserverAdjustInferenceTime(struct ai_network_exec_ctx *ctx,
    uint64_t tend)
{
#ifdef HAS_OBSERVER
  /* When the observer is enabled, the duration reported with
   * the output tensors is the sum of NN executing time
   * and the COM to up-load the info by layer.
   *
   * tnodes = nn.init + nn.l0 + nn.l1 ...
   * tcom   = tl0 + tl1 + ...
   * tend   = nn.done
   *
   */
  struct ai_network_user_obs_ctx  *obs_ctx = ctx->obs_ctx;
  tend = obs_ctx->tcom + obs_ctx->tnodes + tend;
#endif
  return tend;
}

static int aiObserverConfig(struct ai_network_exec_ctx *ctx)
{
#ifdef HAS_OBSERVER
  net_obs_ctx.no_data = true;
  net_obs_ctx.is_enabled = false;

  if (ctx->observer_is_enabled)
    net_obs_ctx.is_enabled = true;

  if (ctx->emit_intermediate_data) {
    net_obs_ctx.is_enabled = true;
    net_obs_ctx.no_data = false;
  }

  net_obs_ctx.tcom = 0ULL;
  net_obs_ctx.tnodes = 0ULL;
  net_obs_ctx.n_cb_in  = 0;
  net_obs_ctx.n_cb_out = 0;

  ctx->obs_ctx = &net_obs_ctx;

#endif /* HAS_OBSERVER */

return 0;
}

static int aiObserverBind(struct ai_network_exec_ctx *ctx,
    const reqMsg *creq, respMsg *cresp)
{
#ifdef HAS_OBSERVER

  ai_bool res;

  struct ai_network_user_obs_ctx  *obs_ctx = ctx->obs_ctx;

  if (obs_ctx->is_enabled == false)
    return 0;

  if (ctx->handle == AI_HANDLE_NULL)
    return -1;

  obs_ctx->creq = creq;
  obs_ctx->cresp = cresp;

  /* register the user call-back */
  res = ai_rel_platform_observer_register(ctx->handle, aiOnExecNode_cb,
      ctx, AI_OBSERVER_PRE_EVT | AI_OBSERVER_POST_EVT);
  if (!res) {
    return -1;
  }

#endif /* HAS_OBSERVER */

  return 0;
}

static int aiObserverUnbind(struct ai_network_exec_ctx *ctx)
{
#ifdef HAS_OBSERVER

  struct ai_network_user_obs_ctx  *obs_ctx = ctx->obs_ctx;

  if (obs_ctx->is_enabled == false)
    return 0;

  /* un-register the call-back */
  ai_rel_platform_observer_unregister(ctx->handle, aiOnExecNode_cb, ctx);

#endif /* HAS_OBSERVER */

  return 0;
}


/* -----------------------------------------------------------------------------
 * AI-related functions
 * -----------------------------------------------------------------------------
 */

static int aiBootstrap(struct ai_network_exec_ctx *ctx)
{
  ai_error err;
  ai_handle weights_addr;
  ai_rel_network_info rt_info;

  /* Creating an instance of the network ------------------------- */
  LC_PRINT("\r\nInstancing the network (reloc)..\r\n");

  err = ai_rel_network_rt_get_info(ai_network_reloc_img_get(), &rt_info);
  if (err.type != AI_ERROR_NONE) {
      aiLogErr(err, "ai_rel_network_rt_get_info");
      return -1;
    }

  cyclesCounterStart();

#if defined(USER_REL_COPY_MODE) && USER_REL_COPY_MODE == 1
  err = ai_rel_network_load_and_create(ai_network_reloc_img_get(),
      reloc_ram, AI_NETWORK_RELOC_RAM_SIZE_COPY, AI_RELOC_RT_LOAD_MODE_COPY,
      &ctx->handle);
#else
  err = ai_rel_network_load_and_create(ai_network_reloc_img_get(),
      reloc_ram, AI_NETWORK_RELOC_RAM_SIZE_XIP, AI_RELOC_RT_LOAD_MODE_XIP,
      &ctx->handle);
#endif

  ctx->install_time = dwtCyclesToFloatMs(cyclesCounterEnd());

  if (err.type != AI_ERROR_NONE) {
    aiLogErr(err, "ai_rel_network_load_and_create");
    return -1;
  }

  /* test returned err value (debug purpose) */
  err = ai_rel_network_get_error(ctx->handle);
  if (err.type != AI_ERROR_NONE) {
    aiLogErr(err, "ai_rel_network_get_error");
    return -1;
  }

  /* Initialize the instance --------------------------------------- */
  LC_PRINT("Initializing the network\r\n");

  weights_addr = rt_info.weights;
#if defined(AI_NETWORK_DATA_WEIGHTS_GET_FUNC) && AI_NETWORK_RELOC_WEIGHTS_SIZE > 0
  if (!weights_addr)
    weights_addr = AI_NETWORK_DATA_WEIGHTS_GET_FUNC();
#endif

  cyclesCounterStart();

  if (!ai_rel_network_init(ctx->handle,
      &weights_addr, data_activations0)) {
    err = ai_rel_network_get_error(ctx->handle);
    aiLogErr(err, "ai_rel_network_init");
    ai_rel_network_destroy(ctx->handle);
    ctx->handle = AI_HANDLE_NULL;
    return -2;
  }
  ctx->init_time = dwtCyclesToFloatMs(cyclesCounterEnd());

  /* Display the network info -------------------------------------- */
  if (ai_rel_network_get_report(ctx->handle, &ctx->report)) {
    aiPrintNetworkInfo(&ctx->report);
  } else {
    err = ai_rel_network_get_error(ctx->handle);
    aiLogErr(err, "ai_rel_network_get_info");
    ai_rel_network_destroy(ctx->handle);
    ctx->handle = AI_HANDLE_NULL;
    return -3;
  }
  
  ctx->inputs_in_activations = false;
  ctx->outputs_in_activations = false;

  for (int i = 0; i < ctx->report.n_inputs; i++) {
    if (!ctx->report.inputs[i].data)
      ctx->report.inputs[i].data = AI_HANDLE_PTR(data_ins[i]);
    else
      ctx->inputs_in_activations = true;
  }

  for (int i = 0; i < ctx->report.n_outputs; i++) {
    if (!ctx->report.outputs[i].data)
      ctx->report.outputs[i].data = AI_HANDLE_PTR(data_outs[i]);
    else
      ctx->outputs_in_activations = true;
  }

  return 0;
}

static int aiInit(void)
{
  int res;
  struct ai_network_exec_ctx *ctx = &net_exec_ctx[0];

  aiPlatformVersion();

  ctx->handle = AI_HANDLE_NULL;
  res = aiBootstrap(ctx);

  return res;
}

static void aiDeInit(void)
{
  ai_error err;
  struct ai_network_exec_ctx *ctx = &net_exec_ctx[0];

  /* Releasing the instance(s) ------------------------------------- */
  LC_PRINT("Releasing the instance...\r\n");

  if (ctx->handle != AI_HANDLE_NULL) {
    if (ai_rel_network_destroy(ctx->handle)
        != AI_HANDLE_NULL) {
      err = ai_rel_network_get_error(ctx->handle);
      aiLogErr(err, "ai_rel_network_destroy");
    }
    ctx->handle = AI_HANDLE_NULL;
  }
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
  UNUSED(param);
  struct ai_network_exec_ctx *ctx = &net_exec_ctx[0];

  if (ctx->handle && req->param == 0)
    send_model_info(req, resp, EnumState_S_IDLE, &ctx->report,
        ctx->inputs_in_activations, ctx->outputs_in_activations);
  else
    aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
        EnumError_E_INVALID_PARAM, EnumError_E_INVALID_PARAM);
}


static void _set_context(const reqMsg *req, struct ai_network_exec_ctx *ctx)
{
  ctx->emit_intermediate_data = false;
  ctx->observer_is_enabled = false;

#if defined(HAS_OBSERVER)
  if ((req->param & EnumRunParam_P_RUN_MODE_PER_LAYER) ==
      EnumRunParam_P_RUN_MODE_PER_LAYER) {
    ctx->observer_is_enabled = true;
  }

  if ((req->param & EnumRunParam_P_RUN_MODE_PER_LAYER_WITH_DATA) ==
      EnumRunParam_P_RUN_MODE_PER_LAYER_WITH_DATA) {
    ctx->observer_is_enabled = true;
    ctx->emit_intermediate_data = true;
  }
#endif

  ctx->direct_write = req->param & EnumRunParam_P_RUN_CONF_DIRECT_WRITE?true:false;
  ctx->simple_value = req->param & EnumRunParam_P_RUN_CONF_CONST_VALUE?true:false;
  ctx->debug = req->param & EnumRunParam_P_RUN_CONF_DEBUG?true:false;
}


void aiPbCmdNNRun(const reqMsg *req, respMsg *resp, void *param)
{
  ai_i32 batch;
  uint64_t tend;
  uint32_t tick;
  bool res;
  struct ai_network_exec_ctx *ctx;

  ai_buffer ai_input[AI_MNETWORK_IN_NUM];
  ai_buffer ai_output[AI_MNETWORK_OUT_NUM];

  UNUSED(param);

  ctx = &net_exec_ctx[0];

  /* 0 - Check if requested c-name model is available -------------- */
  if (!ctx->handle ||
      (strncmp(ctx->report.model_name, req->name,
          strlen(ctx->report.model_name)) != 0)) {
    aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
        EnumError_E_INVALID_PARAM, EnumError_E_INVALID_PARAM);
    return;
  }

  _set_context(req, ctx);

  aiObserverConfig(ctx);

  PB_LC_PRINT(ctx->debug, "RUN: opt=0x%x, param=0x%x, cb=%d/%d (s=%d, dw=%d)\r\n",
      req->opt, req->param,
      ctx->observer_is_enabled, ctx->emit_intermediate_data,
      ctx->simple_value, ctx->direct_write);

  /* Fill the input tensor descriptors */
  for (int i = 0; i < ctx->report.n_inputs; i++) {
    ai_input[i] = ctx->report.inputs[i];
    if (ctx->report.inputs[i].data)
      ai_input[i].data = AI_HANDLE_PTR(ctx->report.inputs[i].data);
    else
      ai_input[i].data = AI_HANDLE_PTR(data_ins[i]);
  }

  /* Fill the output tensor descriptors */
  for (int i = 0; i < ctx->report.n_outputs; i++) {
    ai_output[i] = ctx->report.outputs[i];
    if (ctx->report.outputs[i].data)
      ai_output[i].data = AI_HANDLE_PTR(ctx->report.outputs[i].data);
    else
      ai_output[i].data = AI_HANDLE_PTR(data_outs[i]);
  }
  PB_LC_PRINT(ctx->debug, "RUN: Waiting %d bytes..\r\n", _get_buffer_size(&ai_input[0]));

  /* 1 - Send a ACK (ready to receive a tensor) -------------------- */
  aiPbMgrSendAck(req, resp, EnumState_S_WAITING,
      _get_buffer_size(&ai_input[0]), EnumError_E_NONE);

  /* 2 - Receive all input tensors --------------------------------- */
  batch = 0;
  tick = port_hal_get_tick();
  for (int i = 0; i < ctx->report.n_inputs; i++) {
    /* upload a buffer */
    EnumState state = EnumState_S_WAITING;
    if ((i + 1) == ctx->report.n_inputs)
      state = EnumState_S_PROCESSING;
    batch += _get_buffer_size(&ai_input[i]);
    res = receive_ai_data(req, resp, state, &ai_input[i], ctx->simple_value, ctx->direct_write);
    if (res != true)
      return;
  }
  tick = port_hal_get_tick() - tick;
  PB_LC_PRINT(ctx->debug, "RUN: Receiving %d bytes (%d tensor(s))\r\n", batch, ctx->report.n_inputs);

  aiObserverBind(ctx, req, resp);

  /* 3 - Processing ------------------------------------------------ */
  PB_LC_PRINT(ctx->debug, "RUN: Processing..\r\n");

  tick = port_hal_get_tick();
  cyclesCounterStart();

  batch = ai_rel_network_run(ctx->handle, ai_input, ai_output);
  if (batch != 1) {
    aiLogErr(ai_rel_network_get_error(ctx->handle),
        "ai_rel_network_run");
    aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
        EnumError_E_GENERIC, EnumError_E_GENERIC);
    return;
  }
  tend = cyclesCounterEnd();
  tick = port_hal_get_tick() - tick;
  tend = aiObserverAdjustInferenceTime(ctx, tend);

  PB_LC_PRINT(ctx->debug, "RUN: Processing done. delta_tick=%d\r\n", tick);

  /* 4 - Send basic report (optional) ------------------------------ */
  aiOpPerf perf = {dwtCyclesToFloatMs(tend), 0,  2, (uint32_t *)&tend, -1, -1};
  perf.counter_type = EnumCounterFormat_COUNTER_FMT_64B << EnumCounterFormat_COUNTER_FMT_POS;
  perf.counter_type |= EnumCounterType_COUNTER_TYPE_CPU;
  aiPbMgrSendOperator(req, resp, EnumState_S_PROCESSING, ctx->report.model_name, 0, 0, &perf);

  PB_LC_PRINT(ctx->debug, "RUN: send %d output tensors\r\n", ctx->report.n_outputs);
  /* 5 - Send all output tensors ----------------------------------- */
  for (int i = 0; i < ctx->report.n_outputs; i++) {
    EnumState state = EnumState_S_PROCESSING;
    uint32_t flags =  EnumTensorFlag_TENSOR_FLAG_OUTPUT;
    if (req->param & EnumRunParam_P_RUN_MODE_PERF) {
      flags |= EnumTensorFlag_TENSOR_FLAG_NO_DATA;
    }
    if ((i + 1) == ctx->report.n_outputs) {
      state = EnumState_S_DONE;
      flags |= EnumTensorFlag_TENSOR_FLAG_LAST;
    }
    send_ai_io_tensor(req, resp, state, &ai_output[i], flags, 0.0, 0);
  }

  aiObserverUnbind(ctx);
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
#if defined(USE_USB_CDC_CLASS) && USE_USB_CDC_CLASS == 1
    LC_PRINT("# Note: USB_CDC_CLASS is enabled to transfer the data.\r\n");
#else
    LC_PRINT("# Note: At this point, default ASCII-base terminal should be closed\r\n");
    LC_PRINT("# and a serial COM interface should be used\r\n");
#endif
    LC_PRINT("# (i.e. Python ai_runner module). Protocol version = %d.%d\r\n",
        EnumVersion_P_VERSION_MAJOR,
        EnumVersion_P_VERSION_MINOR);
  }

  aiPbMgrInit(pbCmdFuncTab);

  /* used only by Stellar MCUs, empty for all other MCUs */
  port_io_init();

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

