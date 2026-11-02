/**
 ******************************************************************************
 * @file    aiValidation_ST_AI.c
 * @author  STMicroelectronics
 * @brief   AI Validation application (ST AI embedded c-api)
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 - 2026 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software is licensed under terms that can be found in the LICENSE file in
 * the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Description:
 *  Entry points for AI validation on-target process.
 *
 * History:
 *  - v1.0 - Initial version (ST AI embedded c-api)
 *  - v1.1 - Remove old AI type (macros & specific type)
 *  - v1.2 - Fix to support multiple models.
 *
 */

#if !defined(USE_OBSERVER)
#define USE_OBSERVER         1 /* 0: remove the registration of the user CB to evaluate the inference time by layer */
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
#include <stdarg.h>

/* APP Header files */
#include <aiValidation.h>
#include <aiTestUtility.h>
#include <aiTestHelper_ST_AI.h>
#include <aiPbMgr.h>


/* AI header files */
#include <stai.h>


#if defined(SR5E1) || defined(SR6X)
#include <app_stellar-studio-ai.h>
#else
#include <app_x-cube-ai.h>
#endif

#define _AI_RUNTIME_ID (EnumAiRuntime_AI_RT_STM_AI | EnumAiApiRuntime_AI_RT_API_ST_AI << EnumAiApiRuntime_AI_RT_API_POS)


#if defined(USE_OBSERVER) && USE_OBSERVER == 1
#define _CAP (void *)(EnumCapability_CAP_OBSERVER | (_AI_RUNTIME_ID << 16))
#else
#define _CAP (void *)(_AI_RUNTIME_ID << 16)
#endif

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

/* -----------------------------------------------------------------------------
 * TEST-related definitions
 * -----------------------------------------------------------------------------
 */

/* APP configuration 0: disabled 1: enabled */
#define _APP_DEBUG_                     0

#define _APP_VERSION_MAJOR_     (0x01)
#define _APP_VERSION_MINOR_     (0x02)
#define _APP_VERSION_           ((_APP_VERSION_MAJOR_ << 8) | _APP_VERSION_MINOR_)

#define _APP_NAME_   "AI Validation (ST.AI c-api)"


#define _IN_OUT_MAX_NUMBER (32)

#if (STAI_MNETWORK_IN_NUM > _IN_OUT_MAX_NUMBER) || (STAI_MNETWORK_OUT_NUM > _IN_OUT_MAX_NUMBER)
#error Number of IO exceeds the supported number (max. 32).
#endif

struct ai_network_exec_ctx {
  stai_ptr handle;
  stai_network_info report;
  tensor_descs inputs;
  tensor_descs outputs;
  tensor_descs acts;
  tensor_descs states;
  tensor_descs params;

  /* Handle the physical address of the IO buffers */
  stai_ptr   in_addr[_IN_OUT_MAX_NUMBER];
  stai_ptr   out_addr[_IN_OUT_MAX_NUMBER];

  bool debug;

  bool simple_value;              /* indicate that only the first value has been provided and should be broadcasted
                                     to the whole input tensor */

  bool direct_write;              /* indicate that the data is directly written by an external agent */

  float init_time;

#if defined(USE_OBSERVER) && USE_OBSERVER == 1
  uint32_t  c_idx;
  const stai_network_details *details;
  uint64_t tnodes;                /* number of cycles to execute the operators (including nn.init)
                                     nn.done is excluded but added by the adjust function */
  uint64_t tcom;
  const reqMsg *creq;             /* reference of the current PB request */
  respMsg *cresp;                 /* reference of the current PB response */
  bool observer_is_enabled;       /* indicate if the observer is enabled */
  bool emit_intermediate_data;    /* indicate that the data from the intermediate tensors can be dumped/uploaded */
#endif

} net_exec_ctx[STAI_MNETWORK_NUMBER] = {0};


/* -----------------------------------------------------------------------------
 * Helper functions
 * -----------------------------------------------------------------------------
 */

/* return number of element in a given tensor */
static size_t _get_tensor_size(const stai_tensor* tensor)
{
  int32_t nb_elem = 1;
  for (uint32_t i=0; i < tensor->shape.size; i++)
    nb_elem *= tensor->shape.data[i];
  return nb_elem;
}

static bool _is_32b_packed(const stai_tensor* tensor)
{
  const bool is_32b_packed = (STAI_FORMAT_GET_TYPE(tensor->format) == STAI_FORMAT_TYPE_Q) &&
      (STAI_FORMAT_GET_BITS(tensor->format) < 8);
  return is_32b_packed;
}

/* return element size in bytes */
static size_t _get_tensor_element_size(const stai_tensor* tensor)
{
  if (_is_32b_packed(tensor)) {
    return 4;
  }
  return STAI_FORMAT_GET_BITS(tensor->format) >> 3;
}

/* return size in bytes w/o extra bytes */
static size_t _get_tensor_size_bytes(const stai_tensor* tensor)
{
  if (_is_32b_packed(tensor)) {
    /* ! no extra bytes is expected with 32b packed tensor type */
    return (size_t)tensor->size_bytes;
  }
  return _get_tensor_size(tensor) * _get_tensor_element_size(tensor);
}

static uint32_t _stai_shape_to_n_dims(const stai_tensor* tensor)
{
  uint32_t n_dims;
  uint8_t shape_fmt = EnumShapeFmt_F_SHAPE_FMT_ST_AI;
  if (tensor->flags & STAI_FLAG_HAS_BATCH)
    shape_fmt |= EnumShapeFmt_F_SHAPE_FMT_HAS_BATCH;
  if (tensor->flags & STAI_FLAG_CHANNEL_FIRST)
    shape_fmt |= EnumShapeFmt_F_SHAPE_FMT_CHANNEL_FIRST;
  if (tensor->flags & STAI_FLAG_CHANNEL_LAST)
    shape_fmt |= EnumShapeFmt_F_SHAPE_FMT_CHANNEL_LAST;
  n_dims = shape_fmt << EnumShapeFmt_F_SHAPE_FMT_POS | tensor->shape.size;
  return n_dims;
}

static uint32_t _stai_version_to_uint32(const stai_version *version)
{
  return version->major << 24 | version->minor << 16 | version->micro << 8 | version->reserved;
}

struct _data_tensor_desc {
  const tensor_descs *tens_desc;
  uint32_t flags;
  float  scale;           /* to set the default value */
  int32_t zero_point;     /* to set the default value */
};

struct _data_mempool_desc {
  const char* name;
  uint32_t  size;
  uintptr_t addr;
};


/* -----------------------------------------------------------------------------
 * Protobuf IO port adaptations
 * -----------------------------------------------------------------------------
 */

static void fill_tensor_desc_msg(const stai_tensor *tens,
    uintptr_t address,
    aiTensorDescMsg* msg,
    struct _encode_uint32 *array_u32,
    uint32_t flags,
    float scale,
    int32_t zero_point,
    bool has_batch
)
{
  array_u32->size = tens->shape.size;
  array_u32->data = (uint32_t *)tens->shape.data;
  array_u32->offset = sizeof(tens->shape.data[0]);

  msg->name[0] = 0;
  if (tens->name)
    aiPbStrCopy(tens->name, &msg->name[0], sizeof(msg->name));

  msg->format = (uint32_t)tens->format;
  msg->flags = flags;

  msg->n_dims = _stai_shape_to_n_dims(tens);
  if (has_batch)
    msg->n_dims |= EnumShapeFmt_F_SHAPE_FMT_HAS_BATCH << EnumShapeFmt_F_SHAPE_FMT_POS;
  msg->size = _get_tensor_size(tens);

  msg->scale = scale;
  msg->zeropoint = zero_point;
  if (tens->scale.size > 0) {
    msg->scale = tens->scale.data[0];
    msg->zeropoint = tens->zeropoint.data[0];
  }

  msg->addr = (uint32_t)address;
}

static void encode_ai_buffer_to_tensor_desc(size_t index, void* data,
    aiTensorDescMsg* msg,
    struct _encode_uint32 *array_u32)
{
  struct _data_tensor_desc *info = (struct _data_tensor_desc *)data;
  const stai_tensor *buff = &info->tens_desc->tensors[index];

  fill_tensor_desc_msg(buff, info->tens_desc->address[index],
      msg, array_u32, info->flags, info->scale, info->zero_point, false);
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
    EnumState state, struct ai_network_exec_ctx* ctx)
{
  uint32_t flags;

  stai_runtime_info rt_info;
  stai_runtime_get_info(&rt_info);

  resp->which_payload = respMsg_minfo_tag;

  memset(&resp->payload.minfo, 0, sizeof(aiModelInfoMsg));

  aiPbStrCopy(ctx->report.c_model_name, &resp->payload.minfo.name[0],
      sizeof(resp->payload.minfo.name));
  aiPbStrCopy(ctx->report.model_signature, &resp->payload.minfo.signature[0],
      sizeof(resp->payload.minfo.signature));
  aiPbStrCopy(ctx->report.c_compile_datetime, &resp->payload.minfo.compile_datetime[0],
      sizeof(resp->payload.minfo.compile_datetime));

  resp->payload.minfo.rtid = _AI_RUNTIME_ID | _stai_compiler_id_to(rt_info.compiler_id) << EnumTools_AI_TOOLS_POS;
  aiPbStrCopy(rt_info.compiler_desc, &resp->payload.minfo.runtime_desc[0],
      sizeof(resp->payload.minfo.runtime_desc));

  uint32_to_str(rt_info.runtime_build, &resp->payload.minfo.runtime_desc[strlen(rt_info.compiler_desc)],
                sizeof(resp->payload.minfo.runtime_desc) - strlen(rt_info.compiler_desc) - 1);

  /* return lib/runtime version of the compiled network_runtime lib */
  resp->payload.minfo.runtime_version = _stai_version_to_uint32(&rt_info.runtime_version);

  /* return tools version used to compile the model */
  resp->payload.minfo.tool_version = _stai_version_to_uint32(&ctx->report.tool_version);

  resp->payload.minfo.n_macc = (uint64_t)ctx->report.n_macc;
#if defined(USE_OBSERVER) && USE_OBSERVER == 1
  resp->payload.minfo.n_nodes = ctx->details->n_nodes;
#else
  resp->payload.minfo.n_nodes = ctx->report.n_nodes;
#endif

  resp->payload.minfo.n_init_time = ctx->init_time;

  flags = EnumTensorFlag_TENSOR_FLAG_INPUT;
  if (ctx->report.flags & STAI_FLAG_INPUTS)
    flags |= EnumTensorFlag_TENSOR_FLAG_IN_MEMPOOL;

  struct _data_tensor_desc tensor_desc_ins = {&ctx->inputs, flags, 0.0, 0};
  struct _encode_tensor_desc tensor_ins = {
      &encode_ai_buffer_to_tensor_desc, ctx->report.n_inputs, &tensor_desc_ins };
  resp->payload.minfo.n_inputs = ctx->report.n_inputs;
  resp->payload.minfo.inputs.funcs.encode = encode_tensor_desc;
  resp->payload.minfo.inputs.arg = (void *)&tensor_ins;

  flags = EnumTensorFlag_TENSOR_FLAG_OUTPUT;
  if (ctx->report.flags & STAI_FLAG_OUTPUTS)
    flags |= EnumTensorFlag_TENSOR_FLAG_IN_MEMPOOL;

  struct _data_tensor_desc tensor_desc_outs = {&ctx->outputs, flags, 0.0, 0};
  struct _encode_tensor_desc tensor_outs = {
      &encode_ai_buffer_to_tensor_desc, ctx->report.n_outputs, &tensor_desc_outs };
  resp->payload.minfo.n_outputs = ctx->report.n_outputs;
  resp->payload.minfo.outputs.funcs.encode = encode_tensor_desc;
  resp->payload.minfo.outputs.arg = (void *)&tensor_outs;

  flags = EnumTensorFlag_TENSOR_FLAG_MEMPOOL;
  if (ctx->report.flags & STAI_FLAG_ACTIVATIONS)
    flags |= EnumTensorFlag_TENSOR_FLAG_IN_MEMPOOL;
  struct _data_tensor_desc tensor_desc_acts = {&ctx->acts, flags, 0.0, 0};
  struct _encode_tensor_desc tensor_acts = {
      &encode_ai_buffer_to_tensor_desc, ctx->report.n_activations, &tensor_desc_acts };
  resp->payload.minfo.n_activations = ctx->report.n_activations;
  resp->payload.minfo.activations.funcs.encode = encode_tensor_desc;
  resp->payload.minfo.activations.arg = (void *)&tensor_acts;

  flags = EnumTensorFlag_TENSOR_FLAG_MEMPOOL;
  if (ctx->report.flags & STAI_FLAG_WEIGHTS)
    flags |= EnumTensorFlag_TENSOR_FLAG_IN_MEMPOOL;
  struct _data_tensor_desc tensor_desc_params = {&ctx->params, flags, 0.0, 0};
  struct _encode_tensor_desc tensor_params = {
      &encode_ai_buffer_to_tensor_desc, ctx->report.n_weights, &tensor_desc_params };
  resp->payload.minfo.n_params = ctx->report.n_weights;
  resp->payload.minfo.params.funcs.encode = encode_tensor_desc;
  resp->payload.minfo.params.arg = (void *)&tensor_params;

  aiPbMgrSendResp(req, resp, state);
}

static bool _receive_ai_data(const reqMsg *req, respMsg *resp,
    EnumState state, const stai_tensor *buffer, const uintptr_t address,
    bool simple_value, bool direct_write)
{
  bool res = true;
  uint32_t temp;
  aiPbData data = { 0, _get_tensor_size_bytes(buffer), address, 0};

  if ((simple_value) || (direct_write))
    data.size = _get_tensor_element_size(buffer);
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

    if ((simple_value) && (!direct_write)) /* broadcast the value */
    {
      const size_t el_s = data.size;
      const uintptr_t r_ptr = address;

      uintptr_t w_ptr = r_ptr + el_s;
      for (size_t pos = 1; pos <  _get_tensor_size_bytes(buffer) / el_s; pos++)
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

static bool _send_ai_io_tensor(const reqMsg *req, respMsg *resp,
    EnumState state, const stai_tensor *buffer, const uintptr_t address,
    const uint32_t flags,
    float scale, int32_t zero_point, bool has_batch)
{
  struct _encode_uint32 array_u32;

  /* Build the PB message */
  resp->which_payload = respMsg_tensor_tag;

  /*-- Flags field */
  // resp->payload.tensor.flags = flags;

  /*-- Tensor desc field */
  fill_tensor_desc_msg(buffer, address, &resp->payload.tensor.desc, &array_u32,
      flags, scale, zero_point, has_batch);
  resp->payload.tensor.desc.dims.funcs.encode = encode_uint32;
  resp->payload.tensor.desc.dims.arg = &array_u32;

  /*-- Data field */
  resp->payload.tensor.data.addr = (uint32_t)address;

  if (flags & EnumTensorFlag_TENSOR_FLAG_NO_DATA) {
    resp->payload.tensor.data.size = 0;
  } else {
    resp->payload.tensor.data.size = _get_tensor_size_bytes(buffer);
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

#define PB_LC_PRINT(debug, fmt, ...) _print_debug(debug, fmt, ##__VA_ARGS__)
#define PB_LC_STAT(_cat, _sub_cat, fmt, ...) _print_debug(true, "s:" _cat ":" _sub_cat ":" fmt "\r\n", ##__VA_ARGS__)
#endif


#if defined(USE_OBSERVER) && USE_OBSERVER == 1

/* -----------------------------------------------------------------------------
 * Observer services
 * -----------------------------------------------------------------------------
 */

#include "stai_events.h"

static void _stai_event_cb(void* cb_cookie, const stai_event_type event_type,
    const void* event_payload)
{
  volatile uint64_t ts = cyclesCounterEnd(); /* time stamp to mark the entry */
  cyclesCounterStart();

  struct ai_network_exec_ctx *ctx = (struct ai_network_exec_ctx *)cb_cookie;
  const stai_event_node_start_stop *evt = (stai_event_node_start_stop *)event_payload;
  const stai_network_details *details = ctx->details;

  // PB_LC_PRINT(ctx->debug, "RUN:CB even_type=%d (c_idx=%d)\r\n",
  //    (int)event_type, ctx->c_idx);

  if (event_type == STAI_EVENT_NODE_START && ctx->c_idx == 0) {
    ctx->tnodes = ts;  // nn.init cycles
  }

  if (event_type == STAI_EVENT_NODE_STOP) {
    uint32_t type;

    const stai_array_s32 *input_details = &details->nodes[ctx->c_idx].input_tensors;
    const stai_array_s32 *output_details = &details->nodes[ctx->c_idx].output_tensors;

    type = (EnumOperatorFlag_OPERATOR_FLAG_INTERNAL << 24);
    if (ctx->c_idx == details->n_nodes - 1)
      type |= (EnumOperatorFlag_OPERATOR_FLAG_LAST << 24);
    type |= (details->nodes[ctx->c_idx].type & (uint16_t)0x7FFF);

    aiOpPerf perf = {dwtCyclesToFloatMs(ts), 0,  2, (uint32_t *)&ts, -1, -1};
    perf.counter_type = EnumCounterFormat_COUNTER_FMT_64B << EnumCounterFormat_COUNTER_FMT_POS;
    perf.counter_type |= EnumCounterType_COUNTER_TYPE_CPU;

    PB_LC_PRINT(ctx->debug, "RUN:CB node(%lu) %d,0x%x,%s,%d->%d",
        ctx->c_idx,
        (int)details->nodes[ctx->c_idx].id,
        details->nodes[ctx->c_idx].type,
        uint64ToDecimal(ts),
        (int)input_details->size,
        (int)output_details->size);

    PB_LC_STAT("node", "end", "%d:%s:%ld", ctx->c_idx, "??", ts);

    if (evt->node_id != details->nodes[ctx->c_idx].id) {
      PB_LC_PRINT(ctx->debug, "RUN:CB ERROR -- Mismatch between event id and id - %ld vs %ld", evt->node_id,
          details->nodes[ctx->c_idx].id);
      type |= (EnumOperatorFlag_OPERATOR_FLAG_WITHOUT_TENSOR << 24);
    }

    if (output_details->size != evt->buffers.size) {
      PB_LC_PRINT(ctx->debug, "RUN:CB ERROR -- Mismatch between number of tensor and buffer - %ld vs %ld",
          output_details->size, evt->buffers.size);
      type |= (EnumOperatorFlag_OPERATOR_FLAG_WITHOUT_TENSOR << 24);
    }

    aiPbMgrSendOperator(ctx->creq, ctx->cresp, EnumState_S_PROCESSING,
        NULL, type, details->nodes[ctx->c_idx].id, &perf);

    for (uint32_t i = 0; i < evt->buffers.size; i++) {
      const int idx_tens = output_details->data[i];
      const stai_tensor *tensor = &details->tensors[idx_tens];

      PB_LC_PRINT(ctx->debug, "RUN:CB tensor(%ld) %d,%s,%d/%d,0x%x",
          i, idx_tens,
          tensor->name,
          _get_tensor_size(tensor),
          _get_tensor_size_bytes(tensor),
          (uintptr_t)evt->buffers.data[i]
      );

      uint32_t tens_flags = EnumTensorFlag_TENSOR_FLAG_INTERNAL;
      if (i == evt->buffers.size - 1)
        tens_flags |= EnumTensorFlag_TENSOR_FLAG_LAST;
      if (!ctx->emit_intermediate_data)
        tens_flags |= EnumTensorFlag_TENSOR_FLAG_NO_DATA;

      _send_ai_io_tensor(ctx->creq, ctx->cresp, EnumState_S_PROCESSING, tensor,
          (uintptr_t)evt->buffers.data[i], tens_flags, 0.0, 0, true);
    }

    ctx->tnodes += ts;  // nn.node(x) cycles

    ctx->c_idx += 1;
  }

  ctx->tcom += cyclesCounterEnd();
  cyclesCounterStart();
}

static uint64_t adjustInferenceTime(struct ai_network_exec_ctx *ctx, uint64_t tend)
{
  if (ctx->observer_is_enabled) {
    PB_LC_PRINT(ctx->debug, "RUN:adjust cycles %s + %s",
        uint64ToDecimal(tend), uint64ToDecimal(ctx->tnodes));
    return tend + ctx->tnodes;
  }
  return tend;
}

#endif /* USE_OBSERVER == 1 */

/* -----------------------------------------------------------------------------
 * AI-related functions
 * -----------------------------------------------------------------------------
 */

static struct ai_network_exec_ctx *aiExecCtx(const char *nn_name, int pos)
{
  struct ai_network_exec_ctx *cur = NULL;

  if (!nn_name)
    return NULL;

  if (!nn_name[0]) {
    if ((pos >= 0) && (pos < STAI_MNETWORK_NUMBER) && net_exec_ctx[pos].handle)
      cur = &net_exec_ctx[pos];
  } else {
    int idx;
    for (idx=0; idx < STAI_MNETWORK_NUMBER; idx++) {
      cur = &net_exec_ctx[idx];
      if (cur->handle &&
          (strlen(cur->report.c_model_name) == strlen(nn_name)) &&
          (strncmp(cur->report.c_model_name, nn_name,
              strlen(cur->report.c_model_name)) == 0)) {
        break;
      }
      cur = NULL;
    }
  }
  return cur;
}

static int aiBootstrap(struct ai_network_exec_ctx *ctx, const char *nn_name)
{
  stai_return_code stai_err;

  /* Creating the instance of the  network ------------------------- */
  LC_PRINT("Creating the instance for \"%s\" model..\r\n", nn_name);

  cyclesCounterStart();

  stai_err = stai_mnetwork_init(nn_name, &ctx->handle);
  if (stai_err != STAI_SUCCESS) {
    stai_log_err(stai_err, "ai_mnetwork_get_info");
    return -1;
  }

  ctx->init_time = dwtCyclesToFloatMs(cyclesCounterEnd());

  /* Initialize the instance --------------------------------------- */
  LC_PRINT("Initializing..\r\n");

  stai_err = stai_mnetwork_get_info(ctx->handle, &ctx->report);
  if (stai_err != STAI_SUCCESS) {
    stai_log_err(stai_err, "ai_mnetwork_get_info");
    stai_mnetwork_deinit(ctx->handle);
    ctx->handle = NULL;
  }

  ctx->inputs.tensors = ctx->report.inputs;
  ctx->outputs.tensors = ctx->report.outputs;
  ctx->acts.tensors = ctx->report.activations;
  ctx->params.tensors = ctx->report.weights;

#if defined(USE_OBSERVER) && USE_OBSERVER == 1
  ctx->details = stai_mnetwork_get_details(ctx->handle);
#endif

  /* Set the activations ------------------------------------------- */
  LC_PRINT(" Set activation buffers..\r\n");
  if (ctx->report.flags & STAI_FLAG_ACTIVATIONS)
    stai_err = stai_mnetwork_get_activations(ctx->handle, data_activations);
  else
    stai_err = stai_mnetwork_set_activations(ctx->handle, data_activations);
  if (stai_err != STAI_SUCCESS) {
    stai_log_err(stai_err, "stai_mnetwork_set_activations");
    stai_mnetwork_deinit(ctx->handle);
    ctx->handle = NULL;
  }
  ctx->acts.n_tensor = ctx->report.n_activations;
  ctx->acts.address = (uintptr_t *)data_activations;

  /* Set the states ------------------------------------------- */
  LC_PRINT(" Set state buffers..\r\n");
  if (ctx->report.flags & STAI_FLAG_STATES)
    stai_err = stai_mnetwork_get_states(ctx->handle, data_states);
  else
    stai_err = stai_mnetwork_set_states(ctx->handle, data_states);
  if (stai_err != STAI_SUCCESS) {
    stai_log_err(stai_err, "stai_mnetwork_set_states");
    stai_mnetwork_deinit(ctx->handle);
    ctx->handle = NULL;
  }

  ctx->states.n_tensor = ctx->report.n_states;
  ctx->states.tensors = ctx->report.states;
  ctx->states.address = (uintptr_t *)data_states;

  /* Set params/weights -------------------------------------------- */
  ctx->params.n_tensor = ctx->report.n_weights;
  ctx->params.address = (uintptr_t *)stai_mnetwork_get_weights_ext(ctx->handle);

  /* Set IO -------------------------------------------------------- */
  LC_PRINT(" Set IO buffers..\r\n");
  if (ctx->report.flags & STAI_FLAG_INPUTS)
    stai_err = stai_mnetwork_get_inputs(ctx->handle, &ctx->in_addr[0]);
  else {
    for (int i=0; i< ctx->report.n_inputs; i++) {
      ctx->in_addr[i] = data_ins[i];
    }
    stai_err = stai_mnetwork_set_inputs(ctx->handle, &ctx->in_addr[0]);
  }
  if (stai_err != STAI_SUCCESS) {
    stai_log_err(stai_err, "stai_mnetwork_set/get_inputs");
    stai_mnetwork_deinit(ctx->handle);
    ctx->handle = NULL;
  }
  ctx->inputs.n_tensor = ctx->report.n_inputs;
  ctx->inputs.address = (uintptr_t *)&ctx->in_addr[0];

  if (ctx->report.flags & STAI_FLAG_OUTPUTS)
    stai_err = stai_mnetwork_get_outputs(ctx->handle, &ctx->out_addr[0]);
  else {
    for (int i=0; i< ctx->report.n_outputs; i++) {
      ctx->out_addr[i] = data_outs[i];
    }
    stai_err = stai_mnetwork_set_outputs(ctx->handle, &ctx->out_addr[0]);
  }

  if (stai_err != STAI_SUCCESS) {
    stai_log_err(stai_err, "stai_mnetwork_set/get_outputs");
    stai_mnetwork_deinit(ctx->handle);
    ctx->handle = NULL;
  }
  ctx->outputs.n_tensor = ctx->report.n_outputs;
  ctx->outputs.address = (uintptr_t *)&ctx->out_addr[0];

  /* Display the network info -------------------------------------- */
  stai_log_network_info(&ctx->report,
      &ctx->inputs, &ctx->outputs, &ctx->acts, &ctx->states, &ctx->params);

  return 0;
}

static int aiInit(void)
{
  int res = -1;
  const char *nn_name;
  int idx;

  stai_platform_version();

  /* Reset the contexts -------------------------------------------- */
  for (idx=0; idx < STAI_MNETWORK_NUMBER; idx++) {
    net_exec_ctx[idx].handle = NULL;
  }

  /* Discover and initialize the network(s) ------------------------ */
  LC_PRINT("Discovering the network(s)...\r\n");

  idx = 0;
  do {
    nn_name = stai_mnetwork_find(NULL, idx);
    if (nn_name) {
      LC_PRINT("\r\nFound network \"%s\"\r\n", nn_name);
      res = aiBootstrap(&net_exec_ctx[idx], nn_name);
      if (res)
        nn_name = NULL;
    }
    idx++;
  } while (nn_name);

  return res;
}

static void aiDeInit(void)
{
  stai_return_code stai_err;
  int idx;

  /* Releasing the instance(s) ------------------------------------- */
  LC_PRINT("Releasing the instance(s)...\r\n");

  for (idx=0; idx<STAI_MNETWORK_NUMBER; idx++) {
    if (net_exec_ctx[idx].handle != NULL) {
      stai_err = stai_mnetwork_deinit(net_exec_ctx[idx].handle);
      if (stai_err != STAI_SUCCESS) {
        stai_log_err(stai_err, "ai_mnetwork_deinit");
      }
      net_exec_ctx[idx].handle = NULL;
    }
  }
}


/* -----------------------------------------------------------------------------
 * Specific test APP commands
 * -----------------------------------------------------------------------------
 */

static void aiPbCmdSysInfo(const reqMsg *req, respMsg *resp, void *param)
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

static void aiPbCmdNNInfo(const reqMsg *req, respMsg *resp, void *param)
{
  struct ai_network_exec_ctx *ctx;

  UNUSED(param);

  ctx = aiExecCtx(req->name, req->param);
  if (ctx) {
    send_model_info(req, resp, EnumState_S_IDLE, ctx);
  }
  else
    aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
        EnumError_E_INVALID_PARAM, EnumError_E_INVALID_PARAM);
}

static void _set_context(const reqMsg *req, struct ai_network_exec_ctx *ctx)
{
  ctx->simple_value = req->param & EnumRunParam_P_RUN_CONF_CONST_VALUE?true:false;
  ctx->direct_write = req->param & EnumRunParam_P_RUN_CONF_DIRECT_WRITE?true:false;
  ctx->debug = req->param & EnumRunParam_P_RUN_CONF_DEBUG?true:false;

#if defined(USE_OBSERVER) && USE_OBSERVER == 1
  ctx->observer_is_enabled = false;
  ctx->emit_intermediate_data = false;

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
}

static void _reset_states(const reqMsg *req, struct ai_network_exec_ctx *ctx)
{
  if (((req->opt & 1) != 1) || (ctx->states.n_tensor == 0))
    return;
  PB_LC_PRINT(ctx->debug, "RUN: resetting states..\r\n");
  for (size_t i = 0; i < ctx->states.n_tensor; i++)
  {
    memset((void *)ctx->states.address[i], 0, ctx->states.tensors[i].size_bytes);
  }
}

static void aiPbCmdNNRun(const reqMsg *req, respMsg *resp, void *param)
{
  stai_return_code stai_err;
  uint64_t tend;
  bool res;
  struct ai_network_exec_ctx *ctx;
  uint32_t tick;

  const stai_tensor *ai_input;
  const stai_tensor *ai_output;

  UNUSED(param);

  MON_STACK_INIT();

  /* 0 - Check if requested c-name model is available -------------- */
  ctx = aiExecCtx(req->name, -1);
  if (!ctx) {
    aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
        EnumError_E_INVALID_PARAM, EnumError_E_INVALID_PARAM);
    return;
  }

  _set_context(req, ctx);

  ai_input = ctx->inputs.tensors;
  ai_output = ctx->outputs.tensors;

  PB_LC_PRINT(ctx->debug, "RUN: c-model=%s rtid=%d (c-api=%d)\r\n", ctx->report.c_model_name,
      _AI_RUNTIME_ID & 0xFF, _AI_RUNTIME_ID >> EnumAiApiRuntime_AI_RT_API_POS);
  PB_LC_PRINT(ctx->debug, "RUN:  observer=%d/%d, simple_value=%d, direct_write=%d\r\n",
#if defined(USE_OBSERVER) && USE_OBSERVER == 1
      ctx->observer_is_enabled, ctx->emit_intermediate_data,
#else
      0, 0,
#endif
      ctx->simple_value, ctx->direct_write);

  PB_LC_PRINT(ctx->debug, "RUN: Waiting data (%d bytes).. opt=0x%lx, param=0x%lx\r\n",
      _get_tensor_size_bytes(&ai_input[0]),
      req->opt, req->param);

  /* 1 - Send a ACK (ready to receive a tensor) -------------------- */
  aiPbMgrSendAck(req, resp, EnumState_S_WAITING,
      _get_tensor_size_bytes(&ai_input[0]), EnumError_E_NONE);

  /* 2 - Receive all input tensors --------------------------------- */
  tick = port_hal_get_tick();
  for (int i = 0; i < ctx->report.n_inputs; i++) {
    /* upload a buffer */
    EnumState state = EnumState_S_WAITING;
    if ((i + 1) == ctx->report.n_inputs)
      state = EnumState_S_PROCESSING;
    res = _receive_ai_data(req, resp, state, &ai_input[i], ctx->inputs.address[i],
        ctx->simple_value, ctx->direct_write);
    if (res != true)
      return;
  }

#if defined(USE_OBSERVER) && USE_OBSERVER == 1
  if (ctx->observer_is_enabled) {
    ctx->c_idx = 0;
    ctx->tnodes = 0UL;
    ctx->tcom = 0UL;
    ctx->creq = req;
    ctx->cresp = resp;
    stai_mnetwork_set_callback(ctx->handle, _stai_event_cb, ctx);
  } else {
    stai_mnetwork_set_callback(ctx->handle, NULL, ctx);
  }
#endif

  tick = port_hal_get_tick() - tick;
  PB_LC_PRINT(ctx->debug, "RUN: %ld ticks to download %d input(s)\r\n", tick, ctx->report.n_inputs);

  _reset_states(req, ctx);

  MON_ALLOC_RESET();
  MON_ALLOC_ENABLE();

  /* 3 - Processing ------------------------------------------------ */
#if !defined(SR6X)
  PB_LC_PRINT(ctx->debug, "RUN: Processing.. tick=%ld\r\n", port_hal_get_tick());
#else
  PB_LC_PRINT(ctx->debug, "RUN: Processing.. tick=%lld\r\n", port_hal_get_tick());
#endif

  MON_STACK_CHECK0();
  MON_STACK_MARK();

  tick = port_hal_get_tick();
  cyclesCounterStart();

  stai_err = stai_mnetwork_run(ctx->handle);
  if (stai_err != STAI_SUCCESS) {
    stai_log_err(stai_err, "ai_mnetwork_run");
    aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
        EnumError_E_GENERIC, EnumError_E_GENERIC);
    return;
  }

  tend = cyclesCounterEnd();
  tick = port_hal_get_tick() - tick;

#if defined(USE_OBSERVER) && USE_OBSERVER == 1
  tend = adjustInferenceTime(ctx, tend);
#endif

  MON_ALLOC_DISABLE();
  MON_STACK_EVALUATE();

  PB_LC_PRINT(ctx->debug, "RUN: Processing done. delta_tick=%ld\r\n", tick);

  /* 4 - Send basic report (optional) ------------------------------ */
#if defined(_APP_STACK_MONITOR_) && _APP_STACK_MONITOR_ == 1 && defined(_IS_GCC_COMPILER) && _IS_GCC_COMPILER
  aiOpPerf perf = {dwtCyclesToFloatMs(tend), 0,  2, (uint32_t *)&tend, io_stack.susage, io_malloc.used};
#if defined(USE_OBSERVER) && USE_OBSERVER == 1
  if (ctx->observer_is_enabled) {
    perf.stack_usage = -1;
    perf.heap_usage = -1;
  }
#endif
#else
  aiOpPerf perf = {dwtCyclesToFloatMs(tend), 0,  2, (uint32_t *)&tend, -1, -1};
#endif
  
  aiPbMgrSendOperator(req, resp, EnumState_S_PROCESSING, ctx->report.c_model_name, 0, 0, &perf);

  PB_LC_PRINT(ctx->debug, "RUN: send output tensors\r\n");
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
    _send_ai_io_tensor(req, resp, state, &ai_output[i],
        ctx->outputs.address[i], flags, 0.0, 0, false);
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

