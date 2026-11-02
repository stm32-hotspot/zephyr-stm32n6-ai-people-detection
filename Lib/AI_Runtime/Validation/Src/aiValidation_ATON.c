/**
 ******************************************************************************
 * @file    aiValidation_ATON.c
 * @author  STMicroelectronics
 * @brief   AI Validation application (entry points) - ATON/NPU runtime
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

/* Description
 *
 * - Entry points for the AI validation, ATON/NPU runtime
 *
 * History:
 *  - v0.0 - Initial INTERNAL version for ATON runtime (based on legacy aiValidation 3.1)
 *          USE_CORE_CLOCK_ONLY is forced to 1
 *          hardcoded UART fcts are used (HAL not yest available): uart.c/uart.h
 *          based on the David Siorpaes's project
 *             (ssh://gitolite@codex.cro.st.com/orlando2assp/stice4-python.git)
 *  - v0.1 - add new shape type to consider ONNX format
 *  - v0.2 - clean the code
 *  - v0.3 - fix reported IO shape
 *  - v0.4 - add the dummp of profiling metrics (based on NPU counters) to the host from s-msg + dynamic configuration.
 *  - v0.5 - add log to show the installation/init times
 *  - v0.6 - fix PB_LC_STAT fct with USBC stack
 *  - v0.7 - add overlay support for multiple models (reloc mode)
 *
 *  TODO
 *    - move list of cdt ll_buffer to wrapper code
 *    - add/complete NPU error management and report
 */

/*
 * Note about the xx_PRINT/STAT macros
 *
 *      LC_PRINT mesgs are only routed to the debug print link. Not managed by the PB protocol.
 *        These msgs can be not emitted inside a PB msg (aiPbCmdXXX fct), they are discarded
 *        if no DEDICATED_PRINT_PORT is available.
 *      PB_LC_PRINT & PB_LC_STAT msgs are only managed by the PB protocol and should be
 *        emitted inside a PB msg (aiPbCmdXXX fct).
 *
 */

/* System headers */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "ll_aton_runtime.h"
#include "ll_aton_version.h"
#include "ll_aton_dbgtrc.h"

#include "ai_io_buffers_ATON.h"
#include "stai.h"

#ifndef HAS_RW_MEMORY
// #define HAS_RW_MEMORY
#endif

#define USE_OBSERVER         1  /* 0: remove the registration of the user CB to evaluate the inference time by layer */

#define USE_CORE_CLOCK_ONLY  1  /* 1: remove usage of the HAL_GetTick() to evaluate the number of CPU clock. Only the Core
                                 *    DWT IP is used. HAL_Tick() is requested to avoid an overflow with the DWT clock counter
                                 *    (32b register) - USE_SYSTICK_ONLY should be set to 0.
                                 */

#define USE_SYSTICK_ONLY     0  /* 1: use only the SysTick to evaluate the time-stamps (for Cortex-m0 based device, this define is forced) */


#if defined(USE_OBSERVER) && USE_OBSERVER == 1
#define HAS_OBSERVER
#endif

/* APP Header files */
#include "aiValidation.h"

#include <aiTestUtility.h>
#include <aiTestHelper.h>

#include <aiPbMgr.h>
#include <aiPbMemRWServices.h>
#include <stm32msg.pb.h>

#include "ai_wrapper_ATON.h"
#include "ll_aton_caches_interface.h"

#define _AI_RUNTIME_ID EnumAiRuntime_AI_RT_ATONN

#if defined(HAS_RW_MEMORY) && defined(HAS_OBSERVER)
#define _CAP (void *)(EnumCapability_CAP_READ_WRITE | EnumCapability_CAP_OBSERVER | (_AI_RUNTIME_ID << 16))
#elif defined(HAS_OBSERVER)
#define _CAP (void *)(EnumCapability_CAP_OBSERVER | (_AI_RUNTIME_ID << 16))
#elif defined(HAS_RW_MEMORY)
#define _CAP (void *)(EnumCapability_CAP_READ_WRITE | (_AI_RUNTIME_ID << 16))
#else
#define _CAP (void *)(_AI_RUNTIME_ID << 16)
#endif

/*
 * List of the named tensors which can be dumped
 */
static const char *_dumpable_tensor_name[] = {

    NULL
};


/* -----------------------------------------------------------------------------
 * TEST-related definitions
 * -----------------------------------------------------------------------------
 */
#define _APP_VERSION_MAJOR_  (0x00)
#define _APP_VERSION_MINOR_  (0x06)
#define _APP_VERSION_        ((_APP_VERSION_MAJOR_ << 8) | _APP_VERSION_MINOR_)

#define _APP_NAME_           "AI Validation ATONN/NPU"


struct aton_context {
  struct npu_instance instance;

  const reqMsg *creq;             /* reference of the current PB request */
  respMsg *cresp;                 /* reference of the current PB response */
  int error;

  bool observer_is_enabled;       /* indicate if the observer is enabled */
  bool emit_intermediate_data;    /* indicate that the data from the intermediate tensors can be dumped/uploaded */
  bool simple_value;              /* indicate that only the first value has been provided and should be broadcasted
                                     to the whole input tensor */
  bool debug;

  int16_t cur_epoch_num;
};

/* -----------------------------------------------------------------------------
 * PB_LC_PRINT/STAT functions
 * -----------------------------------------------------------------------------
 */

#if defined(HAS_DEDICATED_PRINT_PORT) && HAS_DEDICATED_PRINT_PORT == 1
#define PB_LC_PRINT(debug, fmt, ...) LC_PRINT(fmt, ##__VA_ARGS__)
#define PB_LC_STAT(_cat, _sub_cat, fmt, ...) LC_PRINT("s:" _cat ":" _sub_cat ":" fmt, ##__VA_ARGS__)
#else

#define _PRINT_BUFFER_SIZE  160

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
 * Protobuf IO port adaptations
 * -----------------------------------------------------------------------------
 */

struct _data_tensor_desc {
  struct npu_model_info* nn;
  uint32_t mode;
  uint32_t* size;
  uintptr_t addr;
  uint32_t flags;
};


static bool _is_ll_buffer_valid(const LL_Buffer_InfoTypeDef *buff)
{
  if (!buff)
    return false;

  uint32_t nb_elem = 1;
  for (int i=0; i<buff->ndims; i++)
    nb_elem *= buff->shape[i];

  if ( get_ll_buffer_size(buff) != nb_elem * get_ll_element_size(buff))
    return false;

  return true;
}

static uint32_t set_ai_buffer_format(const LL_Buffer_InfoTypeDef* buff)
{
  bool is_signed = true;        // Signedness of buffer elements if not fixed point
  if (!buff)
    return 0;

  if (buff->type == DataType_FXP) {
    // Signedness is found using Qunsigned field (there are no "signed/unsigned" fixed point types)
    return aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_FXP,
                            buff->Qunsigned?false:true, buff->nbits, buff->Qn);
  }
  else if (buff->type == DataType_FLOAT) {
    return aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_FLOAT,
                            true, buff->nbits, 0);
  }
  else if (buff->type == DataType_BOOL) {
        return aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_BOOL,
                            true, 8, 0);
  }
  else {
    // All other types are considered as integers, only uint types are considered as UNsigned.
    if ((buff->type == DataType_UINT16) | (buff->type == DataType_UINT8) |\
        (buff->type == DataType_UINT32) | (buff->type == DataType_UINT64))
    {
      is_signed = false;
    }
    return aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_INTEGER,
                            is_signed, buff->nbits, 0);
  }
}

static void encode_buffer_to_tensor_desc(size_t index, void* data,
                                         aiTensorDescMsg* msg,
                                         struct _encode_uint32 *array_u32)
{
  struct _data_tensor_desc *info = (struct _data_tensor_desc *)data;

  array_u32->size = 1;
  array_u32->data = (void *)info->size;
  array_u32->offset = 4;

  msg->name[0] = 0;
  msg->format = aiPbTensorFormat(EnumDataFmtType_DATA_FMT_TYPE_INTEGER,
                                 false, 8, 0);
  msg->size = *(info->size);
  msg->n_dims = EnumShapeFmt_F_SHAPE_FMT_UND << EnumShapeFmt_F_SHAPE_FMT_POS | array_u32->size;
  msg->scale = 0.0;
  msg->zeropoint = 0;
  msg->addr = (uint32_t)info->addr;
  msg->flags = info->flags;
}

static void fill_tensor_desc_msg(const LL_Buffer_InfoTypeDef* buff,
                                 aiTensorDescMsg* msg,
                                 uint32_t flags,
                                 struct _encode_uint32 *array_u32)
{
  array_u32->size = buff->mem_ndims;
  array_u32->data = (void *)buff->mem_shape;
  array_u32->offset = 4;

  aiPbStrCopy(buff->name, &msg->name[0], sizeof(msg->name));
  msg->format = set_ai_buffer_format(buff);
  msg->flags = flags;

  msg->size = 1;
  for (int i=0; i<buff->mem_ndims; i++)
    msg->size *= buff->mem_shape[i];

  // Explanations on buff->batch -- subject to change, be careful.
  // - Batch 1 means in memory channel first
  // - Batch == nchannels means in memory channel last
  // For rank = 4: onnx shape {A,B,C,D} -- A,B,C,D -> you see in the generated code A,C,D,B
  // For now, BCHW is forced for atonn RT, as the pb_mgr_drv interprets the data correctly sending this shape fmt.
  //          EnumShapeFmt_F_SHAPE_FMT_BHWC is not used.
  // @TODO clean this
  msg->n_dims = EnumShapeFmt_F_SHAPE_FMT_UND << EnumShapeFmt_F_SHAPE_FMT_POS | array_u32->size;

  if (buff->scale) {
    msg->scale = buff->scale[0];
    msg->zeropoint = buff->offset[0];
  } else {
    msg->scale = 0.0;
    msg->zeropoint = 0;
  }

  msg->addr = (uint32_t)LL_Buffer_addr_start(buff);
}


static void encode_ll_buffer_to_tensor_desc(size_t index, void* data,
                                            aiTensorDescMsg* msg,
                                            struct _encode_uint32 *array_u32)
{
  struct _data_tensor_desc *info = (struct _data_tensor_desc *)data;
  struct npu_model_info* nn = info->nn;
  const LL_Buffer_InfoTypeDef* buff;
  if (info->mode == 0)
    buff = nn->in_bufs[index];
  else  // if (info->mode == 1)
    buff = nn->out_bufs[index];

  fill_tensor_desc_msg(buff, msg, info->flags, array_u32);
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

static uint32_t _stai_version_to_uint32(const stai_version *version)
{
  return version->major << 24 | version->minor << 16 | version->micro << 8 | version->reserved;
}

/*
 * Fill and send a "aiModelInfoMsg" msg
 */
static void send_model_info(const reqMsg *req, respMsg *resp,
                            EnumState state, struct aton_context* ctx)
{
  uint32_t flags;
  struct npu_model_info *info = &ctx->instance.info;
  resp->which_payload = respMsg_minfo_tag;

  stai_runtime_info rt_info;
  stai_runtime_get_info(&rt_info);

  memset(&resp->payload.minfo, 0, sizeof(aiModelInfoMsg));

  aiPbStrCopy(info->name, &resp->payload.minfo.name[0],
      sizeof(resp->payload.minfo.name));

  resp->payload.minfo.rtid = _AI_RUNTIME_ID | (_stai_compiler_id_to(rt_info.compiler_id) << EnumTools_AI_TOOLS_POS);

  if (info->flags & NPU_MODEL_INFO_FLAGS_RELOC)
    resp->payload.minfo.rtid |= (EnumAiApiRuntime_AI_RT_API_RELOC << EnumAiApiRuntime_AI_RT_API_POS);

  if (info->flags & NPU_MODEL_INFO_FLAGS_NO_MACC)
    resp->payload.minfo.rtid |= (4 << 24);

  if (info->flags & NPU_MODEL_INFO_FLAGS_NO_PARAMS)
    resp->payload.minfo.rtid |= (2 << 24);

  if (info->flags & NPU_MODEL_INFO_FLAGS_NO_ACTS)
    resp->payload.minfo.rtid |= (1 << 24);

  aiPbStrCopy(info->compile_datetime, &resp->payload.minfo.compile_datetime[0],
      sizeof(resp->payload.minfo.compile_datetime));

  resp->payload.minfo.runtime_version = _stai_version_to_uint32(&rt_info.runtime_version);
  resp->payload.minfo.tool_version = info->version;

  aiPbStrCopy(info->rt_desc, &resp->payload.minfo.runtime_desc[0],
      sizeof(resp->payload.minfo.runtime_desc));
  uint32_to_str(rt_info.runtime_build, &resp->payload.minfo.runtime_desc[strlen(info->rt_desc)],
                sizeof(resp->payload.minfo.runtime_desc) - strlen(info->rt_desc) - 1);

  resp->payload.minfo.n_macc = 0;
  resp->payload.minfo.n_nodes = info->n_epochs;
  
  resp->payload.minfo.n_init_time = info->init_time;
  resp->payload.minfo.n_install_time = info->install_time;

  flags = EnumTensorFlag_TENSOR_FLAG_INPUT;
  if (!(info->flags & NPU_MODEL_INFO_FLAGS_NO_IN_ALLOC))
   flags |= EnumTensorFlag_TENSOR_FLAG_IN_MEMPOOL;
  struct _data_tensor_desc tensor_desc_ins = {info, 0, 0, 0, flags};
  struct _encode_tensor_desc tensor_ins = {
      &encode_ll_buffer_to_tensor_desc, info->n_inputs, &tensor_desc_ins };
  resp->payload.minfo.n_inputs = info->n_inputs;
  resp->payload.minfo.inputs.funcs.encode = encode_tensor_desc;
  resp->payload.minfo.inputs.arg = (void *)&tensor_ins;

  flags = EnumTensorFlag_TENSOR_FLAG_OUTPUT;
  if (!(info->flags & NPU_MODEL_INFO_FLAGS_NO_OUT_ALLOC))
   flags |= EnumTensorFlag_TENSOR_FLAG_IN_MEMPOOL;
  struct _data_tensor_desc tensor_desc_outs = {info, 1, 0, 0, flags};
  struct _encode_tensor_desc tensor_outs = {
      &encode_ll_buffer_to_tensor_desc, info->n_outputs, &tensor_desc_outs };
  resp->payload.minfo.n_outputs = info->n_outputs;
  resp->payload.minfo.outputs.funcs.encode = encode_tensor_desc;
  resp->payload.minfo.outputs.arg = (void *)&tensor_outs;

  flags = EnumTensorFlag_TENSOR_FLAG_MEMPOOL;
  struct _data_tensor_desc tensor_desc_acts = {info, 2, &info->activations, 0, flags};
  struct _encode_tensor_desc tensor_acts = {
      &encode_buffer_to_tensor_desc, 1, &tensor_desc_acts };
  resp->payload.minfo.n_activations = 1;
  resp->payload.minfo.activations.funcs.encode = encode_tensor_desc;
  resp->payload.minfo.activations.arg = (void *)&tensor_acts;

  struct _data_tensor_desc tensor_desc_params = {info, 3, &info->params, 0, flags};
  struct _encode_tensor_desc tensor_params = {
      &encode_buffer_to_tensor_desc, 1, &tensor_desc_params };
  resp->payload.minfo.n_params = 1;
  resp->payload.minfo.params.funcs.encode = encode_tensor_desc;
  resp->payload.minfo.params.arg = (void *)&tensor_params;

  aiPbMgrSendResp(req, resp, state);
}


static bool receive_ai_io_tensor(const reqMsg *req, respMsg *resp,
    EnumState state, const LL_Buffer_InfoTypeDef *aton_buf, bool simple_value)
{
  bool res = true;
  aiPbData data = {
    0,
    get_ll_buffer_size(aton_buf),
    (uintptr_t)LL_Buffer_addr_start(aton_buf),
    0
  };

 if (simple_value)
    data.size = get_ll_element_size(aton_buf);

  aiPbMgrReceiveData(&data);

  /* Send ACK and wait ACK (or send ACK only if error) */
  if (data.nb_read != data.size) {
    aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
        data.nb_read,
        EnumError_E_INVALID_SIZE);
    res = false;
  }
  else {

    if (simple_value) /* broadcast the value */
    {
      const size_t el_s = data.size;
      const uintptr_t r_ptr = (uintptr_t)LL_Buffer_addr_start(aton_buf);
      uintptr_t w_ptr = r_ptr + el_s;
      for (size_t pos = 1; pos <  get_ll_buffer_size(aton_buf) / el_s; pos++)
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
    EnumState state, const LL_Buffer_InfoTypeDef *aton_buf,
    const uint32_t flags,
    float scale, int32_t zero_point)
{
  struct _encode_uint32 array_u32;

  /* Build the PB message */
  resp->which_payload = respMsg_tensor_tag;

  /*-- Flags field */
  // resp->payload.tensor.flags = flags;

  /*-- Tensor desc field */
  fill_tensor_desc_msg(aton_buf, &resp->payload.tensor.desc, flags, &array_u32);
  resp->payload.tensor.desc.dims.funcs.encode = encode_uint32;
  resp->payload.tensor.desc.dims.arg = &array_u32;

  /*-- Data field */
  resp->payload.tensor.data.addr = (uint32_t)LL_Buffer_addr_start(aton_buf);
  if (flags & EnumTensorFlag_TENSOR_FLAG_NO_DATA) {
    resp->payload.tensor.data.size = 0;
  } else {
    resp->payload.tensor.data.size = get_ll_buffer_size(aton_buf);
  }
  struct aiPbData data = {
    0, resp->payload.tensor.data.size,
    resp->payload.tensor.data.addr, 0
  };
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
 * AI-related functions and execution context
 * -----------------------------------------------------------------------------
 */
#define _MAX_CDT_LL_BUFFERS     (16)

static struct aton_context net_exec_ctx[NPU_NETWORK_NUMBER_MAX];
static struct aton_context *cur_net_exec_ctx = NULL;

static const LL_Buffer_InfoTypeDef* _cdts_buffers[_MAX_CDT_LL_BUFFERS];

static struct aton_context *aiExecCtx(const char *nn_name, int pos)
{
  struct aton_context *cur = NULL;

  if (!nn_name)
    return NULL;

  if (!nn_name[0]) {
    if ((pos >= 0) && (pos < NPU_NETWORK_NUMBER_MAX) && net_exec_ctx[pos].instance.impl)
      cur = &net_exec_ctx[pos];
  } else {
    int idx;
    for (idx=0; idx < NPU_NETWORK_NUMBER_MAX; idx++) {
      cur = &net_exec_ctx[idx];
      if (cur->instance.impl &&
          (strlen(cur->instance.info.name) == strlen(nn_name)) &&
          (strncmp(cur->instance.info.name, nn_name,
              strlen(cur->instance.info.name)) == 0)) {
        pos = idx;
        break;
      }
      cur = NULL;
    }
  }
  if ((cur != NULL) && (cur != cur_net_exec_ctx) &&
      (cur->instance.info.flags & NPU_MODEL_INFO_FLAGS_RT_OVERLAY))
  {
    npu_get_instance_by_index(pos, &net_exec_ctx[pos].instance);
  }
  if (cur)  // prevent the case where ai_runner retrieves the available models
    cur_net_exec_ctx = cur;
  return cur;
}

static void print_ll_io_buffer(const LL_Buffer_InfoTypeDef *aton_buf)
{
  struct _shape_desc shape;

  shape.ndims = aton_buf->mem_ndims; // ndims;
#if 0
  if ((aton_buf->batch > 1) || (aton_buf->ndims < 4)) {
    for (int i=0; i< aton_buf->ndims; i++)
      shape.shape[i] = aton_buf->shape[i];
  }
  else {
    shape.shape[0] = aton_buf->shape[0];
    shape.shape[1] = aton_buf->shape[3];
    shape.shape[2] = aton_buf->shape[1];
    shape.shape[3] = aton_buf->shape[2];
  }
#endif

  for (int i=0; i< aton_buf->mem_ndims; i++)
      shape.shape[i] = aton_buf->mem_shape[i];

  LC_PRINT(" name    : %s\r\n", aton_buf->name);
  LC_PRINT("  addr   : 0x%x (%d bytes)  (%d bits)\r\n",
           (uint32_t)LL_Buffer_addr_start(aton_buf),
           get_ll_buffer_size(aton_buf),
           aton_buf->nbits);
  LC_PRINT("  type   : %d shape(%d)=(", aton_buf->type, shape.ndims);

  for (int i=0; i<shape.ndims; i++)
    if (i == shape.ndims - 1)
      LC_PRINT("%d", shape.shape[i]);
    else
      LC_PRINT("%d,", shape.shape[i]);

  LC_PRINT(")\r\n");

  if (aton_buf->scale) {
    LC_PRINT("  quant  : scale=%f, zp=%d\r\n", aton_buf->scale[0],
             aton_buf->offset[0]);
  }
}

static int aiBootstrap(struct aton_context *ctx)
{
  struct npu_model_info *info = &ctx->instance.info;
  stai_runtime_info netrt_info;

  stai_runtime_get_info(&netrt_info);

  LC_PRINT("\r\n");
  LC_PRINT("ATONN RT\r\n");
  LC_PRINT("--------------------------------------------------\r\n");
  LC_PRINT(" version         : %s\r\n", LL_ATON_VERSION_NAME);
  LC_PRINT(" network rt lib  : v%d.%d.%d-%x\r\n", netrt_info.runtime_version.major,
           netrt_info.runtime_version.minor,
           netrt_info.runtime_version.micro,
           netrt_info.runtime_build);
  LC_PRINT("   compiled with : %s\r\n", netrt_info.compiler_desc);

  LC_PRINT("\r\n");
  LC_PRINT("C-Model\r\n");
  LC_PRINT("--------------------------------------------------\r\n");
  LC_PRINT(" name          : %s\r\n", info->name);
  LC_PRINT(" n_epochs      : %d\r\n", info->n_epochs);
#if !defined(LL_ATON_DBG_BUFFER_INFO_EXCLUDED)
  LC_PRINT(" params        : %d KiB\r\n", (int)(info->params / 1024));
#else
  if (info->params)
      LC_PRINT(" params        : %d KiB\r\n", (int)(info->params / 1024));
  else
      LC_PRINT(" params        : n.a.\r\n");
#endif
  LC_PRINT(" activations   : %d KiB\r\n", (int)(info->activations / 1024));

  LC_PRINT(" n_inputs      : %d\r\n", info->n_inputs);
  for (int idx=0; idx < info->n_inputs; idx++) {
    print_ll_io_buffer(info->in_bufs[idx]);
  }

  LC_PRINT(" n_outputs     : %d\r\n", info->n_outputs);
  for (int idx=0; idx < info->n_outputs; idx++) {
    print_ll_io_buffer(info->out_bufs[idx]);
  }

  LC_PRINT("\r\n");
  int res = npu_init(&ctx->instance, 1);
  LC_PRINT(" NPU stack initialization (res=%d)\r\n", res);

  LC_PRINT(" \r\n");
  LC_PRINT(" Model installation %d MCU cycles (%f ms)\r\n", ctx->instance.install_cycles,
           dwtCyclesToFloatMs(ctx->instance.install_cycles));
  LC_PRINT(" Model init         %d MCU cycles (%f ms)\r\n", ctx->instance.init_cycles,
           dwtCyclesToFloatMs(ctx->instance.init_cycles));

  return 0;
}

static bool _buffer_is_filtered(struct aton_context *ctx,
                                const LL_Buffer_InfoTypeDef *buff)
{
  if (ctx->emit_intermediate_data == false) // all data are filtered
    return true;

  for (int idx = 0; _dumpable_tensor_name[idx] != NULL; idx++) {
    if (strcmp(buff->name, _dumpable_tensor_name[idx]) == 0) {
        return false;
    }
  }

  return true;
}

int16_t _find_cdt_ll_buffers(struct aton_context *ctx, const int16_t epoch_num,
                             const LL_ATON_RT_EpochBlockItem_t *epoch_block)
{
  int16_t n_cdts_buffers;

  const LL_Buffer_InfoTypeDef *internals = npu_get_internal_buffers_info(&ctx->instance);
  const LL_Buffer_InfoTypeDef *outputs = npu_get_output_buffers_info(&ctx->instance, -1);

  const LL_Buffer_InfoTypeDef *aton_buf;

  int16_t extra_epoch_num = 0;
  if ((epoch_block->epoch_num > 0) && (epoch_num > 0) &&
      ((epoch_block + 1)->epoch_num != epoch_num + 1))
  { /* no resources allocated for the next operation */
    extra_epoch_num = epoch_num + 1;
  }

  if ((epoch_block->epoch_num > 0) &&
      ((epoch_block + 1)->epoch_num == epoch_block->epoch_num))
  { /* next epoch will be used  */
    return 0;
  }

  n_cdts_buffers = 0;
  for (aton_buf = internals; aton_buf && aton_buf->name != NULL; aton_buf++) {
    if ((aton_buf->epoch == epoch_num) ||
        (aton_buf->epoch == extra_epoch_num))
    {
      if ( _is_ll_buffer_valid(aton_buf) && (n_cdts_buffers < _MAX_CDT_LL_BUFFERS))
      _cdts_buffers[n_cdts_buffers++] = aton_buf;
    }
  }

  for (aton_buf = outputs; aton_buf->name != NULL; aton_buf++) {
    if (aton_buf->epoch == epoch_num)
    {
      if ( _is_ll_buffer_valid(aton_buf) && (n_cdts_buffers < _MAX_CDT_LL_BUFFERS))
      _cdts_buffers[n_cdts_buffers++] = aton_buf;
    }
  }
  return n_cdts_buffers;
}

void _log_counters(struct aton_context *ctx, const LL_ATON_RT_EpochBlockItem_t *epoch_block,
                   struct npu_epoch_counters *counters)
{
  PB_LC_STAT("node", "mcu_cycles", "%d:%d:%d", counters->cpu_start,
             counters->cpu_core, counters->cpu_end);

  if (counters->counter_fmt & COUNTER_FMT_OPT(COUNTER_OPT_EPOCH_LEN)) {
    PB_LC_STAT("node", "npu_cycles", "%d:%d:%d", counters->npu_start,
               counters->npu_core, counters->npu_end);
  }

  if (counters->counter_fmt & (COUNTER_FMT_OPT(COUNTER_OPT_STRG_I_ACTIVE | COUNTER_OPT_STRG_O_ACTIVE))) {
    const int counter_n = COUNTER_FMT_NUMBER(counters->counter_fmt);
    int argmax = -1;
    uint32_t maxcount = 0;
    uint32_t count = 0;
    char type = (counters->counter_fmt & COUNTER_FMT_OPT(COUNTER_OPT_STRG_I_ACTIVE))?'i':'o';
    for (int i = 0; i < ATON_STRENG_NUM && count < counter_n; i++) {
      if (counters->counters[count] > 20) {
        int diff = (int)counters->npu_core - (int)counters->counters[count];
        if (counters->counters[count] > maxcount) {
          maxcount = counters->counters[count];
          argmax = i;
        }
        PB_LC_STAT("node", "streng_active", "%c:%u:%u:%d", type, i, counters->counters[count], diff);
      }
      count++;
    }
    PB_LC_STAT("node", "streng_active", "max:%c:%d:%u", type, argmax, maxcount);
  }

  if (counters->counter_fmt & COUNTER_FMT_OPT(COUNTER_OPT_STRG_ACTIVE)) {
    const int counter_n = COUNTER_FMT_NUMBER(counters->counter_fmt);
    uint32_t maxcount = 0;
    int argmax = -1;
    char type = 'i';
    uint32_t count = 0;
    for (int i = 0; i < ATON_STRENG_NUM && count < counter_n; i++) {
      if (epoch_block->in_streng_mask & (1 << i)) {
        int diff = (int)counters->npu_core - (int)counters->counters[count];
        if (counters->counters[count] > maxcount) {
          maxcount = counters->counters[count];
          argmax = i;
          type = 'i';
        }
        PB_LC_STAT("node", "streng_active", "i:%u:%u:%d", i, counters->counters[count], diff);
        count++;
      }
      if (epoch_block->out_streng_mask & (1 << i)) {
        int diff = (int)counters->npu_core - (int)counters->counters[count];
        if (counters->counters[count] > maxcount) {
          maxcount = counters->counters[count];
          argmax = i;
          type = 'o';
        }
        PB_LC_STAT("node", "streng_active", "o:%u:%u:%d", i, counters->counters[count], diff);
        count++;
      }
    }
    PB_LC_STAT("node", "streng_active", "max:%c:%d:%u", type, argmax, maxcount);
  }

  if (counters->counter_fmt & COUNTER_FMT_OPT(COUNTER_OPT_STRG_HENV)) {
    const int counter_n = COUNTER_FMT_NUMBER(counters->counter_fmt);
    uint32_t count = 0;
    for (int i = 0; i < ATON_STRENG_NUM && count < counter_n; i++) {
      if (epoch_block->in_streng_mask & (1 << i)) {
        PB_LC_STAT("node", "streng_henv", "i:%u:%u:%d", i, counters->counters[count],
                   (int)(counters->npu_core - counters->counters[count] ));
        count++;
      }
    }
  }

  if (counters->counter_fmt & COUNTER_FMT_OPT(COUNTER_OPT_BUSIF_RW_DATA)) {
    const int counter_n = COUNTER_FMT_NUMBER(counters->counter_fmt);
    int count_idx = 0;
    do
    {
      if (count_idx < 4) {
        PB_LC_STAT("node", "port0", "burst:w:%d:%d:%d:%d",
                   counters->counters[count_idx + 0],
                   counters->counters[count_idx + 1],
                   counters->counters[count_idx + 2],
                   counters->counters[count_idx + 3]);
      }
      else if (count_idx < 8) {
        PB_LC_STAT("node", "port0", "burst:r:%d:%d:%d:%d",
                   counters->counters[count_idx + 0],
                   counters->counters[count_idx + 1],
                   counters->counters[count_idx + 2],
                   counters->counters[count_idx + 3]);
      }
      else if (count_idx < 12) {
        PB_LC_STAT("node", "port1", "burst:w:%d:%d:%d:%d",
                   counters->counters[count_idx + 0],
                   counters->counters[count_idx + 1],
                   counters->counters[count_idx + 2],
                   counters->counters[count_idx + 3]);
      }
      else {
        PB_LC_STAT("node", "port1", "burst:r:%d:%d:%d:%d",
                   counters->counters[count_idx + 0],
                   counters->counters[count_idx + 1],
                   counters->counters[count_idx + 2],
                   counters->counters[count_idx + 3]);
      }
      count_idx += 4;
    }
    while (count_idx < counter_n);

    unsigned int totalWrites;
    unsigned int totalReads;
    LL_Dbgtrc_GetTotalTranfers(0,  &totalWrites, &totalReads);
    PB_LC_STAT("node", "portx", "burst:rw:%u:%u:%u:0", totalReads, totalWrites, totalReads + totalWrites);
  }

  if (counters->counter_fmt & COUNTER_FMT_OPT(COUNTER_OPT_NPU_CACHE)) {
    // r-hit, r-miss, r-alloc-miss, evict
    PB_LC_STAT("node", "npu_cache", "r:%d:%d:%d:%d",
               counters->cache_counters[0],
               counters->cache_counters[1],
               counters->cache_counters[2],
               counters->cache_counters[3]);
    // w-hit, w-miss, w-alloc-miss, w-through
    PB_LC_STAT("node", "npu_cache", "w:%d:%d:%d:%d",
               counters->cache_counters[4],
               counters->cache_counters[5],
               counters->cache_counters[6],
               counters->cache_counters[7]);
  }
}

static uint32_t _get_node_type(const LL_ATON_RT_EpochBlockItem_t *epoch_block,
                               int16_t n_cdts_buffers, char** desc)
{
  /*
   *       type (32b) - b31..b24 : bit-mask - OPERATOR_FLAG_XX
   *                    b23..b16 : reserved
   *                    b15..b4  : sub-node type
   *                    b3..b0   : node type - 0000b: HW
   *                                           0001b: extra HW
   *                                           0010b: SW
   *                                           0011b: mix HW/SW - HYBRID
   *                                           0100b: epoch controller
   *
   * Note that a SW epoch can be also a simple epoch fct to schedule other HW epoches
   *       like LL_LIB_ATON_Concat()/LL_LIB_ATON_Pad() fcts
   */

  uint32_t type;
  static const char HW_EPOCH[] = "HW";
  static const char SW_EPOCH[] = "SW";
  static const char EC_EPOCH[] = "EC";
  static const char HYBRID_EPOCH[] = "HYBRID";
  static const char EXTRA_EPOCH[] = "EXTRA";

  char* epoch_str = (char *)HW_EPOCH;

    /* Default - HW epoch */
  type = (EnumOperatorFlag_OPERATOR_FLAG_INTERNAL << EnumOperatorFlag_OPERATOR_FLAG_POS);

  if (n_cdts_buffers == 0)
    type |= (EnumOperatorFlag_OPERATOR_FLAG_WITHOUT_TENSOR << EnumOperatorFlag_OPERATOR_FLAG_POS);

  if (EpochBlock_IsEpochBlob(epoch_block) == true)
  {
    type |= 4; /* epoch controller, blob */
    epoch_str = (char *)EC_EPOCH;
  }
  else if(EpochBlock_IsEpochPureSW(epoch_block) == true)
  {
    type |= 2; /* SW epoch */
    epoch_str = (char *)SW_EPOCH;
  }
  else if (EpochBlock_IsEpochHybrid(epoch_block) == true)
  {
    type |= 3;
    epoch_str = (char *)HYBRID_EPOCH;
  }
  else if (EpochBlock_IsEpochInternal(epoch_block) == true)
  {
    uint32_t sub_type = ((-epoch_block->epoch_num) & 0x3F) << 4;
    sub_type |= 1;  /* extra epoch */
    type |= sub_type;
    epoch_str = (char *)EXTRA_EPOCH;
  }

  *desc = epoch_str;
  return type;
}

void npu_dump_tensors_cb(const LL_ATON_RT_Callbacktype_t ctype, const int16_t cidx,
                         const LL_ATON_RT_EpochBlockItem_t *epoch_block,
                         struct npu_epoch_counters *counters)
{
  int16_t n_cdts_buffers = 0;

  struct aton_context *ctx = cur_net_exec_ctx;
  uint32_t type;
  int16_t buff_epoch = -1;
  char *epoch_desc;

  if (epoch_block->epoch_num >= 0) {
    ctx->cur_epoch_num =  epoch_block->epoch_num;
  }
  buff_epoch = ctx->cur_epoch_num;

  if (ctype == LL_ATON_RT_Callbacktype_PRE_START) {
    type = _get_node_type(epoch_block, 0, &epoch_desc);

    if (ctx->debug) {
      PB_LC_PRINT(ctx->debug, "cidx=%d: EpochBlock_%d (%d), type=%08X %s..",
              cidx, epoch_block->epoch_num, epoch_block->last_epoch_num,
              type, epoch_desc);
    }
    PB_LC_STAT("node", "evt_pre_start", "%d:%d:%d:%d:%08X:%08X:%s", cidx,
               ctx->cur_epoch_num, epoch_block->epoch_num,
               epoch_block->last_epoch_num, epoch_block->flags,
               type, epoch_desc);
    return;
  }

  if (ctype != LL_ATON_RT_Callbacktype_POST_END) {
    return;
  }

  uint64_t cycles = counters->cpu_start + counters->cpu_core + counters->cpu_end;

  /* 4 - Send basic report (optional) ------------------------------ */
  aiOpPerf perf = {
    dwtCyclesToFloatMs(cycles),
    EnumCounterFormat_COUNTER_FMT_32B << EnumCounterFormat_COUNTER_FMT_POS | EnumCounterType_COUNTER_TYPE_CPU,
    3,  // 2 * 3,
    (uint32_t *)&counters->cpu_start, -1, -1
  };

  /* calculate number of associated tensors */
  n_cdts_buffers = _find_cdt_ll_buffers(ctx, buff_epoch, epoch_block);

  type = _get_node_type(epoch_block, n_cdts_buffers, &epoch_desc);

  PB_LC_PRINT(ctx->debug, "cidx=%d: EpochBlock_%d (%d), (cur=%d, nb_buffer=%d), type=%08X %s",
              cidx, epoch_block->epoch_num,
              epoch_block->last_epoch_num, ctx->cur_epoch_num, n_cdts_buffers, type, epoch_desc);

  PB_LC_STAT("node", "evt_post_end", "%d:%d:%d:%d:%d", cidx,
               ctx->cur_epoch_num, epoch_block->epoch_num,
               epoch_block->last_epoch_num, n_cdts_buffers);

  _log_counters(ctx, epoch_block, counters);

  /*
   * Send the op descriptor
   *
   *       name         NULL
   *       type (32b)
   *       id  (32b)  - b31..b16 : last_epoch_num
   *                    b15..b0  : epoch_num
   */

  aiPbMgrSendOperator(ctx->creq, ctx->cresp, EnumState_S_PROCESSING,
                      NULL, type, epoch_block->last_epoch_num << 16 | ctx->cur_epoch_num, &perf);


  /*
   * Send the tensor descriptor
   */

  /* no buffer are emitted */
  if (n_cdts_buffers == 0) {
    return;
  }

  for (int pos = 0; pos < n_cdts_buffers; pos++) {

    uint32_t tens_flags = EnumTensorFlag_TENSOR_FLAG_INTERNAL;
    const LL_Buffer_InfoTypeDef *aton_buf;

    aton_buf = _cdts_buffers[pos];

    if (_buffer_is_filtered(ctx, aton_buf))
        tens_flags |= EnumTensorFlag_TENSOR_FLAG_NO_DATA;
    else
        LL_ATON_Cache_MCU_Clean_Invalidate_Range((uint32_t)LL_Buffer_addr_start(aton_buf), (uint32_t)LL_Buffer_addr_end(aton_buf));

    if (pos == n_cdts_buffers - 1)
        tens_flags |= EnumTensorFlag_TENSOR_FLAG_LAST;

    PB_LC_PRINT(ctx->debug, "TENSOR: %d.%d, epoch=%d, %d:%d bytes (flags=%d)",
                  epoch_block->epoch_num, epoch_block->last_epoch_num,
                  aton_buf->epoch, pos, get_ll_buffer_size(aton_buf),
                  tens_flags);

    send_ai_io_tensor(ctx->creq, ctx->cresp, EnumState_S_PROCESSING,
                        aton_buf, tens_flags, 0.0, 0);
  }
}


static void aiDone(struct aton_context *ctx)
{
  LC_PRINT("Releasing the instance...\r\n");

  if (ctx->instance.impl != NULL) {
    npu_init(&ctx->instance, 0);
    ctx->instance.impl = NULL;
  }
}

static int aiInit(void)
{
  int i=0;
  int n_models = 0;
  while (i < NPU_NETWORK_NUMBER_MAX)
  {
    if (npu_get_instance_by_index(i, &net_exec_ctx[i].instance) == 0) {
      aiBootstrap(&net_exec_ctx[i]);
      cur_net_exec_ctx = &net_exec_ctx[i];
      n_models += 1;
    }
    else {
      break;
    }
    i++;
  }
  if (n_models == 0)
    return -1;
  return 0;
}

static void aiDeInit(void)
{
  for (int i=0; i < NPU_NETWORK_NUMBER_MAX; i++)
    aiDone(&net_exec_ctx[i]);
}


/* -----------------------------------------------------------------------------
 * Specific test APP commands
 * -----------------------------------------------------------------------------
 */

#if defined(USE_USB_CDC_CLASS)
#include <aiPbIO.h>
#endif

void aiPbCmdSysInfo(const reqMsg *req, respMsg *resp, void *param)
{
  UNUSED(param);
  struct mcu_conf conf;
  struct _encode_uint32 array_u32;

  getSysConf(&conf);

  resp->which_payload = respMsg_sinfo_tag;

  uint32_t cache = conf.conf;

  resp->payload.sinfo.devid = conf.devid;
  resp->payload.sinfo.sclock = conf.sclk;
  resp->payload.sinfo.hclock = conf.hclk;
  resp->payload.sinfo.cache = cache;
#if defined(USE_USB_CDC_CLASS)
  uint16_t m_p_size = 0;
  pb_io_get_packet_size(&m_p_size);

  resp->payload.sinfo.com_param = m_p_size;
#else
  resp->payload.sinfo.com_param = 0;  // MIN_PACKET_PAYLOAD_IN_SIZE;
#endif
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

#if defined(USE_USB_CDC_CLASS)
  pb_io_set_packet_size(m_p_size);
  // packet_size = LARGE_PACKET_PAYLOAD_IN_SIZE;
#endif
}


void aiPbCmdNNInfo(const reqMsg *req, respMsg *resp, void *param)
{
  struct aton_context *ctx;
  UNUSED(param);

  ctx = aiExecCtx(req->name, req->param);

  if (ctx) {
    struct mcu_conf conf;
    getSysConf(&conf);
    PB_LC_STAT("config", "version", "1.0");
    PB_LC_STAT("config", "dev_id", "0x%X", conf.devid);
    PB_LC_STAT("config", "mcu_freq", "%d", conf.sclk);
#ifdef STM32H7P
    PB_LC_STAT("config", "axi_freq", "%d", conf.hclk);
#else
    PB_LC_STAT("config", "noc_freq", "%d", conf.hclk);
#endif
    PB_LC_STAT("config", "npu_freq", "%d", conf.extra[1]);
#ifdef STM32H7P
    PB_LC_STAT("config", "ahb_freq", "%d", conf.extra[2]);
#else
    PB_LC_STAT("config", "nic_freq", "%d", conf.extra[2]);
#endif
#ifdef USE_NPU_CACHE
    PB_LC_STAT("config", "npu_cache", "1");
#else
    PB_LC_STAT("config", "npu_cache", "0");
#endif

    send_model_info(req, resp, EnumState_S_IDLE, ctx);
  }
  else
    aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
        EnumError_E_INVALID_PARAM, EnumError_E_INVALID_PARAM);

#if 0
  if (net_exec_ctx[0].instance.itf && req->param == 0)
        send_model_info(req, resp, EnumState_S_IDLE, &net_exec_ctx[0]);
  else {
    if (req->param > 0)
      aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
                EnumError_E_INVALID_PARAM, EnumError_E_INVALID_PARAM);
    else
      aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
          net_exec_ctx[0].error, EnumError_E_GENERIC);
  }
#endif
}


static void _set_context(const reqMsg *req, struct aton_context *ctx)
{
  ctx->cur_epoch_num = -1;

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

  ctx->simple_value = req->param & EnumRunParam_P_RUN_CONF_CONST_VALUE?true:false;
  ctx->debug = req->param & EnumRunParam_P_RUN_CONF_DEBUG?true:false;
}

void aiPbCmdNNRun(const reqMsg *req, respMsg *resp, void *param)
{
  int aton_res;
  uint64_t tend;
  uint32_t tick;
  bool res;
  UNUSED(param);

  struct npu_counters counters;
  struct aton_context *ctx;
  struct npu_model_info *info;

  /* 0 - Check if requested c-name model is available -------------- */
  ctx = aiExecCtx(req->name, -1);
  if (!ctx) {
    aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
        EnumError_E_INVALID_PARAM, EnumError_E_INVALID_PARAM);
    return;
  }

  _set_context(req, ctx);

  ctx->instance.option = req->opt >> 8;

  info = &ctx->instance.info;

  PB_LC_PRINT(ctx->debug, "RUN: rtid=%d\r\n", _AI_RUNTIME_ID);
  PB_LC_PRINT(ctx->debug, "RUN: observer=%d/%d, simple_value=%d\r\n",
      ctx->observer_is_enabled, ctx->emit_intermediate_data,
      ctx->simple_value);

  PB_LC_PRINT(ctx->debug, "RUN: Waiting data (%d bytes).. opt=0x%x, param=0x%x\r\n",
      get_ll_buffer_size(info->in_bufs[0]),
      req->opt, req->param);

  connect_input_buffers(ctx->instance.impl);
  connect_output_buffers(ctx->instance.impl);

  for (int i = 0; i < info->n_inputs; i++) {
    PB_LC_PRINT(ctx->debug, "RUN: I.%d @=0x%x size=%d\r\n", i,
                (uint32_t)LL_Buffer_addr_start(info->in_bufs[i]),
                (int)LL_Buffer_len(info->in_bufs[i]));
  }

  for (int i = 0; i < info->n_outputs; i++) {
    PB_LC_PRINT(ctx->debug, "RUN: O.%d @=0x%x size=%d\r\n", i,
                (uint32_t)LL_Buffer_addr_start(info->out_bufs[i]),
                (int)LL_Buffer_len(info->out_bufs[i]));
  }

  /* 1 - Send a ACK (ready to receive a tensor) -------------------- */
  aiPbMgrSendAck(req, resp, EnumState_S_WAITING,
      get_ll_buffer_size(info->in_bufs[0]), EnumError_E_NONE);

  /* 2 - Receive all input tensors --------------------------------- */
  for (int i = 0; i < info->n_inputs; i++) {
    /* upload a buffer */
    EnumState state = EnumState_S_WAITING;
    if ((i + 1) == info->n_inputs)
      state = EnumState_S_PROCESSING;
    res = receive_ai_io_tensor(req, resp, state, info->in_bufs[i], ctx->simple_value);

    if (res != true)
      return;
  }

  /* 3 - Processing ------------------------------------------------ */

  PB_LC_PRINT(ctx->debug, "RUN: Processing.. tick=%d\r\n", port_hal_get_tick());

  ctx->creq = req;
  ctx->cresp = resp;

  if  (ctx->observer_is_enabled)
    npu_set_callback(&ctx->instance, npu_dump_tensors_cb);

  aton_res = 0;
  tick = port_hal_get_tick();
  npu_run(&ctx->instance, &counters);
  tick = port_hal_get_tick() - tick;
  npu_set_callback(&ctx->instance, NULL);

  if (aton_res != 0) {
    aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
        EnumError_E_GENERIC, EnumError_E_GENERIC);
    return;
  }

  PB_LC_PRINT(ctx->debug, "RUN: Processing done. delta_tick=%d\r\n", tick);

  tend = counters.cpu_all;

  /* 4 - Send basic report (optional) ------------------------------ */
  aiOpPerf perf = {
    dwtCyclesToFloatMs(tend),
    EnumCounterFormat_COUNTER_FMT_64B << EnumCounterFormat_COUNTER_FMT_POS | EnumCounterType_COUNTER_TYPE_CPU,
    2 * 5,
    (uint32_t *)&counters.cpu_start, -1, -1
  };

  aiPbMgrSendOperator(req, resp, EnumState_S_PROCESSING, info->name, 0, 0,
                      &perf);

  /* 5 - Send all output tensors ----------------------------------- */
  PB_LC_PRINT(ctx->debug, "RUN: send output tensors\r\n");
  for (int i = 0; i < info->n_outputs; i++) {
    EnumState state = EnumState_S_PROCESSING;
    uint32_t flags =  EnumTensorFlag_TENSOR_FLAG_OUTPUT;
    if (req->param & EnumRunParam_P_RUN_MODE_PERF) {
      flags |= EnumTensorFlag_TENSOR_FLAG_NO_DATA;
    }
    if ((i + 1) == info->n_outputs) {
      state = EnumState_S_DONE;
      flags |= EnumTensorFlag_TENSOR_FLAG_LAST;
    }
    send_ai_io_tensor(req, resp, state, info->out_bufs[i], flags, 0.0, 0);
  }
}

#if defined(CONF_PERF_MODE_ONLY) && CONF_PERF_MODE_ONLY == 1

#else

static aiPbCmdFunc pbCmdFuncTab[] = {
    AI_PB_CMD_SYNC(_CAP),
    AI_PB_CMD_DISCONNECT(),
    { EnumCmd_CMD_SYS_INFO, &aiPbCmdSysInfo, NULL },
    { EnumCmd_CMD_NETWORK_INFO, &aiPbCmdNNInfo, NULL },
    { EnumCmd_CMD_NETWORK_RUN, &aiPbCmdNNRun, NULL },
#if defined(HAS_RW_MEMORY)
    AI_PB_MEMORY_RW_SERVICES(),
#endif
#if defined(AI_PB_TEST) && AI_PB_TEST == 1
    AI_PB_CMD_TEST(NULL),
#endif
    AI_PB_CMD_END,
};

#endif


/* -----------------------------------------------------------------------------
 * Exported/Public functions
 * -----------------------------------------------------------------------------
 */

int aiValidationInit(void)
{
  LC_PRINT("\r\n#\r\n");
  LC_PRINT("# %s %d.%d\r\n", _APP_NAME_ , _APP_VERSION_MAJOR_, _APP_VERSION_MINOR_);
  LC_PRINT("#\r\n");
  LC_PRINT("\r\n");

  systemSettingLog();

  cyclesCounterInit();

  return 0;
}

int aiValidationProcess(void)
{
  int r;

  r = aiInit();
  if (r) {
    LC_PRINT("aiInit() fails with r=%d\r\n", r);
    return r;
  }

#if defined(CONF_PERF_MODE_ONLY) && CONF_PERF_MODE_ONLY == 1
  int err;
  uint8_t res = 0;

  do {
    LC_PRINT("\r\n");
    err = ai_aton_process(0);

    LC_PRINT("\ntype 'r' to re-start (other to quit)...\n");
    ioRawGetUint8(&res, 5000);

  } while ((err == 0) && (res == 'r'));
  LC_PRINT("bye...\n");

#else

  LC_PRINT("\r\n");
  LC_PRINT("-------------------------------------------\r\n");
  LC_PRINT("| READY to receive a CMD from the HOST... |\r\n");
  LC_PRINT("-------------------------------------------\r\n");
  LC_PRINT("\r\n");
#if defined(USE_USB_CDC_CLASS) && USE_USB_CDC_CLASS == 1
  LC_PRINT("# Note: USB_CDC_CLASS is enabled to transfert the data.\r\n");
#else
  LC_PRINT("# Note: At this point, default ASCII-base terminal should be closed\r\n");
  LC_PRINT("# and a serial COM interface should be used\r\n");
#endif
  LC_PRINT("# (i.e. Python stm32com module). Protocol version = %d.%d\r\n",
      EnumVersion_P_VERSION_MAJOR,
      EnumVersion_P_VERSION_MINOR);

  aiPbMgrInit(pbCmdFuncTab);

  /* used only by Stellar MCUs, empty for all other MCUs */
  port_io_init();

  do {
    r = aiPbMgrWaitAndProcess();
  } while (r==0);
#endif

  return r;
}

void aiValidationDeInit(void)
{
  LC_PRINT("\r\n");
  aiDeInit();
  LC_PRINT("bye bye ...\r\n");
}

