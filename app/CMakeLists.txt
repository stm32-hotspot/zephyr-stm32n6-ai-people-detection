##############################################################################
# @file    CMakeLists.txt
# @author  GPM Application Team
#
##############################################################################
# @attention
#
# Copyright (c) 2025 STMicroelectronics.
# All rights reserved.
#
# This software is licensed under terms that can be found in the LICENSE file
# in the root directory of this software component.
# If no LICENSE file comes with this software, it is provided AS-IS.
#
##############################################################################

cmake_minimum_required(VERSION 3.20.0)

set(BOARD stm32n6570_dk)
set(SHIELD st_b_cams_imx_mb1854)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

target_include_directories(app PRIVATE inc Model)
target_sources(app PRIVATE src/main.c src/model.c Model/network.c Model/stai_network.c src/zephyr_logo.c)

# lib_vision_models_pp
target_include_directories(app PRIVATE ../Lib/lib_vision_models_pp/lib_vision_models_pp/Inc
                           ../Lib/lib_vision_models_pp/Drivers/CMSIS/DSP/Include)

target_sources(app PRIVATE ../Lib/lib_vision_models_pp/lib_vision_models_pp/Src/vision_models_pp.c
                           ../Lib/lib_vision_models_pp/lib_vision_models_pp/Src/vision_models_pp_maxi_if32.c
                           ../Lib/lib_vision_models_pp/lib_vision_models_pp/Src/vision_models_pp_maxi_is8.c
                           ../Lib/lib_vision_models_pp/lib_vision_models_pp/Src/vision_models_pp_maxi_iu8.c
                           ../Lib/lib_vision_models_pp/lib_vision_models_pp/Src/od_pp_yolov2.c
                           ../Lib/lib_vision_models_pp/lib_vision_models_pp/Src/od_pp_yolov5.c
                           ../Lib/lib_vision_models_pp/lib_vision_models_pp/Src/od_pp_yolov8.c
                           ../Lib/lib_vision_models_pp/lib_vision_models_pp/Src/od_pp_st_yolox.c)

# AI_Runtime
target_compile_definitions(app PRIVATE LL_ATON_PLATFORM=LL_ATON_PLAT_STM32N6
                           LL_ATON_OSAL=LL_ATON_OSAL_ZEPHYR
                           LL_ATON_RT_MODE=LL_ATON_RT_ASYNC
                           LL_ATON_SW_FALLBACK
                           LL_ATON_DBG_BUFFER_INFO_EXCLUDED=1
                           APP_HAS_PARALLEL_NETWORKS=0)

target_include_directories(app PRIVATE ../Lib/AI_Runtime/Inc
                           ../Lib/AI_Runtime/Npu/ll_aton
                           ../Lib/AI_Runtime/Npu/Devices/STM32N6xx)

target_sources(app PRIVATE ../Lib/AI_Runtime/Npu/ll_aton/ll_aton.c
               ../Lib/AI_Runtime/Npu/ll_aton/ll_aton_osal_zephyr.c
               ../Lib/AI_Runtime/Npu/ll_aton/ll_aton_debug.c
               ../Lib/AI_Runtime/Npu/ll_aton/ll_aton_lib.c
               ../Lib/AI_Runtime/Npu/ll_aton/ll_aton_lib_sw_operators.c
               ../Lib/AI_Runtime/Npu/ll_aton/ll_aton_rt_main.c
               ../Lib/AI_Runtime/Npu/ll_aton/ll_aton_runtime.c
               ../Lib/AI_Runtime/Npu/ll_aton/ll_aton_cipher.c
               ../Lib/AI_Runtime/Npu/ll_aton/ll_aton_stai_internal.c
               ../Lib/AI_Runtime/Npu/ll_aton/ll_aton_util.c
               ../Lib/AI_Runtime/Npu/ll_aton/ll_sw_float.c
               ../Lib/AI_Runtime/Npu/ll_aton/ll_sw_integer.c
               ../Lib/AI_Runtime/Npu/ll_aton/ecloader.c
               ../Lib/AI_Runtime/Npu/Devices/STM32N6xx/npu_cache.c
               ../Lib/AI_Runtime/Npu/Devices/STM32N6xx/mcu_cache.c)

target_link_directories(app PUBLIC ../Lib/AI_Runtime/Lib/GCC/ARMCortexM55)

target_link_libraries(app PRIVATE :NetworkRuntime1210_CM55_GCC.a)
