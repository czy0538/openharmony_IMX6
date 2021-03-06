# Copyright (c) 2013-2019, Huawei Technologies Co., Ltd. All rights reserved.
# Copyright (c) 2020, Huawei Device Co., Ltd. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of
#    conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list
#    of conditions and the following disclaimer in the documentation and/or other materials
#    provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used
#    to endorse or promote products derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

############################# SRCs #################################
HWI_SRC     :=
MMU_SRC     :=
NET_SRC     :=
TIMER_SRC   :=
HRTIMER_SRC :=
UART_SRC    :=
USB_SRC     :=

############################# HI3516DV300 Options#################################
ifeq ($(LOSCFG_PLATFORM_HI3516DV300), y)
    HWI_TYPE     := arm/interrupt/gic
    TIMER_TYPE   := arm/timer/arm_generic
    HRTIMER_TYPE := hisoc/hrtimer
    NET_TYPE     := hieth
    UART_TYPE    := amba_pl011
    USB_TYPE     := usb3.0_hi3516dv300
    LITEOS_CMACRO_TEST += -DTEST3516DV300

########################## HI3518EV300 Options##############################
else ifeq ($(LOSCFG_PLATFORM_HI3518EV300), y)
    HWI_TYPE     := arm/interrupt/gic
    TIMER_TYPE   := hisoc/timer
    HRTIMER_TYPE := hisoc/hrtimer
    NET_TYPE     := hieth
    UART_TYPE    := amba_pl011
    USB_TYPE     := usb3.0_hi3518ev300
    LITEOS_CMACRO_TEST += -DTEST3518EV300
else ifeq ($(LOSCFG_PLATFORM_IMX6ULL), y)
    HWI_TYPE     := arm/interrupt/gic
    TIMER_TYPE   := arm/timer/arm_generic
    HRTIMER_TYPE := imx6ull/hrtimer
else ifeq ($(LOSCFG_PLATFORM_STM32MP157), y)
    HWI_TYPE     := arm/interrupt/gic
    TIMER_TYPE   := arm/timer/arm_generic
    HRTIMER_TYPE := stm32mp157/hrtimer
else ifeq ($(LOSCFG_PLATFORM_DEMOCHIP), y)
    HWI_TYPE     := arm/interrupt/gic
    TIMER_TYPE   := arm/timer/arm_generic
    HRTIMER_TYPE := demochip/hrtimer
endif

HWI_SRC     := hw/$(HWI_TYPE)
TIMER_SRC   := hw/$(TIMER_TYPE)
HRTIMER_SRC := hw/$(HRTIMER_TYPE)
NET_SRC     := net/$(NET_TYPE)
UART_SRC    := uart/$(UART_TYPE)
USB_SRC     := usb/$(USB_TYPE)

LITEOS_BASELIB       += -lbsp

LITEOS_PLATFORM      := $(subst $\",,$(LOSCFG_PLATFORM))

PLATFORM_BSP_HISI_BASE := $(LITEOSTOPDIR)/platform

PLATFORM_INCLUDE := -I $(LITEOSTOPDIR)/../../vendor/hisi/hi35xx/$(LITEOS_PLATFORM)/config/board/include \
                    -I $(PLATFORM_BSP_HISI_BASE)/../kernel/common \
                    -I $(PLATFORM_BSP_HISI_BASE)/../../../drivers/liteos/platform/pm \
                    -I $(PLATFORM_BSP_HISI_BASE)/hw/include \
                    -I $(PLATFORM_BSP_HISI_BASE)/include \
                    -I $(PLATFORM_BSP_HISI_BASE)/$(UART_SRC)

ifeq ($(findstring y, $(LOSCFG_PLATFORM_HI3518EV300)$(LOSCFG_PLATFORM_HI3516DV300)), y)
    PLATFORM_INCLUDE += -I $(LITEOSTOPDIR)/../../vendor/hisi/hi35xx/$(LITEOS_PLATFORM)/config/board/include/hisoc
else ifeq ($(LOSCFG_PLATFORM_IMX6ULL),y)
    PLATFORM_INCLUDE += -I $(LITEOSTOPDIR)/../../vendor/nxp/imx6ull/board/include
else ifeq ($(LOSCFG_PLATFORM_STM32MP157),y)
    PLATFORM_INCLUDE += -I $(LITEOSTOPDIR)/../../vendor/st/stm32mp157/board/include
else ifeq ($(LOSCFG_PLATFORM_DEMOCHIP),y)
    PLATFORM_INCLUDE += -I $(LITEOSTOPDIR)/../../vendor/democom/demochip/board/include
endif

#
#-include $(LITEOSTOPDIR)/platform/bsp/board/$(LITEOS_PLATFORM)/board.mk
#

LIB_SUBDIRS             += $(PLATFORM_BSP_HISI_BASE)
LITEOS_PLATFORM_INCLUDE += $(PLATFORM_INCLUDE)
LITEOS_CXXINCLUDE       += $(PLATFORM_INCLUDE)
