
STM32MP157_BASE_DIR  := $(LITEOSTOPDIR)/../../vendor/st/stm32mp157

LIB_SUBDIRS     += $(STM32MP157_BASE_DIR)/board
LITEOS_BASELIB  += -lboard

LIB_SUBDIRS     += $(STM32MP157_BASE_DIR)/driver/mtd/common
LITEOS_BASELIB  += -lmtd_common

LIB_SUBDIRS     += $(STM32MP157_BASE_DIR)/driver/mtd/spi_nor
LITEOS_BASELIB  += -lspinor_flash

ifeq ($(LOSCFG_DRIVERS_VIDEO), y)
LIB_SUBDIRS             += $(STM32MP157_BASE_DIR)/driver/stm32mp157-fb
LITEOS_BASELIB  += -lstm32mp157-fb
endif

LIB_SUBDIRS             += $(STM32MP157_BASE_DIR)/driver/stm32mp157-uart
LITEOS_BASELIB  += -lstm32mp157-uart


LITEOS_MTD_SPI_NOR_INCLUDE +=   -I$(STM32MP157_BASE_DIR)/driver/mtd/common/include \
                                                                -I$(STM32MP157_BASE_DIR)/driver/mtd/spi_nor/include

