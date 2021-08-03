
IMX6ULL_BASE_DIR  := $(LITEOSTOPDIR)/../../vendor/nxp/imx6ull

LIB_SUBDIRS     += $(IMX6ULL_BASE_DIR)/board
LITEOS_BASELIB  += -lboard

LIB_SUBDIRS     += $(IMX6ULL_BASE_DIR)/driver/mtd/common
LITEOS_BASELIB  += -lmtd_common

LIB_SUBDIRS     += $(IMX6ULL_BASE_DIR)/driver/mtd/spi_nor
LITEOS_BASELIB  += -lspinor_flash

ifeq ($(LOSCFG_DRIVERS_VIDEO), y)
LIB_SUBDIRS             += $(IMX6ULL_BASE_DIR)/driver/imx6ull-fb
LITEOS_BASELIB  += -limx6ull-fb
endif

LIB_SUBDIRS             += $(IMX6ULL_BASE_DIR)/driver/imx6ull-uart
LITEOS_BASELIB  += -limx6ull-uart


LITEOS_MTD_SPI_NOR_INCLUDE +=   -I$(IMX6ULL_BASE_DIR)/driver/mtd/common/include \
                                                                -I$(IMX6ULL_BASE_DIR)/driver/mtd/spi_nor/include

