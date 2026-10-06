# kestrel-air for the Caddx Ascent air unit (HiSilicon Hi3516CV610). It links
# the SDK's MPI libraries directly, so it needs an ARM musl toolchain and a
# copy of the Hi3516CV610 SDK (see README.md):
#   CROSS_COMPILE  the toolchain prefix
#   CV610_SDK_INC  the SDK's root: its kernel/include and libraries/isp/include
#   CV610_SDK_LIB  the directory with the SDK's .so files
# make test and make clean need none of it.
BUILDING := $(if $(MAKECMDGOALS),$(filter-out clean test,$(MAKECMDGOALS)),all)
ifneq ($(BUILDING),)
ifeq ($(CV610_SDK_INC),)
$(error CV610_SDK_INC: set it to the root of a Hi3516CV610 SDK)
endif
ifeq ($(CV610_SDK_LIB),)
$(error CV610_SDK_LIB: set it to the directory with the SDK's libraries)
endif
endif
CC = $(CROSS_COMPILE)gcc

SRC_DIR = src
TARGET = kestrel-air

# The version: git describe, or what the build system passes (KA_VERSION=...).
KA_VERSION ?= $(shell git describe --always --dirty 2>/dev/null || echo unknown)
CFLAGS += -O2 -Wall -Wextra -pthread -D_GNU_SOURCE -I$(SRC_DIR) -Iprotocol \
	-DKA_VERSION='"$(KA_VERSION)"' \
	-I$(CV610_SDK_INC)/kernel/include/hi3516cv6xx \
	-I$(CV610_SDK_INC)/kernel/include/hi3516cv6xx/exp_inc \
	-I$(CV610_SDK_INC)/libraries/isp/include/hi3516cv6xx \
	-I$(CV610_SDK_INC)/libraries/isp/include/hi3516cv6xx/3a \
	-I$(CV610_SDK_INC)/libraries/isp/include/hi3516cv6xx/ext_inc

# The radio goes through ar_libre's client library
# (https://github.com/gehee/ar_libre). ARLIBRE points at its checkout; the
# library is built for the air from $(ARLIBRE)/lib and goes next to kestrel-air.
ifneq ($(BUILDING),)
ifeq ($(ARLIBRE),)
$(error ARLIBRE: set it to an ar_libre checkout)
endif
endif
CFLAGS += -I$(ARLIBRE)/lib
RADIO_LIBS = -L$(ARLIBRE)/lib -lar8030_client

SRCS = $(SRC_DIR)/main.c $(wildcard $(SRC_DIR)/*/*.c)
OBJS = $(SRCS:.c=.o)

# The radio library's directory first: the SDK's has the vendor one too.
LIBS = $(RADIO_LIBS) -L$(CV610_SDK_LIB) -Wl,--allow-shlib-undefined \
	-lss_mpi -lss_mpi_isp -lss_mpi_ae -lss_mpi_awb -lot_mpi_isp \
	-lss_mpi_sysmem -lss_mpi_sysbind -lacs -lbnr -lcalcflicker -ldehaze \
	-ldrc -lextend_stats -lir_auto -lldci -lsecurec -lot_osal -lbin \
	-lm -lpthread -lrt -ldl

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LIBS)

# -MMD -MP: each object also depends on the headers it includes, so a
# changed struct rebuilds everything that uses it.
%.o: %.c
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

-include $(OBJS:.o=.d)

# Host-side tests of what needs no SDK and no radio: the protocol header, the
# ground messages' framing, the checksums, JSON and the packet ring.
HOSTCC ?= cc
TEST_SRCS = $(wildcard tests/*.c) $(SRC_DIR)/ground/frame.c $(SRC_DIR)/common/crc.c \
	$(SRC_DIR)/common/json.c $(SRC_DIR)/video/ring.c $(SRC_DIR)/app/settings.c \
	$(SRC_DIR)/unit/model.c $(SRC_DIR)/video/slices.c
tests/run: $(TEST_SRCS) $(wildcard tests/*.h) $(wildcard protocol/*.h)
	$(HOSTCC) -O1 -g -Wall -Wextra -pthread -I$(SRC_DIR) -Iprotocol -Itests -o $@ $(TEST_SRCS)
test: tests/run
	./tests/run

clean:
	rm -f $(OBJS) $(OBJS:.o=.d) $(TARGET) tests/run

.PHONY: all clean test
