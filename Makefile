# kestrel-air for the Caddx Ascent air unit (HiSilicon Hi3516CV610). It links
# against stub libraries made from stubs/vendor-symbols.txt; the MPI interface
# is in src/sdk/cv610.h. It needs an ARM musl toolchain (see README.md):
#   CROSS_COMPILE  the toolchain prefix
# make test and make clean need none of it.
BUILDING := $(if $(MAKECMDGOALS),$(filter-out clean test check-stubs need-sdk check-sdk-header,$(MAKECMDGOALS)),all)
CC = $(CROSS_COMPILE)gcc

SRC_DIR = src
TARGET = kestrel-air

# The version: git describe, or what the build system passes (KA_VERSION=...).
KA_VERSION ?= $(shell git describe --always --dirty 2>/dev/null || echo unknown)
CFLAGS += -O2 -Wall -Wextra -pthread -D_GNU_SOURCE -I$(SRC_DIR) -Iprotocol \
	-DKA_VERSION='"$(KA_VERSION)"'

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

# The vendor libraries, in the order the unit loads them. Every one is linked
# (--no-as-needed), those kestrel-air calls nothing in too: they do not name
# each other as needed, so the executable has to bring them all in.
VENDOR_LIBS = ss_mpi ss_mpi_isp ss_mpi_ae ss_mpi_awb ot_mpi_isp ss_mpi_sysmem \
	ss_mpi_sysbind acs bnr calcflicker dehaze drc extend_stats ir_auto ldci \
	securec ot_osal bin
STUB_SYMS = stubs/vendor-symbols.txt
STUB_DIR = stubs/out
STUB_SOS = $(patsubst %,$(STUB_DIR)/lib%.so,$(VENDOR_LIBS))

LIBS = $(RADIO_LIBS) -L$(STUB_DIR) -Wl,--no-as-needed $(addprefix -l,$(VENDOR_LIBS)) \
	-Wl,--as-needed -lm -lpthread -lrt -ldl

all: $(TARGET)

$(TARGET): $(OBJS) $(STUB_SOS)
	$(CC) $(CFLAGS) -o $@ $(OBJS) $(LIBS)

# The stub of one library: its SONAME and an empty function for each one
# listed for it.
$(STUB_DIR)/lib%.so: $(STUB_SYMS)
	@mkdir -p $(STUB_DIR)
	awk -v lib=lib$*.so '$$1 == lib { printf "void %s(void) {}\n", $$2 }' $< > $(STUB_DIR)/lib$*.c
	$(CC) -shared -fPIC -nostdlib -Wl,-soname,lib$*.so -o $@ $(STUB_DIR)/lib$*.c

# Every listed function in the libraries (VENDOR_LIB: their directory),
# for when the SDK changes. Needs the toolchain's nm.
check-stubs:
	@test -n "$(VENDOR_LIB)" || { echo "VENDOR_LIB: set it to the directory with the libraries"; exit 1; }
	@bad=0; while read lib sym; do case "$$lib" in lib*) \
	    $(CROSS_COMPILE)nm -D --defined-only $(VENDOR_LIB)/$$lib | awk '{print $$3}' | grep -qx "$$sym" || \
	    { echo "missing: $$lib $$sym"; bad=1; } ;; esac; done < $(STUB_SYMS); \
	  test $$bad = 0 && echo "all $$(grep -c '^lib' $(STUB_SYMS)) functions found"

# check-sdk-header: src/sdk/cv610.h against CV610_SDK_INC (its kernel/include
# and libraries/isp/include), object for object.
SDK_HEADERS = ot_common.h ot_common_sys.h ot_common_vb.h ot_common_video.h \
	ot_common_vi.h ot_common_vpss.h ot_common_venc.h ot_common_isp.h \
	ot_common_3a.h ot_mipi_rx.h ot_sns_ctrl.h ss_mpi_sys.h ss_mpi_sys_bind.h \
	ss_mpi_vb.h ss_mpi_vi.h ss_mpi_vpss.h ss_mpi_venc.h ss_mpi_isp.h \
	ss_mpi_ae.h ss_mpi_awb.h
SDK_INCS = $(addprefix -I$(CV610_SDK_INC)/,kernel/include/hi3516cv6xx \
	kernel/include/hi3516cv6xx/exp_inc libraries/isp/include/hi3516cv6xx \
	libraries/isp/include/hi3516cv6xx/3a libraries/isp/include/hi3516cv6xx/ext_inc)
SDK_SRCS = $(shell grep -l '^\#include "sdk/cv610.h"' $(SRCS))
SDK_CHECK = .sdk-check
need-sdk:
	@test -n "$(CV610_SDK_INC)" || { echo "CV610_SDK_INC: set it to the root of a Hi3516CV610 SDK"; exit 1; }
check-sdk-header: need-sdk
	@rm -rf $(SDK_CHECK); mkdir -p $(SDK_CHECK)/sdk/sdk
	@{ printf "#include <stddef.h>\n#include <stdint.h>\n#include <sys/ioctl.h>\n"; for h in $(SDK_HEADERS); do echo "#include \"$$h\""; done; } > $(SDK_CHECK)/sdk/sdk/cv610.h
	@bad=0; for c in $(SDK_SRCS); do o=$(SDK_CHECK)/$$(echo $$c | tr / _).o; \
	    $(CC) $(CFLAGS) -c $$c -o $$o.ours && \
	    $(CC) $(CFLAGS:-I$(SRC_DIR)=-I$(SDK_CHECK)/sdk -I$(SRC_DIR)) $(SDK_INCS) -c $$c -o $$o.sdk || exit 1; \
	    if cmp -s $$o.ours $$o.sdk; then echo "$$c: same"; else echo "$$c: DIFFERS"; bad=1; fi; done; \
	  rm -rf $(SDK_CHECK); test $$bad = 0

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
	rm -rf $(STUB_DIR) $(SDK_CHECK)

.PHONY: all clean test check-stubs need-sdk check-sdk-header
