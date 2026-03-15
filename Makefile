XLEN ?= 32
DESTDIR ?= $(CURDIR)
COMMON_DIR = $(abspath common)
SRC_DIR = $(abspath src)

CXXFLAGS += -std=c++17 -Wall -Wextra -Wfatal-errors
CXXFLAGS += -fPIC -Wno-maybe-uninitialized
CXXFLAGS += -I$(CURDIR) -I$(COMMON_DIR)
CXXFLAGS += -DXLEN_$(XLEN)
CXXFLAGS += $(CONFIGS)

LDFLAGS +=

SRCS = $(COMMON_DIR)/util.cpp $(COMMON_DIR)/mem.cpp
SRCS += $(SRC_DIR)/main.cpp $(SRC_DIR)/processor.cpp $(SRC_DIR)/core.cpp $(SRC_DIR)/decode.cpp
SRCS += $(SRC_DIR)/ooo.cpp $(SRC_DIR)/RS.cpp $(SRC_DIR)/LSQ.cpp $(SRC_DIR)/ROB.cpp
SRCS += $(SRC_DIR)/ALU.cpp $(SRC_DIR)/BRU.cpp $(SRC_DIR)/LSU.cpp $(SRC_DIR)/SFU.cpp
SRCS += $(SRC_DIR)/BP.cpp
SRCS += $(SRC_DIR)/cache.cpp $(SRC_DIR)/memory.cpp

# Debugigng
ifdef DEBUG
  ifneq ($(DEBUG),false)
	CXXFLAGS += -g -O0 -DDEBUG_LEVEL=$(DEBUG)
  else
	CXXFLAGS += -O2 -DNDEBUG
  endif
else
	CXXFLAGS += -O2 -DNDEBUG
endif

PROJECT = tinyrv

all: $(DESTDIR)/$(PROJECT)

$(DESTDIR)/$(PROJECT): $(SRCS)
	$(CXX) $(CXXFLAGS) $^ $(LDFLAGS) -o $@

.PHONY: tests benchmarks

.depend: $(SRCS)
	$(CXX) $(CXXFLAGS) -MM $^ > .depend;

tests: $(DESTDIR)/$(PROJECT)
	$(MAKE) -C tests run

benchmarks: $(DESTDIR)/$(PROJECT)
	$(MAKE) -C benchmarks run

submit:
	@echo "-- ZIPPING ALL THE FILE ---------"
	zip submission.zip src/*

clean:
	rm -rf $(DESTDIR)/$(PROJECT)
