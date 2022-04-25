# You can add more sources here and libraries if needed
SRCS +=
INCLUDES +=
LIBS +=
LDFLAGS +=


#Set node operating mode (i.e low-energy or low-latency)
ifneq ($(default_operating_mode),0)
    CFLAGS+= -DENABLE_LOW_LATENCY_MODE
endif

# Use Shared Data
SHARED_DATA=yes

# Use App Scheduler and configure it
APP_SCHEDULER=yes

APP_PRINTING=yes
CFLAGS += -DAPP_SCHEDULER_MAX_TASKS=2

SHARED_APP_CONFIG=yes
CFLAGS += -DSHARED_APP_CONFIG_MAX_FILTER=2
# Example:
# SRCS += $(SRCS_PATH)my_driver.c
# LIBS += my_lib.a
# include math.h lib
# LDFLAGS += -lm
HAL_SPI = yes