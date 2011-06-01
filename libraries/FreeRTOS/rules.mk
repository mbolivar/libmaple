# Standard things
sp := $(sp).x
dirstack_$(sp) := $(d)
d := $(dir)
BUILDDIRS += $(BUILD_PATH)/$(d)

# Local flags
CXXFLAGS_$(d) := $(WIRISH_INCLUDES) $(LIBMAPLE_INCLUDES)

# Local rules and targets
cSRCS_$(d) := utility/*.c

cppSRCS_$(d) := MapleFreeRTOS.cpp

cFILES_$(d) := $(cSRCS_$(d):%=$(d)/%)
cppFILES_$(d) := $(cppSRCS_$(d):%=$(d)/%)

OBJS_$(d) := $(cFILES_$(d):%.c=$(BUILD_PATH)/%.o) \
             $(cppFILES_$(d):%.cpp=$(BUILD_PATH)/%.o)
DEPS_$(d) := $(OBJS_$(d):%.o=%.d)

$(OBJS_$(d)): TGT_CXXFLAGS := $(CXXFLAGS_$(d))

TGT_BIN += $(OBJS_$(d))

# Standard things
-include $(DEPS_$(d))
d := $(dirstack_$(sp))
sp := $(basename $(sp))