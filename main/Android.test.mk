include $(CLEAR_VARS)

LOCAL_MODULE=$(test)
LOCAL_SRC_FILES=none/tests/arm/$(test).c

include $(BUILD_NATIVE_TEST)
