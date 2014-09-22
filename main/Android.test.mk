include $(CLEAR_VARS)

LOCAL_MULTILIB := first
LOCAL_MODULE := vg-$(test)
LOCAL_SRC_FILES := none/tests/$(TARGET_ARCH)/$(test).c
LOCAL_NATIVE_COVERAGE := false

include $(BUILD_NATIVE_TEST)
