ifeq ($(TARGET_ARCH),arm)
ifeq ($(ARCH_ARM_HAVE_ARMV7A),true)
    include $(call all-subdir-makefiles)
endif
else
    include $(call all-subdir-makefiles)
endif
