#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

CFLAGS += -Wno-unused-variable -Wno-maybe-uninitialized -DI2C_HandleTypeDef=int