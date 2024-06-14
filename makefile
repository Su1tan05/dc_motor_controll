PROJECT         := dc_motor_controll_new
DEVICES         := NUCLEO_F411RE
GCC4MBED_DIR    := $(GCC4MBED_DIR)
USER_LIBS       := !$(ROS_LIB_DIR) $(ROS_LIB_DIR)/BufferedSerial /home/ubuntu/motor_controll/dc_motor_controll/Libs
NO_FLOAT_SCANF  := 1
NO_FLOAT_PRINTF := 1

include $(GCC4MBED_DIR)/build/gcc4mbed.mk