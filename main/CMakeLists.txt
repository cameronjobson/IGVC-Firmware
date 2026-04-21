set(srcs
        #"main.c"
        #"arduino_main.cpp"
        #"ROS2_test.cpp"
        "ROS2_serial_test.cpp"
)

#set(requires "bluepad32" "bluepad32_arduino" "arduino" "btstack" "callbacks" "micro_ros_espidf_component")
set(requires "arduino" "micro_ros_espidf_component") #for tests

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")
