# How to use?

# QSB-USB2Serial_US-digital
This project is used for getting streaming data from US Digital USB-to-Serial adapter.

# How to solve the problem of not knowing the names of usb devices when multiple USB devices are plugged in the same computer?

Direct to usb_Udev_rules_change_names

# How to set up registers values of these QSB adapters?
          streaming_data_setup.cpp
    
gcc streaming_data_setup.cpp -o streamingdatasetup

./streamingdatasetup

# ROSpackage

Add these files into ros workspace /src folder then catkin_make, add this package

roscore

rosrun encoder_data encoder_node 

rostopic list:

std_msgs::UInt32 encoderl_data

std_msgs::Float32 speed   (this is the rotation speed)


# Reference C/C++ Coding
          qsb-helloworld.c
          qsb-helloworld.cpp
