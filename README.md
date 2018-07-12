This project is used for getting streaming data from US Digital USB-to-Serial adapter.

# How to use QSB-USB2Serial_US-digital?

User manual about QSB from US digital, please go to official web site

# How to solve the problem of not knowing the names of usb devices when multiple USB devices are plugged in the same computer?

Direct to usb_Udev_rules_change_names

# How to set up registers values of these QSB adapters?
          streaming_data_setup.cpp
    
gcc streaming_data_setup.cpp -o streamingdatasetup

./streamingdatasetup

# ROSpackage

Add folder /encoder_data into your ros workspace /src folder then catkin_make, add this package

roscore

rosrun encoder_data encoder_node 

rostopic list: encoderl

          header:
                    seq: 611873
                   stamp: 
                        secs: 1531357195
                        nsecs: 265082307
                  frame_id: "/dev/QSB830"
          val_encoder: 333359
          val_speed: 0.0

Test Video: https://youtu.be/QE1XKatoCH8

# Reference C/C++ Coding
          qsb-helloworld.c
          qsb-helloworld.cpp
