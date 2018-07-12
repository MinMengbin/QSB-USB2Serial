/*
Author: Mike Min
Email: msner10010@hotmail.com
Date: 01/07/2018
-- version 1.0

-The documents about QSB from USdigital, please go to official web site

-Before you start running, please set up the adapter
which is the /home/aspa1/Documents/QSB_encoder
 streaming_data_setup.cpp

-Open a terminal and input: ./streamdatasetup
-Then: rosrun encoder_data encoder_node

*/



#include "ros/ros.h"

#include "std_msgs/UInt32.h"  //Unsigned int 32 for encoder data
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#include "encoder_data/encoderval.h" // msg type

#include <iostream>
#include <vector>

#include <math.h>
#include "string.h"
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#define FALSE 0
#define TRUE  1

using namespace std;

const int BUFFER_SIZE = 27;

// The USB adapter for the serial number of 81830
#define SERIAL_PORT_81830 "/dev/serial/by-id/usb-US_Digital_USB__-__QSB_81830-if00-port0"
// If you write Udev rules, you can change names to any you want
// otherwise you use the above longer one
#define SERIAL_PORT "/dev/QSB830"


void printQsbError(char* errorMessage)
{
        ROS_INFO("%s: %d - %s", errorMessage, errno, strerror(errno));
        exit (-1);
}

void sendQsbInstruction(int qsb, char* command)
{
    // Create a padded instruction string that includes
    // CR+LF. The QSB will be happy with just CR or LF too.
    char* qsbCommand = (char *)malloc(strlen(command) + 3);
    sprintf(qsbCommand, "%s\r\n", command);

    int ioResult = write(qsb, qsbCommand, strlen(qsbCommand));
    free(qsbCommand);

    if (ioResult < 0)
    {
        printQsbError("Error writing to QSB device");
    }
}

void readQsbResponse(int qsb, char* response, int responseSize)
{
    int i = 0;
    int ioResult;
    ioResult = read(qsb, response, responseSize);
	if (ioResult < 0)
	{
	  printQsbError("Error reading from QSB device");
	}

    // Remove the trailing CR+LF if any, and trim to proper size.
    int end = strcspn(response, "\r\n");
    response[end] = '\0';

    if (ioResult < responseSize)
    {
        response[ioResult] = '\0';
    }
}

// Every instruction sent to the QSB is acknowledged
// with a corresponding response string. We then send
// the instruction and retrieve the response as a
// single command transaction.
void qsbCommand(int qsb, char* command, char* response, int responseSize)
{
    sendQsbInstruction(qsb, command);
    readQsbResponse(qsb, response, responseSize);
}

// *************************************************
//
//  General flow of operation:
//   1. Open the port and configure it.
//      The QSB UART wants 230.4K Baud, 8-n-1
//   2. Set up the QSB to read an encoder.
//   3. Loop polling and printing the current
//      encoder count and publish the speed value.
//   4. Upon close request, close the port and
//      exit.
//
// *************************************************


int main (int argc, char *argv[])
{
    // Register the Ctrl-C handler.
    //signal(SIGINT, ctrlCHandler);

	char* command;
    char response[BUFFER_SIZE];

    std::vector <double> t, v_encoder, timestamp;
    t.assign(2,0);
    v_encoder.assign(2,0);
    timestamp.assign(2,0);

	// ROS initialise
	ros::init(argc, argv, "encoder");
	ros::NodeHandle n_encoderl;

	ros::Publisher encoderl_Pub = n_encoderl.advertise<encoder_data::encoderval>("encoderl", 1);

	double t_sec;

    // serial port handle.
    int qsb;


   // Open the port.
	ROS_INFO("Trying to open a serial port.");
	qsb = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
	if (qsb < 0)
	{
		printQsbError("Error opening QSB device");
	}

	ROS_INFO("Serial ports are connected.");

	// Configure serial port.
	struct termios qsbConfiguration;
	tcgetattr(qsb, &qsbConfiguration);

	// Turn on the canonical mode
    qsbConfiguration.c_lflag = ICANON;
    qsbConfiguration.c_cc[VMIN] = 1;
    qsbConfiguration.c_cc[VTIME]= 0;

	qsbConfiguration.c_cflag = B230400 | CS8;
	cfsetospeed(&qsbConfiguration, B230400);
	tcflush(qsb, TCIFLUSH);
	tcsetattr(qsb, TCSANOW, &qsbConfiguration);

	// Activate streaming value mode output
	// Register: VERSION (0x0E)
	qsbCommand(qsb, "S0E", response, BUFFER_SIZE);
	ROS_INFO("Streaming data mode is on.");
	ROS_INFO("Current output: %s.", response);

/*
	// Waiting for reading is stable if needed
	for(int i = 0; i < 3; i++){
    	readQsbResponse(qsb, response, BUFFER_SIZE);
    	ROS_INFO("Current output: %s.", response);
    }
*/

	// publish value
	encoder_data::encoderval val_l;

	val_l.val_speed = 0;
	val_l.val_encoder = 0;


	// According to encoder values then calculate speed and publish
    while ( ros::ok())
    {
         // Read steam data.
         readQsbResponse(qsb, response, BUFFER_SIZE);
         ROS_INFO("\nResponse is : %s\r", response);

        if (response[0] != 's' ) { // check the streaming data is right or not
         // if the streaming data is not right, close the program
				qsbCommand(qsb, "R0E", response, BUFFER_SIZE);
				ROS_INFO("R: %s.", response);
				ROS_INFO("Streaming data mode is off and the program is shut.");
				close(qsb);
				return 0;
         }else{ // calculate the speed
			 char encoder_qsb[8] = {'0','0','0','0','0','0','0','0'};
			 for (int i = 0; i < 8; i++){
				 encoder_qsb[i] = response[3+i];
			 }
			encoder_qsb[8] = '\0';
			ROS_INFO("Encoder Value in hex is %s. \r", encoder_qsb);
			val_l.val_encoder = strtol(encoder_qsb, NULL,16);
			ROS_INFO("Encoder Value in decimal is %u. \r", val_l.val_encoder);

			char time_stamp[8] = {'0','0','0','0','0','0','0','0'};
			for (int i = 0; i < 8; i++){
				 time_stamp[i] = response[11+i];
			 }
			time_stamp[8] = '\0';
			ROS_INFO("Time Stamp Value in hex is %s. \r", time_stamp);
			timestamp[0] = timestamp[1];
			timestamp[1] = strtol(time_stamp, NULL,16);

			ROS_INFO("stamp time interval is: %f ms.", (timestamp[1] - timestamp[0])*1.9);

			if((timestamp[1] - timestamp[0])*1.9 != 1.9 && timestamp[0] !=0 ){
				goto finish;
			}

	    	t_sec = ros::Time::now().toSec();
	    	t[0] = t[1];
	    	t[1] = t_sec;

	    	v_encoder[0] = v_encoder[1];
	    	v_encoder[1] = val_l.val_encoder;

	    	// rotation speed (unit: r/min)
	    	// 5000 cpr/ x2 quadrature count mode output for this incremental encoder
	    	val_l.val_speed = (60*1000*(v_encoder[1] - v_encoder[0]))/(5000*((timestamp[1] - timestamp[0])*1.9));

	    	ROS_INFO("ROS time interval is: %f.", t[1] - t[0]);
	    	ROS_INFO("value_encoder interval is: %f.", v_encoder[1] - v_encoder[0]);

	    	val_l.header.stamp = ros::Time::now();
	    	val_l.header.frame_id = SERIAL_PORT;

	    	encoderl_Pub.publish(val_l);

		}
    }

    finish:
    // Deactivate streaming value mode output
	// Register: VERSION (0x0E)
	qsbCommand(qsb, "R0E", response, BUFFER_SIZE );
	ROS_INFO("R: %s.", response);
	ROS_INFO("Streaming data mode is off.");
    close(qsb);

    return(0);
}





