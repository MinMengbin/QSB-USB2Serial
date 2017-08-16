/****
This project is used for getting streaming data from a USB-to_Serial adapter, 
and also you can use it to set up the values of the relevant registers.
Created by Mengbin Min (Mike) based on the support from US Digital.
Contact: msner10010@hotmail.com    16/08/2017

Usage: gcc qsb-helloworld.cpp -o qsb-helloworld_cpp then ./qsb-helloworld_cpp

Note:
1. This is not created for commercial purpose.
2. Anyone who applies this piece of work into his project should be fully responsible
for the outcome such as damages to properties
****/

#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>


#define FALSE 0
#define TRUE  1


// Default port. Assumes one QSB plugged in
// and no other USB devices posing as serial
// devices.
#define SERIAL_PORT "/dev/ttyUSB0"



// The response from a QSB has the general format:
// 
//     Element: Response  Register  Data  TimeStamp  EOR
//     Bytes  :     1         2       8       8       4
//
//
// Response, Register and Data, are always present.
//
// The rest of the elements are determined by the EOR (15) register.
// Each feature is enabled with a bit=1 or disabled with a bit=0.  
// The default value is: CR/LF = 0x03.
// 
// BITS:   B3 B2 B1 B0
// 
// B0 = Line Feed
// B1 = Carriage Return
// B2 = 4-byte Time Stamp appended to the response
// B3 = Spaces between returned fields
// 
// The timestamp is a 4-byte value but is appended to the 
// paket as an 8-byte hex string.
// 
// If spaces are used, the resulting format is:
// 
//   Response[space]Register[space]Data[space]TimeStampEOR
// 
// Thus the last space is right after the data value. Any
// trailing information (time stamp, etc.) is not separated
// by spaces.
// 
// The number below is size of the longest possible response 
// received from the QSB when all terminators are on and spaces
// separate the elements. It also allows for the string termination.
const int BUFFER_SIZE = 255;


// The USB link between the QSB and the computer introduces
// considerable lag. So, we need to wait a bit when sending
// an instruction before we get the response. The value of
// 5 ms is empirical and may need adjustment on other systems.
const int FIVE_MILLISECONDS = 20000;


// How many times we will try a non-blocking read of the 
// device before we call it a failed operation.
const int MAX_READ_TRIES = 3;



// Rather than getting into complications reading the
// keyboard for input, we will just trap Ctrl+C and 
// notify the app when the user is done.
int CloseRequested = FALSE;



void printQsbError(char* errorMessage)
{
	printf("%s: %d - %s", errorMessage, errno, strerror(errno));
    exit(-1);
}




// The QSB presents itself to the system as a vanilla
// UART and we can talk to it using the standard 
// POSIX IO functions.
//
// In the two utilitarian functions below, 'qsb' is
// a standard UNIX file handle pointing to the serial
// port where the devices is hosted.


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
    do
    {
        ioResult = read(qsb, response, responseSize);
        // This delay is to give some time to the device to
        // pipe the information to the serial port.
        //usleep(FIVE_MILLISECONDS);
        i++;
    } while (ioResult < 0 && errno == EAGAIN && i < MAX_READ_TRIES);

    if (i == MAX_READ_TRIES)
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



void ctrlCHandler(int signal)
{
    CloseRequested = TRUE;
}


// ************************************************* 
// 
//  General flow of operation:
//   1. Open the port and configure it.
//      The QSB UART wants 230.4K Baud, 8-n-1
//   2. Set up the QSB to read an encoder.
//   3. Loop polling and printing the current
//      encoder count.
//   4. Upon close request, close the port and
//      exit. 
// 
// *************************************************

int main (int argc, char *argv[])
{
    // Register the Ctrl-C handler.
    signal(SIGINT, ctrlCHandler);



    // Open the port.
    int qsb = open(SERIAL_PORT, O_RDWR | O_NOCTTY ); //| O_NONBLOCK | O_NOCTTY | O_NDELAY
    if (qsb < 0)
    {
        printQsbError("Error opening QSB device");
    }



    // Configure it.
    struct termios qsbConfiguration;
    tcgetattr(qsb, &qsbConfiguration);

    qsbConfiguration.c_lflag |= ICANON;  // Turn on the canonical mode
    qsbConfiguration.c_cflag |= (CLOCAL | CREAD);

    qsbConfiguration.c_cflag = B230400 | CS8 & ~CSIZE & ~CSTOPB & ~PARENB; // (8 bits, No parity bit, 1 stop bit)
    cfsetospeed(&qsbConfiguration, B230400);
    tcflush(qsb, TCSADRAIN);
    tcsetattr(qsb, TCSANOW, &qsbConfiguration);



    // Below is the basic communications protocol with 
    // the QSB. The commands were composed using the
    // QSB command list available for download at the
    // US Digital site. 


    char* command;
    char response[BUFFER_SIZE];
    char com_value[10] = {'R','1','4'};

    // Read and print product information.
    // Register: VERSION (0x14)
    com_value[0] = 'R'; com_value[1] ='1'; com_value[2]='4';
    qsbCommand(qsb, com_value, response, BUFFER_SIZE);
    printf("Product info: %s\n", response);
    
    // Set encoder in quadrature mode with  10-bit resolution.
    // Register: MODE (0x00)
    com_value[0] = 'W'; com_value[1] ='0'; com_value[2]='0';
    com_value[3] = '0'; com_value[4] ='0';
    qsbCommand(qsb, com_value, response, BUFFER_SIZE);
    printf("Quadrature response: %s\n", response);

    // Count up, 4X, Modulo-N.
    // Register: MDR0 (0x03)
    com_value[0] = 'W'; com_value[1] ='0'; com_value[2]='3';
    com_value[3] = '0'; com_value[4] ='B';
    qsbCommand(qsb, com_value, response, BUFFER_SIZE);
    printf("Count configuration: %s\n", response);

    // Disable counter.
    // Register: MDR1 (0x04)
    com_value[0] = 'W'; com_value[1] ='0'; com_value[2]='4';
    com_value[3] = '0'; com_value[4] ='0';
    qsbCommand(qsb, com_value, response, BUFFER_SIZE);
    printf("Count configuration: %s\n", response);

    // Set maximum count (Preset) to decimal 10000
    // Register: DTR (0x08)
    com_value[0] = 'W'; com_value[1] ='0'; com_value[2]='8';
    com_value[3] = '2'; com_value[4] ='7';
    com_value[5] = '1'; com_value[6] ='0';
    qsbCommand(qsb, com_value, response, BUFFER_SIZE);
    printf("Maximum count: %s\n", response);

    // Set encoder count threshold.
    // Register: THRESHOLD (0x0B)
    com_value[0] = 'W'; com_value[1] ='0'; com_value[2]='B';
    com_value[3] = '0'; com_value[4] ='0';
    com_value[5] = '0';
    printf("Quadrature response: %s\n", response);

    // Set up interval rate
    // Register: VERSION (0x0C)
    com_value[0] = 'W'; com_value[1] ='0'; com_value[2]='C';
    com_value[3] = '0'; com_value[4] ='0';
    com_value[5] = '0'; com_value[6] ='0';
    printf("Interval rate: %s\n", response);
    
    // Set end of reponse, New Line or line return '\n' at the end of the response
    // Register: EOR (0x15)
    com_value[0] = 'W'; com_value[1] ='1'; com_value[2]='5';
    com_value[3] = '0'; com_value[4] ='1';
    printf("EOR is: %s\n", response);

    // Saving QSB Configuration Parameters
    // Register: COMMAND (0x16)
    com_value[0] = 'W'; com_value[1] ='1'; com_value[2]='6';
    com_value[3] = '0'; com_value[4] ='0';
    com_value[5] = '0'; com_value[6] ='3';
    printf("Saving QSB Configuration Parameters: %s\n", response);

    // Deactivate Streaming mode
    // Register: DTR (0x0E)
    com_value[0] = 'R'; com_value[1] ='0'; com_value[2]='E';
    com_value[3] = '\0';
    printf("Read current encoder: %s\n", response);

    // Set Streaming mode
    // Register: DTR (0x0E)
    com_value[0] = 'S'; com_value[1] ='0'; com_value[2]='E';
    com_value[3] = '\0';
    printf("streaming data: %s\n", response);

    puts("\nUse Ctrl+C to exit.\n\n");

    unsigned long val_encoder = 0;

    while (CloseRequested == FALSE)
    {

        // Read current count.
        //Register: READ STRAMING DATA (0x0E)
        readQsbResponse(qsb, response, BUFFER_SIZE);
        printf("Current Response is: %s\n", response);
	    if (response[0] == 's'){
	    	char encoder_qsb[8] = {'0','0','0','0','0','0','0','0'};
			for (int i = 0; i < 8; i++){
				encoder_qsb[i] = response[3+i];
	    	}
			encoder_qsb[8] = '\0';
			printf(" Encoder Value in hex is %s. \n", encoder_qsb);
			val_encoder = strtol(encoder_qsb, NULL,16);
			printf("Encoder Value is %lu . \n", val_encoder);

	    }else
	    {
	    	printf("bad reading\n");
	    	CloseRequested == TRUE;
	    }
    }

    // Deactivate Streaming mode
    // Register: DTR (0x0E)
    //qsbCommand(qsb, "R0E", response, BUFFER_SIZE);

    close(qsb);
    return(0);
}

