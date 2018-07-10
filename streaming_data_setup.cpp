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


// The USB adapter for the serial number of 81830
#define SERIAL_PORT_81830 "/dev/serial/by-id/usb-US_Digital_USB__-__QSB_81830-if00-port0"
#define SERIAL_PORT "/dev/QSB830"


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
const int BUFFER_SIZE = 27;
const int buffer_rec_size = 27;

// The USB link between the QSB and the computer introduces
// considerable lag. So, we need to wait a bit when sending
// an instruction before we get the response. The value of
// 5 ms is empirical and may need adjustment on other systems.
const int FIVE_MILLISECONDS = 100000;


// How many times we will try a non-blocking read of the 
// device before we call it a failed operation.
const int MAX_READ_TRIES = 100;



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

/*
 // this coding for non-block mode ECHO
   do
    {
        ioResult = read(qsb, response, responseSize);
        // This delay is to give some time to the device to
        // pipe the information to the serial port.
        usleep(FIVE_MILLISECONDS);
        i++;
        printf("i is %d\n",i);
    } while (ioResult < 0 && errno == EAGAIN && i < MAX_READ_TRIES);
    printf("ioResult is %d\n",ioResult);


    if (i == MAX_READ_TRIES)
    {
        printQsbError("Error reading from QSB device");
    }
    // this coding for non-block mode ECHO
*/

    ioResult = read(qsb, response, responseSize);
    if (ioResult < 0)
    {
        printQsbError("Error reading from QSB device");
    }
    printf("reveived: %s. \n",response);
    printf("ioResult is %d\n",ioResult);
    // Remove the trailing CR+LF if any, and trim to proper size.
    int end = strcspn(response, "\r\n");
    response[end] = '\0';

/*    if (ioResult < responseSize)
    {
        response[ioResult] = '\0';
    }*/
}


void readStreamData(int qsb, char* response, int responseSize)
{
    int i = 0;
    int ioResult;
    do
    {
        ioResult = read(qsb, response, responseSize);
        i++;
    } while (ioResult < 0 && errno == EAGAIN && i < MAX_READ_TRIES);

    if (ioResult != 0)
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

    char* command;
    char response[BUFFER_SIZE];
    char buffer_rec[buffer_rec_size];
    // Open the port.
    printf("Trying to open serial port......\n");
    int qsb = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
    if (qsb < 0)
    {
        printQsbError("Error opening QSB device");
    }
    printf("Serial ports are connected.\n");
    // Read steam data.
    //printf("Display the first recived message froom QSB. \n");
    //readStreamData(qsb, response, 27);
    //printf("The product is: %s\r", response);

    // Configure serial port.
    struct termios qsbConfiguration;
    tcgetattr(qsb, &qsbConfiguration);


    qsbConfiguration.c_cflag = B230400 | CS8;
    cfsetospeed(&qsbConfiguration, B230400);

    // Turn on the canonical mode
    qsbConfiguration.c_lflag = ICANON;
    qsbConfiguration.c_cc[VMIN] = 1;
    qsbConfiguration.c_cc[VTIME]= 0;

    // Turn on ECHO model if needed
    //qsbConfiguration.c_lflag = ECHO;

    tcflush(qsb, TCIFLUSH);
    tcsetattr(qsb, TCSANOW, &qsbConfiguration);



    // Below is the basic communications protocol with 
    // the QSB. The commands were composed using the
    // QSB command list available for download at the
    // US Digital site. 


    // Read encoder data and deactivate streaming value mode ouput
    // Register: VERSION (0x0E)
    //qsbCommand(qsb, "R0E", response, BUFFER_SIZE);


    // Read and print product information.
    // Register: VERSION (0x14)
    printf("Request product info......\n");
    qsbCommand(qsb, "R14", response, BUFFER_SIZE);
    printf("Product info: %s\n", response);
    
    // Set encoder in quadrature mode.
    // Register: MODE (0x00)
    qsbCommand(qsb, "W0000", response, BUFFER_SIZE);
    printf("Quadrature response: %s\n", response);

    // Count up, 1X, Modulo-N.
    // Register: MDR0 (0x03)
    qsbCommand(qsb, "W030E", response, BUFFER_SIZE);
    printf("Count configuration: %s\n", response);

    // Set maximum count (Preset) to decimal 499
    // Register: DTR (0x08)
    qsbCommand(qsb, "W08FFFFFFFF", response, BUFFER_SIZE);
    printf("Maximum count: %s\n", response);

    // Activate streaming value mode ouput
    // Register: VERSION (0x0E)
    //qsbCommand(qsb, "S0E", response, BUFFER_SIZE);
    //printf("Streaming mode output: %s\n", response);
    
    // Set up output value threshhold
    // Register: VERSION (0x0B)
    qsbCommand(qsb, "W0B0000", response, BUFFER_SIZE);
    printf("Output value threshhold: %s\n", response);

    // Set up interval rate
    // Register: VERSION (0x0C)
    qsbCommand(qsb, "W0C0001", response, BUFFER_SIZE); // 1.9 ms
    printf("Interval rate: %s\n", response);

    // Set up EOF (This is important if CANONICAL model is used)
    // 0x01 means EOF is LF
    // Register: VERSION (0x15)
    qsbCommand(qsb, "W1501", response, BUFFER_SIZE);
    printf("Interval rate: %s\n", response);

     // Read encoder data and deactivate streaming value mode ouput
     // Register: VERSION (0x0E)
     qsbCommand(qsb, "R0E", response, BUFFER_SIZE);
     printf("Streaming mode output: %s\n", response);


     // Saving QSB Configuration Parameters
     // Register: VERSION (0x16)
     qsbCommand(qsb, "W160003", response, BUFFER_SIZE);
     printf("Saving QSB Configuration Parameters: %s\n", response);

     // Activate streaming value mode ouput
     // Register: VERSION (0x0E)
     qsbCommand(qsb, "S0E", response, buffer_rec_size);
     printf("Streaming mode output: %s\n", response);

    while(CloseRequested == FALSE){

    	readQsbResponse(qsb, buffer_rec, buffer_rec_size);
		printf("Streaming mode output: %s\n", buffer_rec);
    }
    // Deactive streaming value mode ouput and close serial port
    qsbCommand(qsb,"R0E",response,BUFFER_SIZE);
    close(qsb);
    printf("serial communication disconnected!\n");
    return(0);
}
