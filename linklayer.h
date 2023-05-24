#ifndef LINKLAYER
#define LINKLAYER

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

typedef struct linkLayer{
    char serialPort[50];
    int role; //defines the role of the program: 0==Transmitter, 1=Receiver
    int baudRate;
    int numTries;
    int timeOut;
} linkLayer;

typedef enum{
    Start,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    MSTOP
}Estados;

//ROLE
#define NOT_DEFINED -1
#define TRANSMITTER 0
#define RECEIVER 1


//SIZE of maximum acceptable payload; maximum number of bytes that application layer should send to link layer
#define MAX_PAYLOAD_SIZE 1000
#define MAX_FRAME_SIZE 2100

//CONNECTION deafault values
#define BAUDRATE_DEFAULT B38400
#define MAX_RETRANSMISSIONS_DEFAULT 3
#define TIMEOUT_DEFAULT 4
#define _POSIX_SOURCE 1 /* POSIX compliant source */

//MISC
#define FALSE 0
#define TRUE 1
#define RR_INFO 2
#define INFO 1
#define SUPERVISION 0


//CODES
#define FLAG 0x5c
#define A_TRAN 0x01
#define A_REC 0x03
#define SET 0x03
#define DISC 0x0B
#define UA 0x07
#define C_0 0x00
#define C_1 0x02
#define RR_0 0x01
#define REJ_0 0x03
#define RR_1 0x21
#define REJ_1 0x23


//BYTE STUFFING
#define ESC 0x5d
#define ESC_STUFFING 0x7d
#define FLAG_STUFFING 0x7c 

// Opens a connection using the "port" parameters defined in struct linkLayer, returns "-1" on error and "1" on sucess
int llopen(linkLayer connectionParameters);
// Sends data in buf with size bufSize
int llwrite(char* buf, int bufSize);
// Receive data in packet
int llread(char* packet);
// Closes previously opened connection; if showStatistics==TRUE, link layer should print statistics in the console on close
int llclose(linkLayer connectionParameters, int showStatistics);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////AUXIILIARY FUNCTIONS//////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief Open file decriptor needed to execute the serial port communication
/// @param connectionParameters parameter given by the main.c file
/// @return file descriptor in case of success and -1 in case of error
int open_connection(linkLayer connectionParameters);

/// @brief handles the llopen transmitter part, send SET frame, alarm instalation, receive UA frame  
/// @param fd file descriptor
/// @param param parameter given by the main.c file
/// @return file descriptor in case of success and -1 in case of error
int open_transmitter(int fd, linkLayer param);

/// @brief Open the reciver mode, wait to receive the SET frame and send the UA frame
/// @param fd file descriptor
/// @param param parameter given by the main.c file
/// @return file descriptor in case of success and -1 in case of error
int open_receiver(int fd, linkLayer param); 

/// @brief write the frame to the file descriptor
/// @param msg frame to be written
/// @param fd file descriptor
/// @param size frame size
/// @return size of frame written in case of success and -1 in case of error
int sendframe(unsigned char* msg, int fd, int size);

/// @brief reads the frame and sends it to the the state machine
/// @param msg2rcv frame that we are supposed to receive
/// @param fd file descriptor
/// @param type SUPERVISON if supervison frame, INFO if information frame, 2 if RR or REJ frame
/// @return 1 in case a byte has been read, 0 in other case
int readframe(unsigned char* msg2rcv, int fd, int type);

/// @brief state machine to handle every bytes received in the frame
/// @param code byte received
/// @param cod2recv frame that we are supposed to receive
void statemachine(char code, unsigned char* cod2recv);

/// @brief function trigered by the alarm, maximize the number of time we sent a frame
void resend();

/// @brief creates a frame with the format of the I frame, and keep it in the global variable I_frame
/// @param I_buf data to be build into the I frame
/// @param I_buf_size size of the data to be build into I frame
/// @param control give the control byte to the I frame, C_0 or C_1
/// @return size of frame
int build_info_frame(unsigned char* I_buf, int I_buf_size,unsigned char control);

/// @brief state machine to the response frame of the I frame
/// @param code byte received
/// @param cod2recv RR_1 and REJ_1 or RR_0 and REJ_0
void state_RR_info_machine(char code, unsigned char* cod2recv);

/// @brief state machine the I frame
/// @param code byte received
/// @param cod2recv control byte C_0 or C_1
void state_info_machine(char code, unsigned char* cod2recv);

/// @brief handles the llclose transmitter part, send DISC frame, alarm instalation, receive DISC frame, send UA frame
/// @param fd file descriptor
/// @param param parameter given by the main.c file
/// @return file descriptor, -1 in case of error
int close_transmitter(int fd, linkLayer param);

/// @brief handles the llclose transmitter part, wait to receive DISC frame, sent DISC frame, wait to receive UA frame
/// @param fd file descriptor
/// @param param parameter given by the main.c file
/// @return file descriptor, -1 in case of error
int close_receiver(int fd, linkLayer param);

#endif
