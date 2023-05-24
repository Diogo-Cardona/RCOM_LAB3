#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "linklayer.h"

int tries, n_retries, timeout, flag, STOP, retry, fd, end;
int right_code; //=1 if RR, =2 if REJ, =-1 otherwise
int i_byte;
struct termios oldtio, newtio;

Estados estado;
int sequence = 0;
unsigned char I_frame[MAX_FRAME_SIZE];
unsigned char aux[5];

//////////AUXILIARY FUNCTIONS//////////

int open_connection(linkLayer param)
{
    int fd;
    //in order to set the baudrate
    char baudrate[15]={'B'};
    char string[14];
    sprintf(string, "%d", param.baudRate);
    strcat(baudrate, string);
    int Brate= atoi(baudrate);
    

    if ( (strlen(param.serialPort)<10) ||
        ((strcmp("/dev/ttyS0", param.serialPort)!=0) &&
        (strcmp("/dev/ttyS1", param.serialPort)!=0) )) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
        return -1;
    }

    fd = open(param.serialPort, O_RDWR | O_NOCTTY );
    if (fd < 0) { perror(param.serialPort); exit(-1); }

    if ( tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
        perror("tcgetattr");
        return -1;
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = Brate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 0.1;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 0;   /* blocking read until 5 chars received */

    tcflush(fd, TCIOFLUSH);
    sleep(1);
    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        return -1;
    }
    
    printf("New termios structure set\n");
    return fd;
}

int open_transmitter(int fd, linkLayer param){
    char buf[255];
    int res;
    unsigned char msg[5] = {FLAG, A_TRAN, SET, A_TRAN^SET, FLAG};
    unsigned char msg2rcv[5] = {FLAG, A_TRAN, UA, A_TRAN^UA, FLAG};
    n_retries = 1;
    printf("entrei no open transmitter\n");

    if((res = sendframe(msg, fd, 5))==-1){
        return -1;
    }
    printf("SET frame sent, %d bytes\n", res);

    int bytes_received=0;
    flag=1;
    end=0;
    while (STOP==FALSE)
    {
        if((res = readframe(msg2rcv, fd,SUPERVISION))>0)  /* returns after 1 chars have been input */
        {
            bytes_received += res;
            alarm(0);
        }

        if(end)
        {
            flag=0;
            alarm(0);
            STOP=TRUE;
            printf("Number of retries exceeded couldn't receive SET frame\n");
            return -1;
        }
        if (flag)
        {
            if (retry)
            {
                if((res = sendframe(msg, fd,5))==-1){
                return -1;
                }
                printf("Timeout: SET frame sent, %d bytes (try: %d/%d)\n", res, n_retries, param.numTries);
                retry=0;
            }
            alarm(param.timeOut);
            flag=0;
        }
        
        if(estado==MSTOP)
        {
            printf("UA frame received, %d bytes received\n", bytes_received);
            STOP=TRUE;
        }      
    }
    return fd;
}

int open_receiver(int fd, linkLayer Param){
    int res, bytes_received=0;
    unsigned char msg[5] = {FLAG, A_TRAN, UA, A_TRAN^UA, FLAG};
    unsigned char msg2rcv[5] = {FLAG, A_TRAN, SET, A_TRAN^SET, FLAG};
    estado = Start; // State machine initial state 

    while (STOP==FALSE) {       //loop for input 
        bytes_received += readframe(msg2rcv, fd,SUPERVISION);
        if (estado==MSTOP)
        {
            printf("Set frame received, %d bytes received\n", bytes_received);
            if( (res = sendframe(msg, fd,5)) ==-1){
                return -1;
            }
            printf("UA frame sent, %d bytes\n", res);
            STOP=TRUE;
        }
    }
    return fd;
}

int sendframe( unsigned char* msg, int fd, int size){
    int res;
    if ( (res= write(fd, msg, size)) <=0)
    {
        return -1;
    }
    return res;
}

int readframe(unsigned char* msg2rcv, int fd, int type){
    int res;
    char buf[255];
    res = read(fd,buf,1);  /* returns after 1 chars have been input */
    if (res>=1){
        buf[res]=0;               /* so we can printf... */     
        if (type==SUPERVISION)
        {
            statemachine(buf[0], msg2rcv);
        }
        if (type==INFO)
        {
            state_info_machine(buf[0], msg2rcv); 
        }
        if (type==2)
        {
            state_RR_info_machine(buf[0],msg2rcv);
        }
        return res;
    }
    return 0;
}

void statemachine(char code, unsigned char* codigo)
{
    switch(estado)
    {
    case Start:
        if (code==codigo[0])
            estado = FLAG_RCV;
        break;
    case FLAG_RCV:
        if (code==codigo[0])
            break;
        if (code==codigo[1])
            estado = A_RCV;
        else
            estado = Start;
        break;
    case A_RCV:
        if(code==codigo[0])
            estado = FLAG_RCV; 
        if(code==codigo[2])
            estado = C_RCV;
        else
            estado = Start;
        break;
    case C_RCV:
        if(code==codigo[0])
            estado = FLAG_RCV;
        if(code==codigo[3])
            estado = BCC_OK;
        else
            estado = Start;
        break;
    case BCC_OK:
        if(code==codigo[4])
            estado = MSTOP;        
        else
            estado = Start;
        break;
    default:
        break;
    }
}

void resend(){
    
    if(n_retries < tries){
        flag = 1;
        n_retries++;
        retry=1;
    }
    else{
        end=1;
        flag=0;
    }
}

int build_info_frame(unsigned char* I_buf, int I_buf_size, unsigned char control)
{
    unsigned char BCC2=I_buf[0];
    I_frame[0] = FLAG;
    I_frame[1] = A_TRAN;
    I_frame[2] = control;
    I_frame[3] = I_frame[1] ^ I_frame[2];

    int j=4;
    for (int i = 0; i < I_buf_size; i++)
    {
        if(i>=1){
            BCC2 = BCC2 ^ I_buf[i];
        }
        if(I_buf[i]==FLAG){
            I_frame[j] = ESC;
            j++;
            I_frame[j]=FLAG_STUFFING;
            j++;
        }
        else if (I_buf[i]==ESC)
        {
            I_frame[j] = ESC;
            j++;
            I_frame[j]=ESC_STUFFING;
            j++;
        }
        else{
            I_frame[j] = I_buf[i];
            j++;
        }
    }
    if(BCC2==FLAG){
        I_frame[j] = ESC;
        j++;
        I_frame[j]=FLAG_STUFFING;
        j++;
    }
    else if (BCC2==ESC)
    {
        I_frame[j] = ESC;
        j++;
        I_frame[j]=ESC_STUFFING;
        j++;
    }
    else{
        I_frame[j] = BCC2;
        j++;
    }
    I_frame[j]=FLAG;
    j++;
    return j;
}

void state_RR_info_machine(char code, unsigned char* cod2recv){
    switch(estado)
    {
        case Start:
            if (code==FLAG){
                estado = FLAG_RCV;
                aux[0]=code;
            }
            break;
        case FLAG_RCV:
            if (code==FLAG)
                break;
            if (code==A_TRAN){
                estado = A_RCV;
                aux[1]=code;
            }
            else
                estado = Start;
            break;
        case A_RCV:
            right_code=0;
            if(code==FLAG)
                estado = FLAG_RCV; 
            if(code==cod2recv[0]){
                estado = C_RCV;
                right_code=1;
                aux[2]=code;
            }
            else if(code==cod2recv[1]){
                estado = C_RCV;
                right_code=2;
                aux[2]=code;
            }
            else
                estado = Start;
            break;
        case C_RCV:
            if(code==FLAG)
                estado = FLAG_RCV;
            if(code==(aux[1] ^aux[2])){
                //printf(">>2 BCC\n");
                estado = BCC_OK;
                aux[3]=code;
            }
            else
                estado = Start;
            break;
        case BCC_OK:
            if(code==FLAG){
                aux[4]=code;
                estado = MSTOP;   
            }     
            else
                estado = Start;
            break;
        default:
            break;
    }
}

void state_info_machine(char code, unsigned char* cod2recv){    
    switch(estado)
    {
        case Start:
            if (code==FLAG){
                estado = FLAG_RCV;
                I_frame[i_byte]=FLAG;
                i_byte++;
            }
            break;
        case FLAG_RCV:
            if (code==FLAG)
                break;
            if (code==A_TRAN){
                estado = A_RCV;
                I_frame[i_byte]=A_TRAN;
                i_byte++;
            }
            else
                estado = Start;
            break;
        case A_RCV:
            if(code==FLAG)
                estado = FLAG_RCV; 
            if(code==C_0){
                estado = C_RCV;
                I_frame[i_byte]=C_0;
                i_byte++;
                break;
            }
            if(code==C_1){
                estado = C_RCV;
                I_frame[i_byte]=C_1;
                i_byte++;
                break;
            }
            else
                estado = Start;
            break;
        case C_RCV:
            if(code==FLAG)
                estado = FLAG_RCV;
            if(code==(I_frame[1]^I_frame[2] )){
                estado = BCC_OK;
                I_frame[i_byte]=code;
                i_byte++;
            }
            else
                estado = Start;
            break;
        case BCC_OK:
            if(code==FLAG){
                estado = MSTOP; 
                I_frame[i_byte]=FLAG;
                i_byte++;
            }       
            else{
                I_frame[i_byte]=code;
                i_byte++;
            }    
            break;
        default:
            break;
    }
}

int bytedestuffing()
{
    unsigned char aux[i_byte];
    for (int i = 0; i < i_byte; i++)
    {
        aux[i]=I_frame[i]; 
    }

    int frame_size=0;
    for (int i = 0; i < i_byte; i++)
    {
        if (aux[i]==ESC)
        {
            if (aux[i+1]==ESC_STUFFING)
            {
                I_frame[frame_size]=ESC;
            }
            else //if (aux[i+1]=FLAG_STUFFING)
            {
                
                I_frame[frame_size]=FLAG;
            }
            i++;
            frame_size++;
        }
        else
        {
            I_frame[frame_size]=aux[i];
            frame_size++;
        }
    }
    return frame_size;
}

int close_transmitter(int fd, linkLayer param)
{
    int res;
    unsigned char msg[5] = {FLAG, A_TRAN, DISC, A_TRAN^DISC, FLAG};
    unsigned char msg2rcv[5] = {FLAG, A_REC, DISC, A_REC^DISC, FLAG};
    n_retries = 1;

    if((res = sendframe(msg, fd, 5))==-1){
        return -1;
    }
    printf("DISC frame sent, %d bytes\n", res);

    int bytes_received=0;
    flag=1;
    end=0;
    STOP=FALSE;
    estado=Start;
    while (STOP==FALSE)
    {
        if((res = readframe(msg2rcv, fd,SUPERVISION))>0)  /* returns after 1 chars have been input */
        {
            bytes_received += res;
            alarm(0);
        }
        if(end)
        {
            flag=0;
            alarm(0);
            STOP=TRUE;
            printf("Number of retries exceeded couldn't receive DISC frame\n");
            return -1;
        }
        if (flag)
        {
            if (retry)
            {
                if((res = sendframe(msg, fd,5))==-1){
                return -1;
                }
                printf("Timeout: DISC frame sent, %d bytes (try: %d/%d)\n", res, n_retries, param.numTries);
                retry=0;
            }
            alarm(param.timeOut);
            flag=0;
        }
        
        if(estado==MSTOP)
        {
            printf("DISC frame received, %d bytes received\n", bytes_received);
            STOP=TRUE;
        }      
    }
    unsigned char msg_2[5] = {FLAG, A_REC, UA, A_REC^UA, FLAG};
    if((res = sendframe(msg_2, fd, 5))==-1){
        return -1;
    }
    printf("UA frame sent, %d bytes\n", res);

    return fd;
}

int close_receiver(int fd, linkLayer param)
{
    int res, bytes_received=0;
    unsigned char msg[5] = {FLAG, A_REC, DISC, A_REC^DISC, FLAG};
    unsigned char msg2rcv[5] = {FLAG, A_TRAN, DISC, A_TRAN^DISC, FLAG};
    estado = Start; // State machine initial state 
    STOP = FALSE;

    while (STOP==FALSE) {       //loop for input 
        bytes_received += readframe(msg2rcv, fd,SUPERVISION);
        if (estado==MSTOP)
        {
            printf("DISC frame received, %d bytes received\n", bytes_received);
            if( (res = sendframe(msg, fd,5)) ==-1){
                return -1;
            }
            printf("DISC frame sent, %d bytes\n", res);
            STOP=TRUE;
        }
    }
    estado=Start;
    STOP=FALSE;
    bytes_received=0;
    unsigned char msg2rcv_2[5] = {FLAG, A_REC, UA, A_REC^UA, FLAG};
    while (STOP==FALSE) {       //loop for input 
        bytes_received += readframe(msg2rcv_2, fd,SUPERVISION);
        if (estado==MSTOP)
        {
            printf("UA frame received, %d bytes received\n", bytes_received);
            STOP=TRUE;
        }
    }
    return fd;
}

//////////END OF AUXILIARY FUNCTIONS//////////


int llopen(linkLayer Parameters){
    (void) signal(SIGALRM, resend); // Install alarm
    tries = Parameters.numTries;
    timeout = Parameters.timeOut; 
    if((fd=open_connection(Parameters))==-1)
    {
        printf("Error establishing connection!\n");
        return -1;
    }
    if (Parameters.role == TRANSMITTER)
    {
        if((fd=open_transmitter(fd, Parameters))==-1){
            printf("Error writing SET frame!\n");
            return -1;
        }
        return fd;
    }
    if (Parameters.role == RECEIVER)
    {
        if((fd=open_receiver(fd, Parameters))==-1){
            return -1;
        }
        return fd;
    }
    else
        return -1; 
}

int llwrite(char* buf, int bufSize)
{
    printf("=============================================\n");
    unsigned char buffer[MAX_PAYLOAD_SIZE];
    unsigned char codes[2];//[0] for RR and [1] for REJ 
    int info_frame_size;
    if (sequence==0){
        info_frame_size=build_info_frame(buf, bufSize, C_0);
        codes[0]=RR_1;
        codes[1]=REJ_1;
    }
    else if(sequence==1)
    {
        info_frame_size=build_info_frame(buf, bufSize, C_1);
        codes[0]=RR_0;
        codes[1]=REJ_0;
    }
    
    
    int bytes_received=0, res, bytes_sent;
    flag=1;
    end=0;  
    if ((bytes_sent = sendframe(I_frame, fd,info_frame_size))==-1)
    {
       return -1;
    }
    estado = Start;
    n_retries=1;
    retry=0;
    STOP =FALSE;
    printf("Information frame sent, %d bytes\n", bytes_sent);
    while (STOP==FALSE)
    {
        if (flag)
            alarm(timeout);
        if((res = readframe(codes, fd, 2))>0)  /* returns after 1 chars have been input */
        {
            bytes_received += res;
            alarm(0);
            flag=1;
        }
        if(end)
        {
            flag=0;
            alarm(0);
            STOP=TRUE;
            printf("Number of retries exceeded couldn't receive response frame\n");
            return -1;
        }
        
        if (flag)
        {
            if (retry)
            {
                if ((res = sendframe(I_frame, fd, info_frame_size))==-1)
                {
                    return -1;
                }
                printf("Timeout: Information frame sent, %d bytes (try: %d/%d)\n", res, n_retries, tries);
                retry=0;
            }
            alarm(timeout);
            flag=0;
        }

        if (estado==MSTOP)
        {
            if(right_code==1)
            {
                alarm(0);
                STOP=TRUE;
            }
            if (right_code==2)
            {
                STOP=FALSE;
            }
        }
    }

    if (sequence==0)
        sequence=1;
    else if (sequence==1)
        sequence=0;
    else
        return -1;

    return bytes_sent-6;
}

int llread(char* packet)
{
    printf("=============================================\n");
    i_byte=0;
    unsigned char control_byte[2]={C_0, C_1}, response_byte;
    int bytes_received=0, res, bytes_read=0;
    STOP=FALSE;
    estado = Start;

    while (STOP==FALSE)
    {
        if((res=readframe(control_byte, fd, INFO)>0)){
            bytes_received +=res;
        }
        if (estado==MSTOP)
            STOP=TRUE;
    }
    int framesize;
    if ((framesize=bytedestuffing())<0){
        return -1;
    }
    
    printf("Information frame received:\n>Bytes received: %d\n>Frame size after destuffing %d\n", bytes_received, framesize);
    
    unsigned char BCC2=I_frame[4];
    for (int i = 5; i < framesize-2; i++)
    {
        BCC2=BCC2^I_frame[i];
    }
    int control_sequence;
    if (I_frame[2]==C_0){
            control_sequence=0;
        }
        else if (I_frame[2]==C_1){
            control_sequence=1;
        }
    
    
    if (I_frame[framesize-2]==BCC2){
            if(control_sequence!=sequence){
                if(control_sequence==0){
                    sequence=0;
                    response_byte = REJ_1;
                }
                else{
                    sequence=1;
                    response_byte = REJ_0;
                }
            }
            else
            {
                for (int i = 4; i < framesize-2; i++)
                {
                    packet[i-4] = I_frame[i];
                    bytes_read++;
                }
                if(control_sequence==0){
                    sequence=1;
                    response_byte = RR_1;
                }
                else{
                    sequence=0;
                    response_byte = RR_0;
                }
            }
        }
        else{
            if (control_sequence==sequence)
            {
            if(control_sequence==0){
                    sequence=1;
                    response_byte = REJ_1;
                }
                else{
                    sequence=0;
                    response_byte = REJ_0;
                }
            } 
            else{
                if(control_sequence==0){
                    sequence=1;
                    response_byte = RR_1;
                }
                else{
                    sequence=0;
                    response_byte = RR_0;
                }
            }
        }
    unsigned char response_msg[5]={FLAG, A_TRAN, response_byte, response_byte^A_TRAN,FLAG};
    if((res=sendframe(response_msg, fd, 5))==-1){
        return -1;
    }
    printf("Response message sent\n");
    return bytes_read;
}

int llclose(linkLayer connectionParameters, int showStatistics){
    
    if (connectionParameters.role == TRANSMITTER)
    {
        if((fd=close_transmitter(fd, connectionParameters))==-1){
            printf("Error writing DISC/UA frame!\n");
            return -1;
        }
        return fd;
    }
    if (connectionParameters.role == RECEIVER)
    {
        if((fd=close_receiver(fd, connectionParameters))==-1){
            printf("Error writing/receiving DISC frame!\n");
            return -1;
        }
        return fd;
    }

    if ( tcsetattr(fd,TCSANOW,&oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }
    close(fd);  
    return 1;
}

