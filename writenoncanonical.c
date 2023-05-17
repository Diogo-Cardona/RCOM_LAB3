/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>

//printf("var = 0x%02x\n", (unsigned int)(buf[0]&0xFF));

#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyS1"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

///////////////CODIGO ALTERADO/////////////////  

unsigned char msg[5] = {0x5c, 0x01, 0x03, (0x01)^(0x03), 0x5c}; //msg a enviar
int flag=1, fd,c, res, tentativas=1;

volatile int STOP=FALSE;

typedef enum{
    Start,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    MSTOP
}Estados; //todos os estados necessarios para a máquina de estados

Estados estado = Start; //estado inicial da maquina de estados

void state_machine(unsigned char msg)
{
    unsigned char codigo[5] = {0x5c, 0x03, 0x07, (0x03)^(0x07), 0x5c};
    switch(estado)
    {
    case Start:
        if (msg==codigo[0])
            estado = FLAG_RCV;
        break;
    case FLAG_RCV:
        if (msg==codigo[0])
            break;
        if (msg==codigo[1])
            estado = A_RCV;
        else
            estado = Start;
        break;
    case A_RCV:
        if(msg==codigo[0]){
            estado = FLAG_RCV;
        break;
        }
        if(msg==codigo[2])
            estado = C_RCV;
        else
            estado = Start;
        break;
    case C_RCV:
        if(msg==codigo[0])
            estado = FLAG_RCV;
        if(msg==codigo[3])
            estado = BCC_OK;
        else
            estado = Start;
        break;
    case BCC_OK:
        if(msg==codigo[4])
            estado=MSTOP;
        else
            estado = Start;
        break;
    default:
        break;
    }
}

void resend()// atende alarme
{
    tentativas++;
    printf("Attempt %d\n", tentativas);
    flag=1;
    res = write(fd,msg,5);
    printf("%d bytes sent\n", res);
}



///////////////FIM DE CODIGO ALTERADO/////////////////


int main(int argc, char** argv)
{

    struct termios oldtio,newtio;
    char buf[255];
    int i, sum = 0, speed = 0;

    if ( (argc < 2) ||
         ((strcmp("/dev/ttyS0", argv[1])!=0) &&
          (strcmp("/dev/ttyS1", argv[1])!=0) )) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
        exit(1);
    }


    /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
    */


    fd = open(argv[1], O_RDWR | O_NOCTTY );
    if (fd < 0) { perror(argv[1]); exit(-1); }

    if ( tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
        perror("tcgetattr");
        exit(-1);
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 0.1;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 0;   /* blocking read until 5 chars received */

    /*
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a
    leitura do(s) próximo(s) caracter(es)
    */
    tcflush(fd, TCIOFLUSH);
    
    sleep(1);

    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    ///////////////CODIGO ALTERADO/////////////////  

    (void) signal(SIGALRM, resend);  // instala rotina que atende interrupcao
  
    res = write(fd,msg,5);
    printf("%d bytes sent\n", res);
    
    int bytes_received;
    while(STOP==FALSE ){
        res=read(fd, buf, 1);
        if (res>=1)
            bytes_received+=res;
        
        if (flag)
        {
            alarm(3); //ativa alarme
            flag=0; 
        }
        buf[res]=0;
        state_machine(buf[0]);
        
        if(tentativas >=3){
            flag=0;
            STOP=TRUE;
        }    
        if(estado==MSTOP)
        {
            printf("%d bytes received\n", bytes_received);
            STOP=TRUE;
        }    
    }
    
///////////////FIM DE CODIGO ALTERADO/////////////////

    sleep(1);
    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}
