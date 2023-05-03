#include <stdio.h>
#include <stdlib.h>

typedef enum{
    Start,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP
}Estados,

//Variável para a máquina de estados 
Estados estado = Start;

void state_machine(unsigned char msg)
{
    unsigned char codigo[] = {0x5c, 0x01, 0x03, (0x01)^(0x03), 0x5c};
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
        if(msg==codigo[0])
            estado = FLAG_RCV;
        break;
        if(msg==codigo[3])
            estado = C_RCV;
        else
            estado = Start;
        break;
    case C_RCV:
        if(msg==codigo[0])
            estado = FLAG_RCV;
        if(msg==codigo[4])
            estado = BCC_OK;
        else
            estado = Start;
    case BCC_OK:
        if(msg==codigo[5])
            estado=STOP;
        else
            estado = Start;
    default:
        break;
    }
}







int main{
    unsigned char msg[5] = {0x5c, 0x03, 0x03, ui, 0x5c} 






}