#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "IntraSatComm.h"
#define HEADER 0x4D555350
#define MINPKTLEN 11

//packet [header,id,len,cmd,data,crc32]

/*
int main() {
    
    PacketInit();
    uint8_t arg[] = {0xAA,0xFF,0xBA,0xFE};
    uint8_t buffer[15];
    PacketAssemble(0x11, 0x01, arg, sizeof(arg), buffer);
    
    uint8_t id, len, cmd;
    uint8_t argRet[4];
    // uint8_t buffer[15];
    deframeReturn retVal = PacketDisassemble(&id, &len, &cmd, argRet, buffer,sizeof(buffer));
    if(retVal == DEFRAME_OK){
        printf("\nretVal is %d id is %x len is %d cmd is %x\n",retVal, id, len, cmd);
        for (int i = 0;i<len;i++){
            printf("arg %d is %x\n",i, argRet[i]);
        }
    }else if (retVal == DEFRAME_INVALID_HEADER){
        printf("Can't find header");
    }else{
        printf("CRC doesnt match");
    }
    
    
    return 0;
}

*/
