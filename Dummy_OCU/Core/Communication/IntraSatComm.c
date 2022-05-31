#include <stdint.h>
#include <string.h>
#include "IntraSatComm.h"

uint8_t header[4];
//packet [header 4 byte,id,cmd,len,arg,crc32 4 bytes]

/*
*********************************************************************************************
* @brief Initialize the intra sat communication system
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void PacketInit(void)
{
    //initialize buffer in big endian byte array
    header[3] = (HEADER & 0x000000ff);
    header[2] = (HEADER & 0x0000ff00) >> 8;
    header[1] = (HEADER & 0x00ff0000) >> 16;
    header[0] = (HEADER & 0xff000000) >> 24;
}
/*
*********************************************************************************************
* @brief Assemble the data packet to be transmitted to other microcontrollers
*********************************************************************************************
* @param[input]      id: ID of the target device, cmd: command number, *arg pointer to buffer containing the cmd argument, argSize: size of the argument buffer, *buffer: pointer to the packet buffer
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void PacketAssemble(uint8_t id, uint8_t cmd, uint8_t *arg, uint8_t argSize, uint8_t *buffer)
{
    //Add header to the buffer
    memcpy(buffer, header, sizeof(header));
    //Add id, cmd and argsize to the buffer
    buffer[4] = id;
    buffer[5] = cmd;
    buffer[6] = argSize;
    uint8_t lastPos;

    //check if command has an argument and add them to array
    if (argSize)
    {
        for (int i = 0; i < argSize; i++)
        {
            buffer[7 + i] = arg[i];
            lastPos = 7 + i;
        }
    }
    else
    {
        lastPos = 7;
    }
    //calculate CRC
    uint8_t tempData[lastPos+1];
    uint8_t CRC[4];
    memcpy(tempData, buffer, sizeof(tempData));
    uint32_t calcCRC = crc32(0, tempData, sizeof(tempData));

 
    //add crc in big endian at the end of the buffer
    CRC[3] = (calcCRC & 0x000000ff);
    CRC[2] = (calcCRC & 0x0000ff00) >> 8;
    CRC[1] = (calcCRC & 0x00ff0000) >> 16;
    CRC[0] = (calcCRC & 0xff000000) >> 24;
    memcpy(buffer+lastPos+1,CRC,sizeof(CRC));
}
/*
*********************************************************************************************
* @brief Assemble the data packet to be transmitted to other microcontrollers
*********************************************************************************************
* @param[input]      *id: pointer to ID variable, *cmd: pointer to cmd, *len: pointer to len, *arg pointer to buffer containing the cmd argument, *buffer: pointer to the target packet buffer, bufferSize: size of the buffer
* @param[output]     none
* @return            packet disassemble status: DEFRAME_OK = ok, DEFRAME_INVALID_HEADER = can't find header in the buffer, DEFRAME_INVALID_CRC = crc doesn't match
* @note              none
*********************************************************************************************
*/
deframeReturn PacketDisassemble(uint8_t *id, uint8_t *len, uint8_t *cmd, uint8_t *arg, uint8_t *buffer, int bufferSize)
{
    /*Search for header*/
    int pos = -1;
    for (int i = 0; i < bufferSize; i++)
    {
        if ((buffer[i] == header[0]) & (bufferSize - i >= MINPKTLEN))//find a start of header before length of message - 4
        {
            uint8_t cmpHeader[4];
            memcpy(cmpHeader, buffer + i, 4);
            if (!memcmp(cmpHeader, header, 4))
            {
                pos = i;//set the position of the start of the message
            }
        }
    }
    if (pos == -1)//cant find header
    {
        return DEFRAME_INVALID_HEADER;
    }
    else
    {
        //CRC verification

        uint32_t givenCRC;
        uint8_t tempCRC[4];
        uint8_t actualLen = buffer[pos + 6] + 1 + 1 + 1+ 4;
        memcpy(tempCRC,buffer+(pos+actualLen),4);//get CRC in the packet
        givenCRC=(tempCRC[0]<<24)|(tempCRC[1]<<16)|(tempCRC[2]<<8)|(tempCRC[3]);//make the CRC in uint32 form
        
        //copy everything in the packet except the crc (the last 4 bytes)
        uint8_t tempData[actualLen];
        memcpy(tempData,buffer+pos,actualLen);

        uint32_t calcCRC = crc32(0,tempData,sizeof(tempData));//calculate the checksum crc32 of the copied message
        if (calcCRC!=givenCRC){
            return DEFRAME_INVALID_CRC;//return because crc doesnt match
        }

        *id = buffer[pos + 4];
        *len = buffer[pos+6];
        *cmd = buffer[pos + 5];

        if (*len != 0)//len !=0 means there is a argument to the command
        {
            memcpy(arg, buffer + pos + 7, *len );
        }
        return DEFRAME_OK;//return with ok
    }

}
