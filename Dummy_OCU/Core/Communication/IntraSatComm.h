#define HEADER 0x4D555350
#define MINPKTLEN 11
#include "es_crc32.h"

typedef enum
{
    DEFRAME_OK,    // deframe successful
    DEFRAME_INVALID_HEADER,    // wrong header
    DEFRAME_INVALID_CRC,    // wrong crc
} deframeReturn;

void PacketInit(void);
void PacketAssemble(uint8_t id, uint8_t cmd, uint8_t* arg, uint8_t argSize, uint8_t* buffer);
deframeReturn PacketDisassemble(uint8_t* id, uint8_t* len, uint8_t* cmd, uint8_t* arg, uint8_t* buffer, int bufferSize);
