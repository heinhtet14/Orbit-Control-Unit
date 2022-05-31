To assemble or disassemble the package, call functions listed in IntraSatComm.h

IntraSatComm has es_crc32 and interger.h as dependencies.

See test.c for the example of how to use IntraSatComm library

To run test.c , compile test.c alongside IntraSatComm.c and es_crc32.c (gcc test.c IntraSatComm.c es_crc32.c)
 
Packet structure: 
Header 4 Bytes “musp”: [4d,55,53,50]
BUS ID 1 byte:
CMD ID 1 byte
Length of CMD argument 1 byte:
CMD argument x bytes depends on the command
Checksum CRC32 4 byte 

1 transaction: send two messages
Send message to determine size of the next message (handshake)
Send actual message

Example OBC-1 request angular rate (3x floats) from ACU

1st message
OBC-1 send (ACU receive)
[4d,55,53,50,01,A2,01,09,61,A4,0F,3B]

OBC-1 receive (ACU Send)
[4d,55,53,50,01,A2,01,0C,11,CE,FB,B4]

OBC knows that the next message to be sent to ACU must have argument size of 12 (0x0C = 12)

2nd message
OBC send (ACU receive)
[4d,55,53,50,01,09,0C,0,0,0,0,0,0,0,0,0,0,0,0,DC,BF,15,59]
OBC receive (ACU send)
[4d,55,53,50,01,09,0C,01,02,03,04,05,06,07,08,09,0A,0B,0C,DC,BF,15,59]

Note
1st message always have a fixed size of 12 bytes. Information on 1st message can be used to set up the buffer size of the second transaction
For cmd argument, use big endian for anything that is not 8 bit int


 