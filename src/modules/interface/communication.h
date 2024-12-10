#include <stdint.h>
#include "stabilizer_types.h"

#define MAX_UART_BUFFER_SIZE 200

#define PACKET_HEADER_LENGTH 3
#define PACKET_CRC_LENGTH 1
#define PACKET_META_LENGTH (PACKET_HEADER_LENGTH + PACKET_CRC_LENGTH)

#define MAX_PAYLOAD_LENGTH (MAX_UART_BUFFER_SIZE - PACKET_META_LENGTH)

#define CONTROL_PACKET 0x01
#define TRAJECTORY_PACKET 0x02
#define FORWARDED_CONTROL_PACKET 0x03


typedef enum {
  CONNECTED,
  SYNC,
} CommState;


typedef struct {
  uint8_t start;
  uint8_t serviceType;
  uint8_t payloadLength;
  uint8_t payload[MAX_PAYLOAD_LENGTH];
  uint8_t crc;
} __attribute__((packed)) uart_packet;


void communicationInit();

void sendDataUART(const char *format, ...);

bool receiveDataUART(uart_packet*);

bool communicationTest(void);

void handle_control_packet(uart_packet *packet, float* thrustDesired, float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired);

void handle_forwarded_packet(uart_packet *packet, float* thrustDesired, float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired);
