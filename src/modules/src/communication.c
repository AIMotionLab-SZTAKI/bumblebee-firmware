/**
 * This code is the work of Peter Brezovcsik, who spent his summer internship
 * at the AIMotionLab of HUN-REN SZTAKI
 */

#define DEBUG_MODULE "COMMUNICATION"

#include <math.h>
#include "FreeRTOS.h"

#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "motors.h"
#include "pm.h"
#include "platform.h"

#include "stabilizer.h"

#include "event_groups.h"
#include "supervisor.h"

#include "quatcompress.h"
#include "statsCnt.h"
#include "static_mem.h"
#include "rateSupervisor.h"
#include "stream_buffer.h"

#include "communication.h"
#include "uart2.h"
#include <stdarg.h>


#define BAUD_RATE 921600

#define QUEUE_LENGTH 1
#define QUEUE_ITEM_SIZE (sizeof(uart_packet))


static bool isInit;
static bool shutdownTransport = false;

const static uint8_t SYNC_BYTE = 0xA5;
const static uint8_t START_BYTE = 0xFF;
const static uint8_t DEFAULT_BYTE = 0x00;

static SemaphoreHandle_t pckDataMutex;
static StaticSemaphore_t pckDataMutexBuffer;

static xQueueHandle txQueue;
static xQueueHandle rxQueue;

CommState commState = SYNC;

static int comm_timeout = 20;

STATIC_MEM_TASK_ALLOC(communicationTask, COMMUNICATION_TASK_STACKSIZE);

static void communicationTask(void* param);


/**
 * @brief Calculates the CRC (Cyclic Redundancy Check) for a given UART packet.
 *
 * This function computes the CRC by performing an XOR operation on each byte
 * of the packet's payload.
 *
 * @param packet Pointer to the UART packet whose CRC is to be calculated.
 * @return The calculated CRC value as an 8-bit unsigned integer (uint8_t).
 */
static uint8_t calcCrc(const uart_packet* packet) {
  const uint8_t* start = (const uint8_t*) packet->payload;
  
  const uint8_t* end = &packet->payload[packet->payloadLength];

  uint8_t crc = 0;
  for (const uint8_t* p = start; p < end; p++) {
    crc ^= *p;
  }
  return crc;
}




void handleControlDataPacket(uart_packet *packet) {
  //DEBUG_PRINT("Service type: [CONTROL]\n");
}



void handleTrajectoryDataPacket(uart_packet *packet) {
  //DEBUG_PRINT("Service type: [TRAJECTORY]\n");
}



/**
 * @brief Initializes the communication module.
 *
 * This function sets up the UART communication, creates necessary mutexes and queues,
 * and starts the communication task. It ensures that the initialization is performed
 * only once.
 *
 * - Initializes UART with a predefined baud rate.
 * - Creates a static mutex for packet data protection.
 * - Creates transmission and reception queues.
 * - Asserts the successful creation of mutex and queues.
 * - Starts the communication task.
 *
 * If the initialization has already been performed, the function returns immediately.
 */
void communicationInit() {

  if(isInit)
    return;
  
  uart2Init(BAUD_RATE);
  
  pckDataMutex = xSemaphoreCreateMutexStatic(&pckDataMutexBuffer);
  ASSERT(pckDataMutex);

  txQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
  rxQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);

  configASSERT(txQueue);
  configASSERT(rxQueue);

  if (txQueue == NULL) {
    DEBUG_PRINT("Queue could not be initialized!");
  }

  STATIC_MEM_TASK_CREATE(communicationTask, communicationTask, COMMUNICATION_TASK_NAME, NULL, COMMUNICATION_TASK_PRI);

  isInit = true;
}



/**
 * @brief Tests if the communication module has been initialized.
 *
 * This function checks the initialization status of the communication module.
 *
 * @return true if the communication module is initialized, false otherwise.
 */
bool communicationTest(void) {
  if (isInit) {
    return true;
  }
  return false;
}


/**
 * @brief Communication task for handling UART communication.
 *
 * This function runs as a task in the system, managing the communication
 * over UART. It waits for the system to be fully started before entering
 * the communication loop. The task handles synchronization, sending and
 * receiving packets, and processing received data based on the service type.
 *
 * - Waits for the system to be fully started.
 * - Enters a loop to handle communication until shutdown is requested.
 * - Manages synchronization by sending and receiving sync bytes.
 * - Sends data packets and waits for responses.
 * - Processes received packets, including checking CRC and handling payloads.
 * - Handles different types of service packets (control, trajectory).

 */
static void communicationTask(void* param)
{
  //Wait for the system to be fully started to start communication loop
  systemWaitStart();


  uart_packet txPacket;
  uart_packet rxPacket;


  uint8_t syncBuffer = DEFAULT_BYTE;
  

  while(shutdownTransport == false) {

    if (commState == SYNC) {

      //DEBUG_PRINT("Sending sync bytes\n");
      uart2SendData(sizeof(SYNC_BYTE), (uint8_t *) &SYNC_BYTE);
      uart2GetDataWithTimeout(1, &syncBuffer, M2T(comm_timeout));

      if (syncBuffer == SYNC_BYTE) {
        commState = CONNECTED;
        //DEBUG_PRINT("Comm state: [CONNECTED]\n");
      }
    }


    
    if (xQueueReceive(txQueue, &txPacket, portMAX_DELAY) == pdTRUE && commState == CONNECTED) {

      //TickType_t timeStart = xTaskGetTickCount();
      //uart2SendData((uint32_t) MAX_UART_BUFFER_SIZE, (uint8_t *) &txPacket);

      // This only sends the data that is necessery, the first (payloadLength + PACKET_META_LENGTH) bytes of the txPacket
      uart2SendData((uint32_t) txPacket.payloadLength + PACKET_META_LENGTH, (uint8_t *) &txPacket);
      rxPacket.start = DEFAULT_BYTE;

      // Read start byte
      uart2GetDataWithTimeout(1, &rxPacket.start, M2T(comm_timeout));
      

      if (rxPacket.start == START_BYTE) {
        uart2GetDataWithTimeout(1, &rxPacket.serviceType, M2T(comm_timeout));
        uart2GetDataWithTimeout(1, &rxPacket.payloadLength, M2T(comm_timeout));
      }
      else if (rxPacket.start == DEFAULT_BYTE) {
        DEBUG_PRINT("Bad CRC\n");
        syncBuffer = DEFAULT_BYTE;
        commState = SYNC;
      }
      else {
        DEBUG_PRINT("Start byte is %X\n", rxPacket.start);
        syncBuffer = DEFAULT_BYTE;
        commState = SYNC;
      }
      
      if (rxPacket.payloadLength != 0 && commState == CONNECTED) {

        // Get payload
        uart2GetDataWithTimeout((uint8_t) rxPacket.payloadLength, (uint8_t *) &rxPacket.payload, M2T(comm_timeout));

        // Read CRC byte and check whether it is correct
        uart2GetDataWithTimeout(1, &rxPacket.crc, M2T(comm_timeout));
        uint8_t calculatedCRC = calcCrc(&rxPacket);

        if (rxPacket.crc != calculatedCRC) {
          DEBUG_PRINT("Bad CRC.\n");
          syncBuffer = DEFAULT_BYTE;
          commState = SYNC;
        }

        // Handle payload based on the service type
        switch (rxPacket.serviceType) {
        case CONTROL_PACKET:
          handleControlDataPacket(&rxPacket);
          break;

        case TRAJECTORY_PACKET:
          handleTrajectoryDataPacket(&rxPacket);
          break;

        case FORWARDED_CONTROL_PACKET:
          // we handle it later, todo: refactor other parts of the code accordingly
          break;

        default:
          DEBUG_PRINT("Unknown service type.\n");
          commState = SYNC;
          syncBuffer = DEFAULT_BYTE;
          break;
        }

        if (rxQueue) {
          xQueueSend(rxQueue, &rxPacket, M2T(comm_timeout));
        }

      }
      else {
        syncBuffer = DEFAULT_BYTE;
        commState = SYNC;
      }
      //TickType_t timeEnd = xTaskGetTickCount();
      //DEBUG_PRINT("Transaction time in ticks: %lu\n", (timeEnd - timeStart));
    }
  }
}



// Control packet payload contains a float and a state
void createControlPacket(uart_packet* packet, va_list* args) {
  float* thrust = va_arg(*args, float*);
  const state_t* state = va_arg(*args, state_t*);

  ASSERT(thrust != NULL && state != NULL);

  uint8_t payloadLength = sizeof(*thrust) + sizeof(*state);
  ASSERT(payloadLength <= MAX_PAYLOAD_LENGTH);

  packet->start = START_BYTE;  
  packet->serviceType = CONTROL_PACKET;
  packet->payloadLength = payloadLength;

  unsigned long ptr = 0;

  memcpy(&packet->payload[ptr], thrust, sizeof(*thrust));
  ptr += sizeof(*thrust);

  memcpy(&packet->payload[ptr], state, sizeof(*state));
  
  // Put the crc byte at the end of the payload.
  // This is necessary to ensure that only the required amount of data is sent through the UART.
  packet->payload[packet->payloadLength] = calcCrc(packet);
    
}

// Create trajectory packet
// As of now this is only creates a dummy packet with three floats
void createTrajectoryPacket(uart_packet* packet, va_list* args) {
  float* dummy_float_1 = va_arg(*args, float*);
  float* dummy_float_2 = va_arg(*args, float*);
  float* dummy_float_3 = va_arg(*args, float*);

  ASSERT(dummy_float_1 != NULL && dummy_float_2 != NULL && dummy_float_3 != NULL);

  uint8_t payloadLength = sizeof(*dummy_float_1) + sizeof(*dummy_float_2) + sizeof(*dummy_float_3);
  ASSERT(payloadLength <= MAX_PAYLOAD_LENGTH);

  packet->start = START_BYTE;
  packet->serviceType = TRAJECTORY_PACKET;
  packet->payloadLength = payloadLength;

  unsigned long ptr = 0;

  memcpy(&packet->payload[ptr], dummy_float_1, sizeof(*dummy_float_1));
  ptr += sizeof(*dummy_float_1);

  memcpy(&packet->payload[ptr], dummy_float_2, sizeof(*dummy_float_2));
  ptr += sizeof(*dummy_float_2);

  memcpy(&packet->payload[ptr], dummy_float_3, sizeof(*dummy_float_3));

  packet->payload[packet->payloadLength] = calcCrc(packet);
}

// Create a packet that is sent back when controls are forwarded from a PC by a Raspberry Pi
// As of now this only creates a dummy packet with three floats
void createForwardPacket(uart_packet* packet, va_list* args) {
  float* dummy_float_1 = va_arg(*args, float*);
  float* dummy_float_2 = va_arg(*args, float*);
  float* dummy_float_3 = va_arg(*args, float*);

  ASSERT(dummy_float_1 != NULL && dummy_float_2 != NULL && dummy_float_3 != NULL);

  uint8_t payloadLength = sizeof(*dummy_float_1) + sizeof(*dummy_float_2) + sizeof(*dummy_float_3);
  ASSERT(payloadLength <= MAX_PAYLOAD_LENGTH);

  packet->start = START_BYTE;
  packet->serviceType = FORWARDED_CONTROL_PACKET;
  packet->payloadLength = payloadLength;

  unsigned long ptr = 0;

  memcpy(&packet->payload[ptr], dummy_float_1, sizeof(*dummy_float_1));
  ptr += sizeof(*dummy_float_1);

  memcpy(&packet->payload[ptr], dummy_float_2, sizeof(*dummy_float_2));
  ptr += sizeof(*dummy_float_2);

  memcpy(&packet->payload[ptr], dummy_float_3, sizeof(*dummy_float_3));

  packet->payload[packet->payloadLength] = calcCrc(packet);
}



void sendDataUART(const char *format, ...) {

  ASSERT(isInit == true && shutdownTransport == false);

  xSemaphoreTake(pckDataMutex, portMAX_DELAY);

  va_list args;
  va_start(args, format);

  uart_packet packet;

  while (*format != '\0') {
    switch (*format++) {
      case 'C': { 
        // Control (thrust, state) 
        createControlPacket(&packet, &args);
        break;
      }
      case 'T': {
        // Trajectory (dummy, 3xfloat)
        createTrajectoryPacket(&packet, &args);
        break;
      }
      case 'F': { 
        // Forward (currently dummy values) 
        createForwardPacket(&packet, &args);
        break;
      }
      default:
        break;
    } 
  }

  xSemaphoreGive(pckDataMutex);

  if (txQueue) {
    xQueueSend(txQueue, &packet, portMAX_DELAY);
  }

  va_end(args);
}


 

bool receiveDataUART(uart_packet *packet) {
  ASSERT(isInit == true && shutdownTransport == false);

  if (commState == CONNECTED) {
     static uart_packet rxUartPacket;

    if (xQueueReceive(rxQueue, &rxUartPacket, M2T(comm_timeout)) == pdTRUE) {
      xSemaphoreTake(pckDataMutex, portMAX_DELAY);

      packet->start = rxUartPacket.start;
      packet->serviceType = rxUartPacket.serviceType;
      packet->payloadLength = rxUartPacket.payloadLength;
      memcpy(packet->payload, &rxUartPacket.payload, (size_t) MAX_PAYLOAD_LENGTH);
      packet->crc = rxUartPacket.crc;

      xSemaphoreGive(pckDataMutex);
      return true;
    }
    return false;
  }
  return false;
}


void handle_control_packet(uart_packet *packet, float* thrustDesired, float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired) {
  unsigned long ptr = 0;

  memcpy(thrustDesired, &packet->payload[ptr], sizeof(*thrustDesired));
  ptr += sizeof(*thrustDesired);
  ptr += 4; // timestamp sent as an integer
  memcpy(rollRateDesired, &packet->payload[ptr], sizeof(*rollRateDesired));
  ptr += sizeof(*rollRateDesired);
  memcpy(pitchRateDesired, &packet->payload[ptr], sizeof(*pitchRateDesired));
  ptr += sizeof(*pitchRateDesired);
  memcpy(yawRateDesired, &packet->payload[ptr], sizeof(*yawRateDesired));
}

void handle_forwarded_packet(uart_packet *packet, float* thrustDesired, float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired) {
  unsigned long ptr = 0;

  ptr += sizeof(*thrustDesired); // timestamp sent as a float, not used for now
  memcpy(thrustDesired, &packet->payload[ptr], sizeof(*thrustDesired));
  ptr += sizeof(*thrustDesired);
  memcpy(rollRateDesired, &packet->payload[ptr], sizeof(*rollRateDesired));
  ptr += sizeof(*rollRateDesired);
  memcpy(pitchRateDesired, &packet->payload[ptr], sizeof(*pitchRateDesired));
  ptr += sizeof(*pitchRateDesired);
  memcpy(yawRateDesired, &packet->payload[ptr], sizeof(*yawRateDesired));
}
