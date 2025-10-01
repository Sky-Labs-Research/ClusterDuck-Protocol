/**
 * @file MavMamaDuck.ino
 * @brief Implements a MamaDuck using the ClusterDuck Protocol (CDP) and Mavlink
 * with robust, multi-threaded MAVLink forwarding.
 *
 * This firmware forwards MAVLink packets between Serial, TCP, and a LoRa-based
 * CDP mesh network. It uses a multi-tasking architecture to handle high data rates
 *  packet loss.
 *
 * @date 2025-07-17
 */

#include <string>
#include <arduino-timer.h>
#include <CDP.h>
#include <checksum.h>
#include <common/mavlink.h>
#include "DuckMav.h"
#include "cdpcfg.h"
// added
#include "MavlinkFragment.h"
#include <cmath>
#include <cstring>
#include <stdlib.h>

#ifdef SERIAL_PORT_USBVIRTUAL
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

// --- Function Declarations ---
bool sendData(std::string message, byte topic = topics::mavlink);
bool runSensor(void *);
void splitAndSendMavlinkMessage(const mavlink_message_t &msg);
void drainLoRaQueueFromLoop();

// --- Global Variables ---
MavMamaDuck duck;                    // CDP MamaDuck instance
auto timer = timer_create_default(); // Creating a timer with default settings
const int INTERVAL_MS = 10000;       // Interval in milliseconds between runSensor call
int counter = 1;                     // Counter for the sensor data
bool setupOK = false;                // Flag to check if setup is complete
MavlinkHandler mavlinkHandler;
QueueHandle_t loraTxQueue = NULL;
static uint16_t current_message_id = 0;

/**
 * @brief Setup function to initialize the MamaDuck
 *
 * - Sets up the Duck device ID (exactly 8 bytes).
 * - Initializes MamaDuck using default configuration.
 * - Sets up periodic execution of sensor data transmissions.
 */
void setup()
{

  std::string deviceId("SLRMAMA2"); // MUST be 8 bytes and unique from other ducks
  std::array<byte, 8> devId;
  std::copy(deviceId.begin(), deviceId.end(), devId.begin());

  if (duck.setupWithDefaults(devId) != DUCK_ERR_NONE)
  {
    Serial.println("[MAMA] Failed to setup MamaDuck");
    return;
  }

  // timer.every(INTERVAL_MS, runSensor); // Triggers runSensor every INTERVAL_MS

  // TODO: Implement serial / tcp / udp mavlink connection

  // Example 1: Serial link to a flight controller on Serial2.
  mavlinkHandler.beginSerial(&Serial2, 57600);

  // Example 2: TCP client connecting to a flight controller TCP server.
  // This connects to the first device (e.g., a laptop) that joins the Duck's access point.
  // mavlinkHandler.beginTcpClient(IPAddress(CDPCFG_AP_IP1, CDPCFG_AP_IP2, CDPCFG_AP_IP3, CDPCFG_AP_IP4 + 1).toString().c_str(), 5760);

  // Example 3: TCP server for a GCS to connect to this Duck.
  // QGroundControl or MAVProxy can connect to this port.
  // mavlinkHandler.beginTcpServer(5761);

  // Example 3: UDP server for a GCS to connect to this Duck.
  // QGroundControl or MAVProxy can connect to this port.
  mavlinkHandler.beginUdpServer(14500);

  // After configuring interfaces, start the dedicated background parsing tasks.
  // This moves all high-frequency MAVLink processing off the main loop().
  mavlinkHandler.beginMavlinkTasks();

  // --- LoRa TX Queue Setup ---
  loraTxQueue = xQueueCreate(20, sizeof(MavlinkFragmentPacket *));
  if (!loraTxQueue)
  {
    Serial.println("[MAMA] ERROR: failed to create loraTxQueue");
  }
  else
  {
    Serial.println("[MAMA] loraTxQueue created");
  }

  setupOK = true;
  Serial.println("[MAMA] Setup OK! MAVLink tasks are running in the background.");

  Serial.printf("setup() running on core ");
  Serial.println(xPortGetCoreID());
}

/**
 * @brief Main loop runs continuously.
 *
 * Executes scheduled tasks and maintains Duck operation.
 */
void loop()
{
  if (!setupOK)
  {
    return;
  }
  timer.tick();

  // Run CDP state machine
  duck.run();

  // The main loop is now only responsible for draining the LoRa queue.
  // All other MAVLink parsing is handled by dedicated FreeRTOS tasks for high performance.
  drainLoRaQueueFromLoop();

  // A small delay to yield CPU time, preventing this loop from starving other tasks.
  vTaskDelay(pdMS_TO_TICKS(1));
}

// --- Data Sending Functions ---

/**
 * @brief Sends the provided message as CDP packet to the mesh network.
 */
bool sendData(std::string message, byte topic)
{
  int err = duck.sendData(topic, message);
  if (err != DUCK_ERR_NONE)
  {
    logmav_ln("[Link] Failed to send data. error = %d\n", err);
    return false;
  }
  return true;
}

/**
 * @brief Splits a mavlink_message_t and queues it for LoRa transmission.
 * This function is called from the MAVLink parsing tasks.
 */
void splitAndSendMavlinkMessage(const mavlink_message_t &msg)
{
  if (!loraTxQueue)
  {
    // This might happen if a MAVLink message arrives before the queue is ready.
    return;
  }

  // Serialize the MAVLink message into a byte buffer
  uint8_t send_buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(send_buffer, &msg);

  // Calculate the number of fragments needed
  uint8_t total_fragments = static_cast<uint8_t>(ceil((float)len / MAX_PAYLOAD_PER_FRAGMENT));

  // Create and send each fragment
  uint16_t message_id = current_message_id++;
  for (uint8_t i = 0; i < total_fragments; ++i)
  {
    MavlinkFragmentPacket *packetPtr = (MavlinkFragmentPacket *)malloc(sizeof(MavlinkFragmentPacket));
    if (!packetPtr)
    {
      logmav_ln("[LoRaTx] ERROR: malloc MavlinkFragmentPacket failed");
      return; // Out of memory
    }

    memset(packetPtr, 0, sizeof(MavlinkFragmentPacket));

    packetPtr->message_id = message_id;
    packetPtr->total_fragments = total_fragments;
    packetPtr->fragment_sequence = i;

    size_t offset = i * MAX_PAYLOAD_PER_FRAGMENT;
    size_t chunk_size = (i == total_fragments - 1) ? (len - offset) : MAX_PAYLOAD_PER_FRAGMENT;

    packetPtr->payload_length = chunk_size;
    memcpy(packetPtr->data, &send_buffer[offset], chunk_size);

    if (xQueueSend(loraTxQueue, &packetPtr, pdMS_TO_TICKS(50)) != pdPASS)
    {
      logmav_ln("[LoRaTx] Queue full, dropping fragment %u/%u (msgid=%u)\n",
                    packetPtr->fragment_sequence + 1, packetPtr->total_fragments, packetPtr->message_id);
      free(packetPtr); // Free memory if queue is full
    }
  }
}

/**
 * @brief Drains a few fragments from the LoRa queue each loop iteration.
 */
void drainLoRaQueueFromLoop()
{
  if (!loraTxQueue)
    return;

  const int MAX_PER_LOOP = 6; // How many fragments to send per loop iteration
  int handled = 0;

  MavlinkFragmentPacket *packetPtr = nullptr;
  while (handled < MAX_PER_LOOP && xQueueReceive(loraTxQueue, &packetPtr, 0) == pdPASS)
  {
    if (!packetPtr)
      continue;

    size_t total_packet_size = MAVLINK_FRAGMENT_HEADER_SIZE + packetPtr->payload_length;
    std::string packet_as_string(reinterpret_cast<char *>(packetPtr), total_packet_size);

    if (sendData(packet_as_string, topics::mavlink))
    {
      logmav("Sent fragment %u/%u (length=%u, msgid=%u)\n", packetPtr->fragment_sequence + 1, packetPtr->total_fragments, packetPtr->payload_length, packetPtr->message_id);
    }
    else
    {
      logmav_ln("Failed to send fragment %u/%u (length=%u, msgid=%u)\n", packetPtr->fragment_sequence + 1, packetPtr->total_fragments, packetPtr->payload_length, packetPtr->message_id);
    }

    free(packetPtr);
    handled++;
  }
}
