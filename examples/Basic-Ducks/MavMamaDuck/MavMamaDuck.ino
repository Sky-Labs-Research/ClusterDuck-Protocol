/**
 * @file MavMamaDuck.ino
 * @brief Implements a MamaDuck using the ClusterDuck Protocol (CDP) and Mavlink.
 *
 * This example firmware periodically sends sensor health data (counter and free memory)
 * through a CDP mesh network. It also relays messages that it receives from other ducks
 * that has not seen yet.
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

#ifdef SERIAL_PORT_USBVIRTUAL
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

// --- Function Declarations ---
bool sendData(std::string message, byte topic = topics::mavlink);
bool runSensor(void *);

// --- Global Variables ---
MamaDuck duck;                       // CDP MamaDuck instance
auto timer = timer_create_default(); // Creating a timer with default settings
const int INTERVAL_MS = 10000;       // Interval in milliseconds between runSensor call
int counter = 1;                     // Counter for the sensor data
bool setupOK = false;                // Flag to check if setup is complete
MavlinkHandler mavlinkHandler;

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

  // Initialize MAVLink interfaces.
  // Use `Serial` for the console, and `Serial2` for MAVLink
  mavlinkHandler.beginSerial(&Serial2, 57600);
  Serial2.write("This is MAVLink serial bitches! We are live ;)");

  // Example TCP server: MAVProxy is often on 14550
  // Example: TCP client to MAVProxy
  Serial.print("Duck IP: ");
  Serial.println(IPAddress(CDPCFG_AP_IP1, CDPCFG_AP_IP2, CDPCFG_AP_IP3, CDPCFG_AP_IP4).toString().c_str());

  // TODO: Possibly make this dynamic for when devices join AP?
  // Temp connect to the next ip device on the ducks access point
  mavlinkHandler.beginTcpClient(IPAddress(CDPCFG_AP_IP1, CDPCFG_AP_IP2, CDPCFG_AP_IP3, CDPCFG_AP_IP4 + 1).toString().c_str(), 5760);

  // mavlinkHandler.beginTcpServer(5761);

  // Example UDP: local port 14550, sending to a GCS on 14550
  // mavlinkHandler.beginUdp(14550, "192.168.1.101", 14550);

  setupOK = true;
  Serial.println("[MAMA] Setup OK!");
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

  duck.run();
  mavlinkHandler.handleMavlink();
}

/**
 * @brief Sends the provided message as CDP packet to the mesh network.
 *
 * Encapsulates the message within a CDP topic and handles errors in transmission.
 *
 * @param message The payload data to send as a std::string
 * @param topic CDP topic. CDP topics can be found in CdpPacket.h (default: status)
 * @return true if data sent successfully, false otherwise
 */
bool sendData(std::string message, byte topic)
{
  bool sentOk = false;

  int err = duck.sendData(topic, message);
  if (err == DUCK_ERR_NONE)
  {
    sentOk = true;
  }
  if (!sentOk)
  {
    Serial.println(("[Link] Failed to send data. error = " + std::to_string(err)).c_str());
  }
  return sentOk;
}

#include "MavlinkFragment.h"
#include <cmath>
#include <cstring>

// Global variable to ensure unique message IDs
static uint16_t current_message_id = 0;

/**
 * @brief Splits a mavlink_message_t and sends it in fragments.
 * @param msg The MAVLink message to send.
 */
void splitAndSendMavlinkMessage(const mavlink_message_t &msg)
{
  // Serialize the MAVLink message into a byte buffer
  uint8_t send_buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(send_buffer, &msg);

  // Calculate the number of fragments needed
  uint8_t total_fragments = static_cast<uint8_t>(ceil((float)len / MAX_PAYLOAD_PER_FRAGMENT));

  // Create and send each fragment
  uint16_t message_id = current_message_id++;
  for (uint8_t i = 0; i < total_fragments; ++i)
  {
    MavlinkFragmentPacket packet;
    packet.message_id = message_id;
    packet.total_fragments = total_fragments;
    packet.fragment_sequence = i;

    // Calculate the starting point and size of the data chunk
    size_t offset = i * MAX_PAYLOAD_PER_FRAGMENT;
    size_t chunk_size = (i == total_fragments - 1) ? (len - offset) : MAX_PAYLOAD_PER_FRAGMENT;

    packet.payload_length = chunk_size;
    memcpy(packet.data, &send_buffer[offset], chunk_size);

    // --- SEND THE PACKET ---
    // Create a string that contains the exact size of the packet (header + payload).
    // This is binary-safe and won't be truncated by null bytes.
    size_t total_packet_size = MAVLINK_FRAGMENT_HEADER_SIZE + packet.payload_length;
    std::string packet_as_string(reinterpret_cast<char *>(&packet), total_packet_size);

    bool sent = sendData(packet_as_string, topics::mavlink);

    if (sent)
    {
      Serial.println("Sent fragment " + String(i + 1) + "/" + String(total_fragments));
    }
    else
    {
      Serial.println("Failed to send fragment " + String(i + 1) + "/" + String(total_fragments));
    }
  }
}