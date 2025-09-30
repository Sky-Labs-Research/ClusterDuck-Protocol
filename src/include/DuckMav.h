#ifndef DUCKMAV_H
#define DUCKMAV_H

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <AsyncUDP.h>
#include <common/mavlink.h>
#include <vector>
#include <map>
#include "freertos/ringbuf.h" // For efficient, thread-safe ring buffers
#include "freertos/semphr.h"  // For mutexes

// Forward declaration
class MavlinkHandler;

// Structure to hold context for each connected TCP server client
struct ServerClientContext
{
    AsyncClient *client;
    RingbufHandle_t rxBuffer;
    TaskHandle_t taskHandle;
    MavlinkHandler *handler; // Pointer back to the parent handler
    bool active;             // Flag to signal task termination
};

// Structure to hold context for each UDP "client"
struct UdpClientContext
{
    IPAddress ip;
    uint16_t port;
    uint64_t lastHeard; // Timestamp for tracking activity and timing out old clients
};

// Enum to identify MAVLink interfaces for forwarding logic
enum class MavlinkInterface
{
    NONE,
    MAVSERIAL,
    TCP_CLIENT,
    TCP_SERVER,
    UDP_SERVER,
    LORA
};

class MavlinkHandler
{
public:
    MavlinkHandler();
    ~MavlinkHandler();

    // Initialization Methods
    void beginSerial(HardwareSerial *serial, long baudRate);
    void beginTcpClient(const char *host, int port);
    void beginTcpServer(int port);
    void beginUdpServer(int port);
    void beginMavlinkTasks(); // Starts the background parsing tasks

    // Data Handling (Sending/Forwarding)
    void sendMessage(const mavlink_message_t *msg, MavlinkInterface excludeInterface = MavlinkInterface::NONE);
    void sendDuckMessage(const mavlink_message_t *msg, MavlinkInterface excludeInterface = MavlinkInterface::NONE);

private:
    // Low-level sending methods
    void sendSerial(const uint8_t *buffer, size_t len);
    void sendTcpClient(const uint8_t *buffer, size_t len);
    void sendToAllTcpClients(const uint8_t *buffer, size_t len);
    void sendToAllUdpClients(const uint8_t *buffer, size_t len);

    // Connection management
    void connectToTCPClient();
    void handleNewClient(AsyncClient *client);
    void cleanupClient(AsyncClient *client);
    void cleanupUdpClients();

    // MAVLink parsing task functions (must be static for xTaskCreate)
    static void mavlinkSerialParseTask(void *pvParameters);
    static void mavlinkTcpClientParseTask(void *pvParameters);
    static void mavlinkTcpServerClientParseTask(void *pvParameters);
    static void mavlinkUdpParseTask(void *pvParameters);

    // --- Member Variables ---

    // Serial Interface
    bool _serialEnabled = false;
    HardwareSerial *_serialPort = nullptr;
    TaskHandle_t _serialTaskHandle = NULL;

    // TCP Client Interface
    bool _tcpClientEnabled = false;
    AsyncClient *_tcpClient = nullptr;
    IPAddress _tcpHost;
    int _tcpClientPort;
    RingbufHandle_t _tcpClientRxBuffer = NULL;
    TaskHandle_t _tcpClientTaskHandle = NULL;

    // TCP Server Interface
    bool _tcpServerEnabled = false;
    AsyncServer *_tcpServer = nullptr;
    int _tcpServerPort;
    // Map to hold context for each server client (client pointer -> context)
    std::map<AsyncClient *, ServerClientContext *> _serverClients;

    // UDP Server Interface
    bool _udpServerEnabled = false;
    AsyncUDP _udp;
    int _udpServerPort;
    // Map to hold context for each UDP client (IP:Port string -> context)
    std::map<String, UdpClientContext *> _udpClients;
    RingbufHandle_t _udpRxBuffer = NULL;
    TaskHandle_t _udpTaskHandle = NULL;

    // Mutexes for thread-safe access to client maps
    SemaphoreHandle_t _serverClientsMutex;
    SemaphoreHandle_t _udpClientsMutex;
};

#endif // DUCKMAV_H
