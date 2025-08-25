#ifndef DUCKMAV_H
#define DUCKMAV_H

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
// #include <ESPAsyncUDP.h>
#include <common/mavlink.h>

class MavlinkHandler
{
public:
    // Constructor
    MavlinkHandler();

    // Initialization methods for each interface
    void beginSerial(HardwareSerial *serial, long baudRate);
    void beginTcp(const char *host, int port);
    // void beginUdp(int localPort, const char *remoteHost, int remotePort);

    // Main loop for reading/writing
    void handleMavlink();

    // Methods to enable/disable interfaces
    void enableSerial(bool enable);
    void enableTcp(bool enable);
    // void enableUdp(bool enable);

    // MAVLink data handling
    void sendMessage(mavlink_message_t *msg);

private:
    // Configuration flags
    bool _serialEnabled;
    bool _tcpEnabled;
    bool _udpEnabled;

    // Serial
    HardwareSerial *_serialPort;

    // TCP
    AsyncClient *_tcpClient;
    IPAddress _tcpHost;
    int _tcpPort;

    // // UDP
    // AsyncUDP _udpSocket;
    // IPAddress _udpRemoteHost;
    // int _udpLocalPort;
    // int _udpRemotePort;

    // MAVLink parser state
    mavlink_status_t _mavlinkStatus;
    mavlink_message_t _receivedMessage;

    // Private helper functions
    void readSerial();
    void readTcp();
    // void readUdp();

    void sendSerial(const uint8_t *buffer, size_t len);
    void sendTcp(const uint8_t *buffer, size_t len);
    // void sendUdp(const uint8_t *buffer, size_t len);
};

#endif