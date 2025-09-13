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
    void beginTcpClient(const char *host, int port);
    void beginTcpServer(int port);
    // void beginUdp(int localPort, const char *remoteHost, int remotePort);

    // Main loop for reading/writing
    void handleMavlink();

    // Methods to enable/disable interfaces
    void enableSerial(bool enable);
    void enableTcpClient(bool enable);
    void enableTcpServer(bool enable);
    // void enableUdp(bool enable);

    // MAVLink data handling
    void sendMessage(mavlink_message_t *msg);
    // Forwarding across duck and MAVLINK
    void sendDuckMessage(mavlink_message_t *msg);

private:
    // Configuration flags

    // Serial
    bool _serialEnabled;
    HardwareSerial *_serialPort;
    void readSerial();
    void sendSerial(const uint8_t *buffer, size_t len);

    // TCP
    bool _tcpClientEnabled;
    AsyncClient *_tcpClient;
    IPAddress _tcpHost;
    int _tcpClientPort;
    void connectToTCPClient();
    void readTcpClient();
    void sendTcpClient(const uint8_t *buffer, size_t len);

    bool _tcpServerEnabled;
    AsyncServer *_tcpServer;
    std::vector<AsyncClient *> _serverClients;
    int _tcpServerPort;
    void connectToTCPServer();
    void handleNewClient(AsyncClient *client);
    void sendToAllTcpClients(const uint8_t *buffer, size_t len);

    // // UDP
    // AsyncUDP _udpSocket;
    // IPAddress _udpRemoteHost;
    // int _udpLocalPort;
    // int _udpRemotePort;
    // bool _udpEnabled;
    // void readUdp();
    // void sendUdp(const uint8_t *buffer, size_t len);

    // MAVLink parser state
    mavlink_status_t _mavlinkStatus;
    mavlink_message_t _receivedMessage;

    // Private helper functions
};

#endif