#include "DuckMav.h"

MavlinkHandler::MavlinkHandler() : _serialEnabled(false),
                                   _tcpEnabled(false),
                                   _udpEnabled(false),
                                   _serialPort(nullptr),
                                   _tcpClient(nullptr)
{
    // Initialize MAVLink parser
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
    if (status)
    {
        memset(status, 0, sizeof(mavlink_status_t));
    }
}

// Initialization Methods
void MavlinkHandler::beginSerial(HardwareSerial *serial, long baudRate)
{
#if defined(CDPCFG_MAV_RX) && defined(CDPCFG_MAV_TX)
    _serialPort = serial;
    _serialPort->begin(baudRate, SERIAL_8N1, CDPCFG_MAV_RX, CDPCFG_MAV_TX);
    _serialEnabled = true;
    Serial.println("Serial MAVLink initialized.");
#endif
}

void MavlinkHandler::beginTcp(const char *host, int port)
{
    _tcpHost.fromString(host);
    _tcpPort = port;
    _tcpClient = new AsyncClient();

    // The data is received in this callback.
    // The 'data' parameter is the buffer, and 'len' is the length.
    _tcpClient->onData([this](void *arg, AsyncClient *client, void *data, size_t len)
                       {
        const uint8_t* buffer = (const uint8_t*)data;
        for (size_t i = 0; i < len; ++i) {
            // Process each byte from the buffer
            if (mavlink_parse_char(MAVLINK_COMM_1, buffer[i], &_receivedMessage, &_mavlinkStatus)) {
                // MAVLink message received! Process it here.
                Serial.print("Received MAVLink message from TCP, ID: ");
                Serial.println(_receivedMessage.msgid);
                sendMessage(&_receivedMessage);
            }
        } });

    _tcpClient->onConnect([this](void *arg, AsyncClient *client)
                          { Serial.println("TCP MAVLink client connected!"); });
    _tcpClient->onDisconnect([this](void *arg, AsyncClient *client)
                             { Serial.println("TCP MAVLink client disconnected!"); });
    _tcpClient->connect(_tcpHost, _tcpPort);
    _tcpEnabled = true;
    Serial.println("TCP MAVLink client initialized.");
}

// The following function is now unnecessary and should be removed.
/*
void MavlinkHandler::readTcp() {
    // This function is no longer needed
}
*/

// void MavlinkHandler::beginTcp(const char *host, int port)
// {
//     _tcpHost.fromString(host);
//     _tcpPort = port;
//     _tcpClient = new AsyncClient();
//     _tcpClient->onData([this](void *arg, AsyncClient *client, void *data, size_t len)
//                        {
//         const uint8_t* buffer = (const uint8_t*)data;
//         for (size_t i = 0; i < len; ++i) {
//            readTcp(); // Process each byte
//         } });
//     _tcpClient->onConnect([this](void *arg, AsyncClient *client)
//                           { Serial.println("TCP MAVLink client connected!"); });
//     _tcpClient->onDisconnect([this](void *arg, AsyncClient *client)
//                              { Serial.println("TCP MAVLink client disconnected!"); });
//     _tcpClient->connect(_tcpHost, _tcpPort);
//     _tcpEnabled = true;
//     Serial.println("TCP MAVLink client initialized.");
// }

// void MavlinkHandler::beginUdp(int localPort, const char *remoteHost, int remotePort)
// {
//     _udpLocalPort = localPort;
//     _udpRemoteHost.fromString(remoteHost);
//     _udpRemotePort = remotePort;
//     if (_udpSocket.listen(_udpLocalPort))
//     {
//         _udpSocket.onPacket([this](AsyncUDPPacket packet)
//                             {
//             const uint8_t* buffer = packet.data();
//             for (size_t i = 0; i < packet.length(); ++i) {
//                 readUdp(); // Process each byte
//             } });
//         _udpEnabled = true;
//         Serial.println("UDP MAVLink listener initialized.");
//     }
// }

// Main handler loop
void MavlinkHandler::handleMavlink()
{
    if (_serialEnabled && _serialPort->available())
    {
        readSerial();
    }
    // TCP and UDP data is handled by async callbacks
}

// Data Handling (Reading)
void MavlinkHandler::readSerial()
{
    while (_serialPort->available())
    {
        uint8_t c = _serialPort->read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &_receivedMessage, &_mavlinkStatus))
        {
            // MAVLink message received! Process it here.
            Serial.print("Received MAVLink message ID: ");
            Serial.println(_receivedMessage.msgid);
            // Example: Forward message to other interfaces
            sendMessage(&_receivedMessage);
        }
    }
}

// void MavlinkHandler::readTcp()
// {
//     // This function is for processing the single byte from the async callback
//     // A more robust implementation would buffer the data
//     // For this example, let's assume we get one byte at a time
//     uint8_t c = _tcpClient->read();
//     if (mavlink_parse_char(MAVLINK_COMM_1, c, &_receivedMessage, &_mavlinkStatus))
//     {
//         // MAVLink message received! Process it here.
//         Serial.print("Received MAVLink message from TCP, ID: ");
//         Serial.println(_receivedMessage.msgid);
//         sendMessage(&_receivedMessage);
//     }
// }

// void MavlinkHandler::readUdp()
// {
//     // Similar to TCP, this is a simplified example
//     uint8_t c = _udpSocket.read();
//     if (mavlink_parse_char(MAVLINK_COMM_2, c, &_receivedMessage, &_mavlinkStatus))
//     {
//         // MAVLink message received! Process it here.
//         Serial.print("Received MAVLink message from UDP, ID: ");
//         Serial.println(_receivedMessage.msgid);
//         sendMessage(&_receivedMessage);
//     }
// }

// Data Handling (Sending)
void MavlinkHandler::sendMessage(mavlink_message_t *msg)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, msg);

    if (_serialEnabled)
    {
        sendSerial(buffer, len);
    }
    if (_tcpEnabled && _tcpClient->connected())
    {
        sendTcp(buffer, len);
    }
    // if (_udpEnabled)
    // {
    //     sendUdp(buffer, len);
    // }
}

void MavlinkHandler::sendSerial(const uint8_t *buffer, size_t len)
{
    _serialPort->write(buffer, len);
}

void MavlinkHandler::sendTcp(const uint8_t *buffer, size_t len)
{
    _tcpClient->write((const char *)buffer, len);
}

// void MavlinkHandler::sendUdp(const uint8_t *buffer, size_t len)
// {
//     _udpSocket.writeTo(buffer, len, _udpRemoteHost, _udpRemotePort);
// }

// Enable/Disable functions
void MavlinkHandler::enableSerial(bool enable)
{
    _serialEnabled = enable;
}

void MavlinkHandler::enableTcp(bool enable)
{
    _tcpEnabled = enable;
}

// void MavlinkHandler::enableUdp(bool enable)
// {
//     _udpEnabled = enable;
// }