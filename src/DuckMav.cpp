#include "DuckMav.h"
#include <include/cdpcfg.h>

// Forward declaration for the function in the main .ino file
void splitAndSendMavlinkMessage(const mavlink_message_t &msg);

MavlinkHandler::MavlinkHandler() : _serialEnabled(false),
                                   _tcpClientEnabled(false),
                                   _tcpServerEnabled(false),
                                   //    _udpEnabled(false),
                                   _serialPort(nullptr),
                                   _tcpClient(nullptr),
                                   _tcpServer(nullptr)
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

void MavlinkHandler::connectToTCPClient()
{
    bool result;
    result = _tcpClientEnabled && (_tcpClient->connected() || _tcpClient->connecting() || _tcpClient->disconnecting());
    if (result)
    {
        Serial.println("[MAMA] Already Connecting or Disconnecting to TCP Mavlink.");
    }
    else
    {
        Serial.println("[MAMA] Retrying connection to TCP Mavlink.");
        _tcpClient->connect(_tcpHost, _tcpClientPort);
    }
    // return result;
}

void MavlinkHandler::beginTcpClient(const char *host, int port)
{
    _tcpHost.fromString(host);
    _tcpClientPort = port;
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
                sendDuckMessage(&_receivedMessage);
            }
        } });

    _tcpClient->onConnect([this](void *arg, AsyncClient *client)
                          { Serial.println("TCP MAVLink client connected!"); });
    _tcpClient->onDisconnect([this](void *arg, AsyncClient *client)
                             { Serial.println("TCP MAVLink client disconnected!"); 
                                connectToTCPClient(); });
    _tcpClient->connect(_tcpHost, _tcpClientPort);
    _tcpClientEnabled = true;
    Serial.print("TCP MAVLink client initialized. Connecting to ");
    Serial.print(_tcpHost);
    Serial.print(":");
    Serial.println(_tcpClientPort);
}

void MavlinkHandler::beginTcpServer(int port)
{
    _tcpServerPort = port;
    _tcpServer = new AsyncServer(_tcpServerPort);
    _tcpServerEnabled = true;

    _tcpServer->onClient([this](void *arg, AsyncClient *client)
                         { handleNewClient(client); },
                         this);

    _tcpServer->begin();
    Serial.print("TCP MAVLink Server started on port ");
    Serial.println(_tcpServerPort);
}

void MavlinkHandler::handleNewClient(AsyncClient *client)
{
    Serial.printf("New TCP client connected to server from %s\n", client->remoteIP().toString().c_str());
    _serverClients.push_back(client);

    client->onData([this](void *arg, AsyncClient *c, void *data, size_t len)
                   {
        const uint8_t* buffer = (const uint8_t*)data;
        for (size_t i = 0; i < len; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_1, buffer[i], &_receivedMessage, &_mavlinkStatus)) {
                Serial.print("Received MAVLink message from TCP Server, ID: ");
                Serial.println(_receivedMessage.msgid);
                sendDuckMessage(&_receivedMessage);
            }
        } });

    client->onDisconnect([this](void *arg, AsyncClient *c)
                         {
        Serial.printf("TCP client %s disconnected from server\n", c->remoteIP().toString().c_str());
        _serverClients.erase(std::remove_if(_serverClients.begin(), _serverClients.end(), [c](AsyncClient *item) {
            return item == c;
        }), _serverClients.end()); });
}

// The following function is now unnecessary and should be removed.
/*
void MavlinkHandler::readTcpClient() {
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
//            readTcpClient(); // Process each byte
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
            Serial.print("Received MAVLink message from Serial, ID: ");
            Serial.println(_receivedMessage.msgid);
            // Example: Forward message to other interfaces
            sendDuckMessage(&_receivedMessage);
        }
    }
}

// void MavlinkHandler::readUdp()
// {
//     // Similar to TCP, this is a simplified example
//     uint8_t c = _udpSocket.read();
//     if (mavlink_parse_char(MAVLINK_COMM_2, c, &_receivedMessage, &_mavlinkStatus))
//     {
//         // MAVLink message received! Process it here.
//         Serial.print("Received MAVLink message from UDP, ID: ");
//         Serial.println(_receivedMessage.msgid);
//         sendDuckMessage(&_receivedMessage);
//     }
// }

// Data Handling (Sending)
void MavlinkHandler::sendMessage(mavlink_message_t *msg)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, msg);
    Serial.println("Sending Messages across enabled mavlink links");
    if (_serialEnabled)
    {
        sendSerial(buffer, len);
    }
    if (_tcpClientEnabled && _tcpClient->connected())
    {
        sendTcpClient(buffer, len);
    }
    if (_tcpServerEnabled)
    {
        sendToAllTcpClients(buffer, len);
    }
    // if (_udpEnabled)
    // {
    //     sendUdp(buffer, len);
    // }
}

void MavlinkHandler::sendDuckMessage(mavlink_message_t *msg)
{
    sendMessage(msg);
    // TODO: send out to ducks
    splitAndSendMavlinkMessage(*msg);
}

void MavlinkHandler::sendSerial(const uint8_t *buffer, size_t len)
{
    _serialPort->write(buffer, len);
}

void MavlinkHandler::sendTcpClient(const uint8_t *buffer, size_t len)
{
    _tcpClient->write((const char *)buffer, len);
}

void MavlinkHandler::sendToAllTcpClients(const uint8_t *buffer, size_t len)
{
    for (auto client : _serverClients)
    {
        if (client && client->connected())
        {
            client->write((const char *)buffer, len);
        }
    }
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

void MavlinkHandler::enableTcpClient(bool enable)
{
    _tcpClientEnabled = enable;
}

void MavlinkHandler::enableTcpServer(bool enable)
{
    _tcpServerEnabled = enable;
}

// void MavlinkHandler::enableUdp(bool enable)
// {
//     _udpEnabled = enable;
// }
