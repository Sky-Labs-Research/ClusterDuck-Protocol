#include "DuckMav.h"
#include "cdpcfg.h"

// External function from the main .ino file for sending LoRa fragments
extern void splitAndSendMavlinkMessage(const mavlink_message_t &msg);

// --- Constants ---
#define RING_BUFFER_SIZE 8192                // Bytes for each RX ring buffer. Increase if you see drops.
#define PARSE_TASK_STACK_SIZE 4096           // Stack size for parser tasks
#define PARSE_TASK_PRIORITY 3                // FreeRTOS task priority (higher number = higher priority)
#define PARSE_TASK_CORE 1                    // Pin tasks to a specific core (0 or 1) to improve performance (core 0 is dedicated to running wifi and things of that nature)
#define UDP_CLIENT_TIMEOUT_US (30 * 1000000) // 30 seconds in microseconds for UDP client timeout

// Helper to print MAVLink messages for debugging (optional)
void printMavlinkMessage(const mavlink_message_t *msg_ptr)
{
    // // This is useful for debugging but can be commented out to save CPU cycles
    // __mavlink_message msg = *msg_ptr;
    // switch (msg.msgid)
    // {
    // case MAVLINK_MSG_ID_HEARTBEAT:
    //     Serial.printf("[HEARTBEAT] sys:%u comp:%u\n", msg.sysid, msg.compid);
    //     break;
    // // Add other cases as needed
    // default:
    //     break;
    // }
}

MavlinkHandler::MavlinkHandler()
{
    _serverClientsMutex = xSemaphoreCreateMutex();
    _udpClientsMutex = xSemaphoreCreateMutex();
}

MavlinkHandler::~MavlinkHandler()
{
    // Safely clean up tasks and resources
    if (_serialTaskHandle)
        vTaskDelete(_serialTaskHandle);
    if (_tcpClientTaskHandle)
        vTaskDelete(_tcpClientTaskHandle);
    if (_udpTaskHandle)
        vTaskDelete(_udpTaskHandle);

    // TCP Server clients
    xSemaphoreTake(_serverClientsMutex, portMAX_DELAY);
    for (auto const &[client, context] : _serverClients)
    {
        context->active = false; // Signal task to stop
    }
    _serverClients.clear();
    xSemaphoreGive(_serverClientsMutex);
    vSemaphoreDelete(_serverClientsMutex);

    // UDP clients
    xSemaphoreTake(_udpClientsMutex, portMAX_DELAY);
    for (auto const &[key, context] : _udpClients)
    {
        delete context;
    }
    _udpClients.clear();
    xSemaphoreGive(_udpClientsMutex);
    vSemaphoreDelete(_udpClientsMutex);

    if (_tcpClientRxBuffer)
        vRingbufferDelete(_tcpClientRxBuffer);
    if (_udpRxBuffer)
        vRingbufferDelete(_udpRxBuffer);

    delete _tcpClient;
    delete _tcpServer;
    _udp.close();
}

// --- Initialization ---

void MavlinkHandler::beginSerial(HardwareSerial *serial, long baudRate)
{
#if defined(CDPCFG_MAV_RX) && defined(CDPCFG_MAV_TX)
    _serialPort = serial;
    _serialPort->begin(baudRate, SERIAL_8N1, CDPCFG_MAV_RX, CDPCFG_MAV_TX);
    _serialEnabled = true;
    logmav_ln("Serial MAVLink initialized.");
#endif
}

void MavlinkHandler::beginTcpClient(const char *host, int port)
{
    _tcpHost.fromString(host);
    _tcpClientPort = port;
    _tcpClient = new AsyncClient();
    _tcpClient->setNoDelay(true); // Disable Nagle's algorithm for low latency

    _tcpClientRxBuffer = xRingbufferCreate(RING_BUFFER_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (!_tcpClientRxBuffer)
    {
        logmav_ln("[ERROR] Failed to create TCP client RX ring buffer!");
        return;
    }

    _tcpClient->onData([this](void *arg, AsyncClient *client, void *data, size_t len)
                       {
        // Send data to the ring buffer. A small timeout allows for brief contention.
        // If the buffer is full (parser task is stalled), data will be dropped.
        if (xRingbufferSend(_tcpClientRxBuffer, data, len, pdMS_TO_TICKS(10)) != pdTRUE) {
            logmav_ln("[WARN] TCP client RX buffer overflow!");
        } });

    _tcpClient->onConnect([this](void *arg, AsyncClient *client)
                          { logmav_ln("TCP MAVLink client connected!"); });

    _tcpClient->onDisconnect([this](void *arg, AsyncClient *client)
                             {
        logmav_ln("TCP MAVLink client disconnected. Reconnecting in 2s...");
        // A simple timed reconnect to avoid spamming connection attempts
        vTaskDelay(pdMS_TO_TICKS(2000));
        connectToTCPClient(); });
    _tcpClient->onTimeout([this](void *arg, AsyncClient *client, uint32_t time)
                          {
          logmav_ln("TCP MAVLink client timed out. Reconnecting in 2s...");
          // A simple timed reconnect to avoid spamming connection attempts
          vTaskDelay(pdMS_TO_TICKS(2000));
          connectToTCPClient(); });
    connectToTCPClient();
    _tcpClientEnabled = true;
    logmav_ln("TCP MAVLink client initialized. Connecting to %s:%d\n", _tcpHost.toString().c_str(), _tcpClientPort);
}

void MavlinkHandler::connectToTCPClient()
{
    if (_tcpClientEnabled && !_tcpClient->connected() && !_tcpClient->connecting())
    {
        logmav_ln("Attempting TCP client connection...");
        _tcpClient->connect(_tcpHost, _tcpClientPort);
    }
    else
    {
        vTaskDelay(pdMS_TO_TICKS(2000));
        logmav_ln("Attempting TCP client connection... after 2 sec delay");
        _tcpClient->connect(_tcpHost, _tcpClientPort);
        // connectToTCPClient();
    }
}
// void MavlinkHandler::connectToTCPClient()
// {
//     bool result;
//     result = _tcpClientEnabled && (_tcpClient->connected() || _tcpClient->connecting() || _tcpClient->disconnecting());
//     if (result)
//     {
//         logmav_ln("[MAMA] Already Connecting or Disconnecting to TCP Mavlink.");
//     }
//     else
//     {
//         logmav_ln("[MAMA] Retrying connection to TCP Mavlink.");
//         _tcpClient->connect(_tcpHost, _tcpClientPort);
//     }
//     // return result;
// }

void MavlinkHandler::beginTcpServer(int port)
{
    _tcpServerPort = port;
    _tcpServer = new AsyncServer(_tcpServerPort);

    _tcpServer->onClient([this](void *arg, AsyncClient *client)
                         { handleNewClient(client); }, this);

    _tcpServer->begin();
    _tcpServerEnabled = true;
    logmav_ln("TCP MAVLink Server started on port %d\n", _tcpServerPort);
}

void MavlinkHandler::beginUdpServer(int port)
{
    _udpServerPort = port;
    if (_udp.listen(_udpServerPort))
    {
        _udpServerEnabled = true;
        _udpRxBuffer = xRingbufferCreate(RING_BUFFER_SIZE, RINGBUF_TYPE_BYTEBUF);
        if (!_udpRxBuffer)
        {
            logmav_ln("[ERROR] Failed to create UDP RX ring buffer!");
            _udpServerEnabled = false;
            return;
        }

        _udp.onPacket([this](AsyncUDPPacket &packet)
                      {
            String clientKey = packet.remoteIP().toString() + ":" + String(packet.remotePort());
            if (xSemaphoreTake(_udpClientsMutex, portMAX_DELAY) == pdTRUE) {
                auto it = _udpClients.find(clientKey);
                if (it != _udpClients.end()) {
                    // Client known, just update timestamp
                    it->second->lastHeard = esp_timer_get_time();
                } else {
                    // New client
                    logmav_ln("New UDP client connected from %s\n", clientKey.c_str());
                    UdpClientContext* context = new UdpClientContext{
                        .ip = packet.remoteIP(),
                        .port = packet.remotePort(),
                        .lastHeard = esp_timer_get_time()
                    };
                    _udpClients[clientKey] = context;
                }
                xSemaphoreGive(_udpClientsMutex);
            }

            if (xRingbufferSend(_udpRxBuffer, packet.data(), packet.length(), pdMS_TO_TICKS(10)) != pdTRUE) {
                 logmav_ln("[WARN] UDP RX buffer overflow!");
            } });
        logmav_ln("UDP MAVLink Server started on port %d\n", _udpServerPort);
    }
    else
    {
        logmav_ln("Failed to start UDP Server on port %d\n", _udpServerPort);
    }
}

void MavlinkHandler::beginMavlinkTasks()
{
    if (_serialEnabled)
    {
        xTaskCreatePinnedToCore(mavlinkSerialParseTask, "MavlinkSerialTask", PARSE_TASK_STACK_SIZE, this, PARSE_TASK_PRIORITY, &_serialTaskHandle, PARSE_TASK_CORE);
    }
    if (_tcpClientEnabled)
    {
        xTaskCreatePinnedToCore(mavlinkTcpClientParseTask, "MavlinkTcpClientTask", PARSE_TASK_STACK_SIZE, this, PARSE_TASK_PRIORITY, &_tcpClientTaskHandle, PARSE_TASK_CORE);
    }
    if (_udpServerEnabled)
    {
        xTaskCreatePinnedToCore(mavlinkUdpParseTask, "MavlinkUdpTask", PARSE_TASK_STACK_SIZE, this, PARSE_TASK_PRIORITY, &_udpTaskHandle, PARSE_TASK_CORE);
    }
}

// --- Client Management ---

void MavlinkHandler::handleNewClient(AsyncClient *client)
{
    logmav_ln("New TCP client connected from %s\n", client->remoteIP().toString().c_str());

    auto *context = new ServerClientContext{
        .client = client,
        .rxBuffer = xRingbufferCreate(RING_BUFFER_SIZE, RINGBUF_TYPE_BYTEBUF),
        .taskHandle = NULL,
        .handler = this,
        .active = true};

    if (!context->rxBuffer)
    {
        logmav_ln("[ERROR] Failed to create ring buffer for server client. Disconnecting.");
        delete context;
        client->stop();
        return;
    }

    client->setNoDelay(true);

    if (xSemaphoreTake(_serverClientsMutex, portMAX_DELAY) == pdTRUE)
    {
        _serverClients[client] = context;
        xSemaphoreGive(_serverClientsMutex);
    }

    client->onData([this, context](void *arg, AsyncClient *c, void *data, size_t len)
                   {
        if (xRingbufferSend(context->rxBuffer, data, len, pdMS_TO_TICKS(10)) != pdTRUE) {
            logmav_ln("[WARN] TCP server client %s RX buffer overflow!\n", c->remoteIP().toString().c_str());
        } });

    auto onDisconnectOrError = [this](void *arg, AsyncClient *c)
    {
        logmav_ln("TCP client %s disconnected/errored.\n", c->remoteIP().toString().c_str());
        cleanupClient(c);
    };
    client->onDisconnect(onDisconnectOrError);
    // client->onError(onDisconnectOrError);

    char taskName[32];
    snprintf(taskName, sizeof(taskName), "MavSrv_%s", client->remoteIP().toString().c_str());
    xTaskCreatePinnedToCore(mavlinkTcpServerClientParseTask, taskName, PARSE_TASK_STACK_SIZE, context, PARSE_TASK_PRIORITY, &context->taskHandle, PARSE_TASK_CORE);
}

void MavlinkHandler::cleanupClient(AsyncClient *client)
{
    if (xSemaphoreTake(_serverClientsMutex, portMAX_DELAY) == pdTRUE)
    {
        auto it = _serverClients.find(client);
        if (it != _serverClients.end())
        {
            ServerClientContext *context = it->second;
            context->active = false; // Signal the task to terminate
            _serverClients.erase(it);
            // The task is now responsible for cleaning up its own resources.
        }
        xSemaphoreGive(_serverClientsMutex);
    }
}

void MavlinkHandler::cleanupUdpClients()
{
    if (xSemaphoreTake(_udpClientsMutex, portMAX_DELAY) == pdTRUE)
    {
        uint64_t now = esp_timer_get_time();
        for (auto it = _udpClients.begin(); it != _udpClients.end();)
        {
            if ((now - it->second->lastHeard) > UDP_CLIENT_TIMEOUT_US)
            {
                logmav_ln("UDP client %s timed out. Removing.\n", it->first.c_str());
                delete it->second;          // Free the context memory
                it = _udpClients.erase(it); // Erase and move to the next valid iterator
            }
            else
            {
                ++it;
            }
        }
        xSemaphoreGive(_udpClientsMutex);
    }
}

// --- MAVLink Parsing Tasks (static methods) ---

void MavlinkHandler::mavlinkSerialParseTask(void *pvParameters)
{
    auto *handler = static_cast<MavlinkHandler *>(pvParameters);
    mavlink_status_t status = {};
    mavlink_message_t msg = {};

    while (true)
    {
        if (handler->_serialPort->available())
        {
            uint8_t c = handler->_serialPort->read();
            if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
            {
                // Serial.printf("[Serial] MAVLink msg ID: %u\n", msg.msgid);
                handler->sendDuckMessage(&msg, MavlinkInterface::MAVSERIAL);
            }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(5)); // No data, yield to other tasks
        }
    }
}

void MavlinkHandler::mavlinkTcpClientParseTask(void *pvParameters)
{
    logmav_ln("mavlinkTcpClientParseTask() running on core %u", xPortGetCoreID());
    logmav_ln("max freeRTOS tasks: %u", configMAX_PRIORITIES - 1);
    auto *handler = static_cast<MavlinkHandler *>(pvParameters);
    mavlink_status_t status = {};
    mavlink_message_t msg = {};
    size_t item_size;

    while (true)
    {
        uint8_t *item = (uint8_t *)xRingbufferReceive(handler->_tcpClientRxBuffer, &item_size, portMAX_DELAY);
        if (item)
        {
            for (size_t i = 0; i < item_size; ++i)
            {
                if (mavlink_parse_char(MAVLINK_COMM_1, item[i], &msg, &status))
                {
                    // Serial.printf("[TCP Client] MAVLink msg ID: %u\n", msg.msgid);
                    handler->sendDuckMessage(&msg, MavlinkInterface::TCP_CLIENT);
                }
            }
            vRingbufferReturnItem(handler->_tcpClientRxBuffer, (void *)item);
        }
    }
}

void MavlinkHandler::mavlinkTcpServerClientParseTask(void *pvParameters)
{
    auto *context = static_cast<ServerClientContext *>(pvParameters);
    MavlinkHandler *handler = context->handler;
    mavlink_status_t status = {};
    mavlink_message_t msg = {};
    size_t item_size;

    while (context->active)
    {
        // Wait up to 100ms for data before checking the 'active' flag again
        uint8_t *item = (uint8_t *)xRingbufferReceive(context->rxBuffer, &item_size, pdMS_TO_TICKS(100));
        if (item)
        {
            for (size_t i = 0; i < item_size; ++i)
            {
                if (mavlink_parse_char(MAVLINK_COMM_2, item[i], &msg, &status))
                {
                    // Serial.printf("[TCP Srv %s] MAVLink msg ID: %u\n", context->client->remoteIP().toString().c_str(), msg.msgid);
                    handler->sendDuckMessage(&msg, MavlinkInterface::TCP_SERVER);
                }
            }
            vRingbufferReturnItem(context->rxBuffer, (void *)item);
        }
    }

    // Cleanup phase: client has disconnected
    vRingbufferDelete(context->rxBuffer);
    delete context;
    vTaskDelete(NULL); // Task deletes itself
}

void MavlinkHandler::mavlinkUdpParseTask(void *pvParameters)
{
    auto *handler = static_cast<MavlinkHandler *>(pvParameters);
    mavlink_status_t status = {};
    mavlink_message_t msg = {};
    size_t item_size;
    uint32_t cleanup_timer = 0;

    while (true)
    {
        uint8_t *item = (uint8_t *)xRingbufferReceive(handler->_udpRxBuffer, &item_size, pdMS_TO_TICKS(1000));
        if (item)
        {
            for (size_t i = 0; i < item_size; ++i)
            {
                if (mavlink_parse_char(MAVLINK_COMM_3, item[i], &msg, &status))
                {
                    // logmav_ln("[UDP] MAVLink msg ID: %u\n", msg.msgid);
                    handler->sendDuckMessage(&msg, MavlinkInterface::UDP_SERVER);
                }
            }
            vRingbufferReturnItem(handler->_udpRxBuffer, (void *)item);
        }

        // Periodically run the cleanup function every 10 seconds
        if (millis() - cleanup_timer > 10000)
        {
            handler->cleanupUdpClients();
            cleanup_timer = millis();
        }
    }
}

// --- Data Forwarding Logic ---

void MavlinkHandler::sendMessage(const mavlink_message_t *msg, MavlinkInterface excludeInterface)
{
    printMavlinkMessage(msg);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, msg);

    if (_serialEnabled && excludeInterface != MavlinkInterface::MAVSERIAL)
    {
        sendSerial(buffer, len);
    }
    if (_tcpClientEnabled && _tcpClient && _tcpClient->connected() && excludeInterface != MavlinkInterface::TCP_CLIENT)
    {
        sendTcpClient(buffer, len);
    }
    if (_tcpServerEnabled && excludeInterface != MavlinkInterface::TCP_SERVER)
    {
        sendToAllTcpClients(buffer, len);
    }
    if (_udpServerEnabled && excludeInterface != MavlinkInterface::UDP_SERVER)
    {
        sendToAllUdpClients(buffer, len);
    }
}

void MavlinkHandler::sendDuckMessage(const mavlink_message_t *msg, MavlinkInterface excludeInterface)
{
    // Forward to other conventional MAVLink interfaces
    sendMessage(msg, excludeInterface);

    // Forward to LoRa (Duck) interface
    if (excludeInterface != MavlinkInterface::LORA)
    {
        splitAndSendMavlinkMessage(*msg);
    }
}

void MavlinkHandler::sendSerial(const uint8_t *buffer, size_t len)
{
    _serialPort->write(buffer, len);
}

void MavlinkHandler::sendTcpClient(const uint8_t *buffer, size_t len)
{
    // tries not to block when buffer is full
    while (_tcpClient->space() < len)
    {
        vTaskDelay(pdMS_TO_TICKS(5)); // Wait 5ms
    }
    _tcpClient->add((const char *)buffer, len, ASYNC_WRITE_FLAG_COPY);
}

void MavlinkHandler::sendToAllTcpClients(const uint8_t *buffer, size_t len)
{
    if (xSemaphoreTake(_serverClientsMutex, portMAX_DELAY) == pdTRUE)
    {
        for (auto const &[client, context] : _serverClients)
        {
            if (client && client->connected())
            {
                // tries not to block when buffer is full
                while (client->space() < len)
                {
                    vTaskDelay(pdMS_TO_TICKS(5)); // Wait 5ms
                }
                client->add((const char *)buffer, len, ASYNC_WRITE_FLAG_COPY);
            }
        }
        xSemaphoreGive(_serverClientsMutex);
    }
}

void MavlinkHandler::sendToAllUdpClients(const uint8_t *buffer, size_t len)
{
    if (xSemaphoreTake(_udpClientsMutex, portMAX_DELAY) == pdTRUE)
    {
        for (auto const &[key, context] : _udpClients)
        {
            _udp.writeTo(buffer, len, context->ip, context->port);
        }
        xSemaphoreGive(_udpClientsMutex);
    }
}
