#include "DuckMav.h"
#include "cdpcfg.h"

// External function from the main .ino file for sending LoRa fragments
extern void splitAndSendMavlinkMessage(const mavlink_message_t &msg);

// --- Constants ---
#define RING_BUFFER_SIZE 4096      // Bytes for each RX ring buffer. Increase if you see drops.
#define PARSE_TASK_STACK_SIZE 4096 // Stack size for parser tasks
#define PARSE_TASK_PRIORITY 2      // FreeRTOS task priority (higher number = higher priority)
#define PARSE_TASK_CORE 1          // Pin tasks to a specific core (0 or 1) to improve performance

// Helper to print MAVLink messages for debugging (optional)
void printMavlinkMessage(const mavlink_message_t &msg)
{
    // This is useful for debugging but can be commented out to save CPU cycles
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
}

MavlinkHandler::~MavlinkHandler()
{
    // Safely clean up tasks and resources
    if (_serialTaskHandle)
        vTaskDelete(_serialTaskHandle);
    if (_tcpClientTaskHandle)
        vTaskDelete(_tcpClientTaskHandle);

    xSemaphoreTake(_serverClientsMutex, portMAX_DELAY);
    for (auto const &[client, context] : _serverClients)
    {
        context->active = false; // Signal task to stop
    }
    _serverClients.clear();
    xSemaphoreGive(_serverClientsMutex);

    vSemaphoreDelete(_serverClientsMutex);

    if (_tcpClientRxBuffer)
        vRingbufferDelete(_tcpClientRxBuffer);

    delete _tcpClient;
    delete _tcpServer;
}

// --- Initialization ---

void MavlinkHandler::beginSerial(HardwareSerial *serial, long baudRate)
{
#if defined(CDPCFG_MAV_RX) && defined(CDPCFG_MAV_TX)
    _serialPort = serial;
    _serialPort->begin(baudRate, SERIAL_8N1, CDPCFG_MAV_RX, CDPCFG_MAV_TX);
    _serialEnabled = true;
    Serial.println("Serial MAVLink initialized.");
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
        Serial.println("[ERROR] Failed to create TCP client RX ring buffer!");
        return;
    }

    _tcpClient->onData([this](void *arg, AsyncClient *client, void *data, size_t len)
                       {
        // Send data to the ring buffer. A small timeout allows for brief contention.
        // If the buffer is full (parser task is stalled), data will be dropped.
        if (xRingbufferSend(_tcpClientRxBuffer, data, len, pdMS_TO_TICKS(10)) != pdTRUE) {
            Serial.println("[WARN] TCP client RX buffer overflow!");
        } });

    _tcpClient->onConnect([this](void *arg, AsyncClient *client)
                          { Serial.println("TCP MAVLink client connected!"); });

    _tcpClient->onDisconnect([this](void *arg, AsyncClient *client)
                             {
        Serial.println("TCP MAVLink client disconnected. Reconnecting in 2s...");
        // A simple timed reconnect to avoid spamming connection attempts
        vTaskDelay(pdMS_TO_TICKS(2000));
        connectToTCPClient(); });
    _tcpClient->onTimeout([this](void *arg, AsyncClient *client, uint32_t time)
                          {
          Serial.println("TCP MAVLink client timed out. Reconnecting in 2s...");
          // A simple timed reconnect to avoid spamming connection attempts
          vTaskDelay(pdMS_TO_TICKS(2000));
          connectToTCPClient(); });
    connectToTCPClient();
    _tcpClientEnabled = true;
    Serial.printf("TCP MAVLink client initialized. Connecting to %s:%d\n", _tcpHost.toString().c_str(), _tcpClientPort);
}

void MavlinkHandler::connectToTCPClient()
{
    if (_tcpClientEnabled && !_tcpClient->connected() && !_tcpClient->connecting())
    {
        Serial.println("Attempting TCP client connection...");
        _tcpClient->connect(_tcpHost, _tcpClientPort);
    }
    else
    {
        vTaskDelay(pdMS_TO_TICKS(2000));
        Serial.println("Attempting TCP client connection... after 2 sec delay");
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
//         Serial.println("[MAMA] Already Connecting or Disconnecting to TCP Mavlink.");
//     }
//     else
//     {
//         Serial.println("[MAMA] Retrying connection to TCP Mavlink.");
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
    Serial.printf("TCP MAVLink Server started on port %d\n", _tcpServerPort);
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
}

// --- Client Management for TCP Server ---

void MavlinkHandler::handleNewClient(AsyncClient *client)
{
    Serial.printf("New TCP client connected from %s\n", client->remoteIP().toString().c_str());

    auto *context = new ServerClientContext{
        .client = client,
        .rxBuffer = xRingbufferCreate(RING_BUFFER_SIZE, RINGBUF_TYPE_BYTEBUF),
        .taskHandle = NULL,
        .handler = this,
        .active = true};

    if (!context->rxBuffer)
    {
        Serial.println("[ERROR] Failed to create ring buffer for server client. Disconnecting.");
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
            Serial.printf("[WARN] TCP server client %s RX buffer overflow!\n", c->remoteIP().toString().c_str());
        } });

    auto onDisconnectOrError = [this](void *arg, AsyncClient *c)
    {
        Serial.printf("TCP client %s disconnected/errored.\n", c->remoteIP().toString().c_str());
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
                printMavlinkMessage(msg);
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
    Serial.printf("beginMavlinkTasks() running on core ");
    Serial.println(xPortGetCoreID());
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
                    printMavlinkMessage(msg);
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
                    printMavlinkMessage(msg);
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

// --- Data Forwarding Logic ---

void MavlinkHandler::sendMessage(const mavlink_message_t *msg, MavlinkInterface excludeInterface)
{
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
}

void MavlinkHandler::sendDuckMessage(const mavlink_message_t *msg, MavlinkInterface excludeInterface)
{
    // Forward to other conventional MAVLink interfaces
    sendMessage(msg, excludeInterface);

    // Forward to LoRa (Duck) interface
    // if (excludeInterface != MavlinkInterface::LORA)
    // {
    //     splitAndSendMavlinkMessage(*msg);
    // }
}

void MavlinkHandler::sendSerial(const uint8_t *buffer, size_t len)
{
    _serialPort->write(buffer, len);
}

void MavlinkHandler::sendTcpClient(const uint8_t *buffer, size_t len)
{
    // Use add() for robust backpressure handling. Check space first.
    if (_tcpClient->space() > len)
    {
        _tcpClient->add((const char *)buffer, len, ASYNC_WRITE_FLAG_COPY);
    }
    else
    {
        Serial.println("[WARN] TCP client TX buffer full, dropping packet.");
    }
}

void MavlinkHandler::sendToAllTcpClients(const uint8_t *buffer, size_t len)
{
    if (xSemaphoreTake(_serverClientsMutex, portMAX_DELAY) == pdTRUE)
    {
        for (auto const &[client, context] : _serverClients)
        {
            if (client && client->connected())
            {
                if (client->space() > len)
                {
                    client->add((const char *)buffer, len, ASYNC_WRITE_FLAG_COPY);
                }
                else
                {
                    Serial.printf("[WARN] TCP server client %s TX buffer full, dropping packet.\n", client->remoteIP().toString().c_str());
                }
            }
        }
        xSemaphoreGive(_serverClientsMutex);
    }
}
