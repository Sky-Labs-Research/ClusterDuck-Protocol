#ifndef MAVMAMADUCK_H
#define MAVMAMADUCK_H

#include <Arduino.h>
#include "include/Duck.h"
#include "include/DuckUtils.h"

#include <vector>
#include <map>
#include <string>
#include <array>
#include <cstdint> // For uint16_t, uint8_t

class MavMamaDuck : public Duck
{
public:
    using Duck::Duck;

    ~MavMamaDuck() {}

    using rxDoneCallback = void (*)(std::vector<byte> data);
    /**
     * @brief Register callback for handling data received from duck devices
     *
     * The callback will be invoked if the packet needs to be relayed (i.e not seen before)
     * @param cb a callback to handle data received by the papa duck
     */
    void onReceiveDuckData(rxDoneCallback cb) { this->recvDataCallback = cb; }

    /**
     * @brief Cleans up stale entries from the reassembly map to prevent memory leaks.
     * This should be called periodically from the main loop.
     */
    void cleanupStaleReassemblyBuffers();

    /**
     * @brief Provide the DuckLink specific implementation of the base `run()`
     * method.
     *
     */
    void run();

    /**
     * @brief Override the default setup method to match MamaDuck specific
     * defaults.
     *
     * In addition to Serial component, the Radio component is also initialized.
     * When ssid and password are provided the duck will setup the wifi related
     * components.
     *
     * @param deviceId required device unique id
     * @param ssid wifi access point ssid (defaults to an empty string if not
     * provided)
     * @param password wifi password (defaults to an empty string if not provided)
     *
     * @returns DUCK_ERR_NONE if setup is successfull, an error code otherwise.
     */
    int setupWithDefaults(std::array<byte, 8> deviceId, std::string ssid = "",
                          std::string password = "");

    /**
     * @brief Get the DuckType
     *
     * @returns the duck type defined as DuckType
     */
    int getType() { return DuckType::MAMA; }

    bool getDetectState();
    static constexpr size_t MAX_PACKET_SIZE = 229;
    static constexpr size_t FRAGMENT_HEADER_SIZE = sizeof(uint16_t) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint8_t);
    static constexpr size_t MAX_PAYLOAD_PER_FRAGMENT = MAX_PACKET_SIZE - FRAGMENT_HEADER_SIZE;

private:
#pragma pack(push, 1)
    struct MavlinkFragmentPacket
    {
        uint16_t message_id;
        uint8_t total_fragments;
        uint8_t fragment_sequence;
        uint8_t payload_length;
        uint8_t data[MAX_PAYLOAD_PER_FRAGMENT];
    };
#pragma pack(pop)

    struct ReassemblyBuffer
    {
        uint8_t total_fragments;
        uint8_t fragments_received_count = 0;
        std::vector<std::vector<uint8_t>> fragments;
        uint32_t last_update_time;
    };
    rxDoneCallback recvDataCallback;
    void handleReceivedPacket();
    void handleMavlinkPacket(const CdpPacket &packet);
    void handleCommand(const CdpPacket &packet);
    void handleDuckCommand(const CdpPacket &packet);
    void processIncomingFragment(const MavlinkFragmentPacket *incoming_packet);
    std::map<uint16_t, ReassemblyBuffer> reassembly_map;
};

#endif
