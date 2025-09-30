#include "../MavMamaDuck.h"
#include "../MemoryFree.h"
#include "common/mavlink.h"
#include "../DuckMav.h" // Include DuckMav to know about the MavlinkHandler class

// Make the global mavlinkHandler from the .ino file available to this file
extern MavlinkHandler mavlinkHandler;

int MavMamaDuck::setupWithDefaults(std::array<byte, 8> deviceId, std::string ssid, std::string password)
{

    int err = Duck::setupWithDefaults(deviceId, ssid, password);
    if (err != DUCK_ERR_NONE)
    {
        logerr_ln("ERROR setupWithDefaults rc = %d", err);
        return err;
    }

    err = setupRadio();
    if (err != DUCK_ERR_NONE)
    {
        logerr_ln("ERROR setupWithDefaults rc = %d", err);
        return err;
    }

    std::string name(deviceId.begin(), deviceId.end());
    err = setupWifi(name.c_str());

    if (err != DUCK_ERR_NONE)
    {
        logerr_ln("ERROR setupWithDefaults rc = %d", err);
        return err;
    }

    err = setupDns();
    if (err != DUCK_ERR_NONE)
    {
        logerr_ln("ERROR setupWithDefaults rc = %d", err);
        return err;
    }

    err = setupWebServer(false);
    if (err != DUCK_ERR_NONE)
    {
        logerr_ln("ERROR setupWithDefaults rc = %d", err);
        return err;
    }

    duckutils::getTimer().every(CDPCFG_MILLIS_ALIVE, imAlive);

    duckNet->loadChannel();

    return DUCK_ERR_NONE;
}
// If no fragment is received for a message within X seconds, discard it.
constexpr uint32_t REASSEMBLY_TIMEOUT_MS = 2000;

void MavMamaDuck::cleanupStaleReassemblyBuffers()
{
    uint32_t now = millis();

    // We must use an iterator-based loop to safely delete elements while iterating
    for (auto it = reassembly_map.begin(); it != reassembly_map.end();)
    {
        // Check if the entry has timed out
        if (now - it->second.last_update_time > REASSEMBLY_TIMEOUT_MS)
        {
            Serial.println("TIMEOUT: Discarding incomplete message ID " + String(it->first));
            // Erase the stale entry and update the iterator to the next valid element
            it = reassembly_map.erase(it);
        }
        else
        {
            // Otherwise, move to the next element
            ++it;
        }
    }
}

void MavMamaDuck::run()
{
    Duck::logIfLowMemory();

    duckRadio.serviceInterruptFlags();

    if (DuckRadio::getReceiveFlag())
    {

        handleReceivedPacket();
        rxPacket->reset();
    }
    processPortalRequest();
    cleanupStaleReassemblyBuffers();
}

void MavMamaDuck::handleReceivedPacket()
{

    std::vector<byte> data;
    bool relay = false;

    loginfo_ln("====> handleReceivedPacket: START");

    int err = duckRadio.readReceivedData(&data);
    if (err != DUCK_ERR_NONE)
    {
        logerr_ln("ERROR failed to get data from DuckRadio. rc = %d", err);
        return;
    }
    logdbg_ln("Got data from radio, prepare for relay. size: %d", data.size());

    relay = rxPacket->prepareForRelaying(&filter, data);
    if (relay)
    {
        // TODO: this callback is causing an issue, needs to be fixed for mamaduck to get packet data
        // recvDataCallback(rxPacket->getBuffer());
        loginfo_ln("handleReceivedPacket: packet RELAY START");
        // NOTE:
        // Ducks will only handle received message one at a time, so there is a chance the
        // packet being sent below will never be received, especially if the cluster is small
        // there are not many alternative paths to reach other mama ducks that could relay the packet.

        CdpPacket packet = CdpPacket(rxPacket->getBuffer());

        // Check if Duck is desitination for this packet before relaying
        if (duckutils::isEqual(BROADCAST_DUID, packet.dduid))
        {
            switch (packet.topic)
            {
            case reservedTopic::ping:
                loginfo_ln("PING received. Sending PONG!");
                err = sendPong();
                if (err != DUCK_ERR_NONE)
                {
                    logerr_ln("ERROR failed to send pong message. rc = %d", err);
                }
                return;
                break;
            case reservedTopic::pong:
                loginfo_ln("PONG received. Ignoring!");
                break;
            case reservedTopic::cmd:
                loginfo_ln("Command received");
                handleCommand(packet);

                err = duckRadio.relayPacket(rxPacket);
                if (err != DUCK_ERR_NONE)
                {
                    logerr_ln("====> ERROR handleReceivedPacket failed to relay. rc = %d", err);
                }
                else
                {
                    loginfo_ln("handleReceivedPacket: packet RELAY DONE");
                }
                break;
            }
        }
        else if (duckutils::isEqual(duid, packet.dduid))
        { // Target device check
            std::vector<byte> dataPayload;
            byte num = 1;

            switch (packet.topic)
            {
            case topics::dcmd:
                loginfo_ln("Duck command received");
                handleDuckCommand(packet);
                break;
            case reservedTopic::cmd:
                loginfo_ln("Command received");

                // Handle Command
                handleCommand(packet);

                break;
            default:
                err = duckRadio.relayPacket(rxPacket);
                if (err != DUCK_ERR_NONE)
                {
                    logerr_ln("====> ERROR handleReceivedPacket failed to relay. rc = %d", err);
                }
                else
                {
                    loginfo_ln("handleReceivedPacket: packet RELAY DONE");
                }
            }
        }
        else
        {
            // TODO: properly implement a papa or broadcast id, and use the mavlink topic
            switch (packet.topic)
            {
            case topics::mavlink:
                loginfo_ln("Mavlink Packet received");

                handleMavlinkPacket(packet);

                err = duckRadio.relayPacket(rxPacket);
                if (err != DUCK_ERR_NONE)
                {
                    logerr_ln("====> ERROR handleReceivedPacket failed to relay. rc = %d", err);
                }
                else
                {
                    loginfo_ln("handleReceivedPacket: packet MAVLINK RELAY DONE");
                }

                break;

            default:

                err = duckRadio.relayPacket(rxPacket);
                if (err != DUCK_ERR_NONE)
                {
                    logerr_ln("====> ERROR handleReceivedPacket failed to relay. rc = %d", err);
                }
                else
                {
                    loginfo_ln("handleReceivedPacket: packet RELAY DONE");
                }
            }
        }
    }
}

void MavMamaDuck::handleCommand(const CdpPacket &packet)
{
    int err;
    std::vector<byte> dataPayload;
    std::vector<byte> alive{'I', 'm', ' ', 'a', 'l', 'i', 'v', 'e'};

    switch (packet.data[0])
    {
    case 0:
        // Send health quack
        loginfo_ln("Health request received");
        dataPayload.insert(dataPayload.end(), alive.begin(), alive.end());
        err = txPacket->prepareForSending(&filter, PAPADUCK_DUID,
                                          DuckType::MAMA, topics::health, dataPayload);
        if (err != DUCK_ERR_NONE)
        {
            logerr_ln("ERROR handleReceivedPacket. Failed to prepare ack. Error: %d", err);
        }

        err = duckRadio.sendData(txPacket->getBuffer());
        if (err == DUCK_ERR_NONE)
        {
            CdpPacket healthPacket = CdpPacket(txPacket->getBuffer());
            filter.bloom_add(healthPacket.muid.data(), MUID_LENGTH);
        }
        else
        {
            logerr_ln("ERROR handleReceivedPacket. Failed to send ack. Error: %d", err);
        }

        break;
    case 1:
// Change wifi status
#ifdef CDPCFG_WIFI_NONE
        logwarn_ln("WiFi not supported");
#else
        if ((char)packet.data[1] == '1')
        {
            loginfo_ln("Command WiFi ON");
            WiFi.mode(WIFI_AP);
        }
        else if ((char)packet.data[1] == '0')
        {
            loginfo_ln("Command WiFi OFF");
            WiFi.mode(WIFI_MODE_NULL);
        }
#endif
        break;
    default:
        logerr_ln("Command not recognized");
    }
}

void MavMamaDuck::handleDuckCommand(const CdpPacket &packet)
{
    loginfo_ln("Doesn't do anything yet. But Duck Command was received.");
}

bool MavMamaDuck::getDetectState() { return duckutils::getDetectState(); }
#include "MavlinkFragment.h"
#include <cstdint>

// --- Mavlink Fragmentation Packet Structure ---

#include <map>
#include <vector>
#include <numeric> // For std::accumulate
#include <protocol.h>
#include "MamaDuck.h"

/**
 * @brief Processes a received fragment and attempts to reassemble the full message.
 * @param incoming_packet Pointer to the received MavlinkFragmentPacket.
 */
void MavMamaDuck::processIncomingFragment(const MavlinkFragmentPacket *incoming_packet)
{
    uint16_t msg_id = incoming_packet->message_id;

    // Find or create the reassembly buffer for this message_id
    if (reassembly_map.find(msg_id) == reassembly_map.end())
    {
        // First time we see this message_id, create a new buffer
        ReassemblyBuffer new_buffer;
        new_buffer.total_fragments = incoming_packet->total_fragments;
        // Pre-allocate space for all fragments to avoid resizing
        new_buffer.fragments.resize(new_buffer.total_fragments);
        reassembly_map[msg_id] = new_buffer;
        new_buffer.last_update_time = millis();
        Serial.println("New message started (ID: " + String(msg_id) + "), expecting " + String(new_buffer.total_fragments) + " fragments.");
    }

    ReassemblyBuffer &buffer = reassembly_map[msg_id];

    // Store the fragment's data if we haven't seen it before
    uint8_t seq = incoming_packet->fragment_sequence;
    if (buffer.fragments[seq].empty())
    {
        buffer.fragments_received_count++;
        buffer.fragments[seq].assign(incoming_packet->data, incoming_packet->data + incoming_packet->payload_length);
        buffer.last_update_time = millis();
        Serial.println("Received fragment " + String(seq + 1) + "/" + String(buffer.total_fragments) + " for message ID " + String(msg_id));
    }
    else
    {
        // Duplicate fragment, ignore it
        return;
    }

    // Check if the message is complete
    if (buffer.fragments_received_count == buffer.total_fragments)
    {
        Serial.println("Message " + String(msg_id) + " is complete! Rebuilding...");

        // Concatenate all fragments into a single buffer
        std::vector<uint8_t> full_message_buffer;
        for (const auto &fragment_data : buffer.fragments)
        {
            full_message_buffer.insert(full_message_buffer.end(), fragment_data.begin(), fragment_data.end());
        }

        // Parse the reassembled buffer back into a MAVLink message
        mavlink_message_t reassembled_msg;
        mavlink_status_t status;
        for (uint8_t byte : full_message_buffer)
        {
            if (mavlink_parse_char(MAVLINK_COMM_0, byte, &reassembled_msg, &status))
            {
                // Successfully parsed a message!
                Serial.println("Successfully reassembled MAVLink message! ID: " + String(reassembled_msg.msgid));

                // --- DO SOMETHING WITH THE reassembled_msg ---
                // For example, print its ID
                // handleReassembledMessage(reassembled_msg);

                // TODO: FORWARD TO ENDPOINT
                // Incoming duck to outgoing mavlink
                // Send to TCP, UDP, and Serial via the mavlinkHandler
                // Duck -> Mavlink
                mavlinkHandler.sendMessage(&reassembled_msg);
                // Mavlink -> Duck in DuckMav.cpp
            }
        }

        // Clean up the map to release memory
        reassembly_map.erase(msg_id);
    }
}

void MavMamaDuck::handleMavlinkPacket(const CdpPacket &packet)
{
    // Safety check: ensure packet is large enough for the header
    if (packet.data.size() < MAVLINK_FRAGMENT_HEADER_SIZE)
    {
        logmav_ln("Error: Received packet is too small for MAVLink fragment header.");
        return;
    }

    // Cast the data to our fragment packet struct
    const MavlinkFragmentPacket *fragment_packet = reinterpret_cast<const MavlinkFragmentPacket *>(packet.data.data());

    // Second safety check: ensure the reported payload length doesn't exceed the actual packet size
    size_t expected_size = MAVLINK_FRAGMENT_HEADER_SIZE + fragment_packet->payload_length;
    if (packet.data.size() < expected_size)
    {
        logmav_ln("Error: Received packet size is smaller than indicated by its MAVLink fragment header.");
        return;
    }

    processIncomingFragment(fragment_packet);
}