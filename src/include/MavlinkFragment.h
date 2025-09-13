#pragma once

#include <cstdint>

// --- Our Mavlink Fragmentation Packet Structure ---

// maximum CDP payload size per fragment
constexpr size_t MAX_CDP_PACKET_SIZE = 229;

// size of the header fields: message_id (2) + total_fragments (1) + fragment_sequence (1) + payload_length (1)
constexpr size_t MAVLINK_FRAGMENT_HEADER_SIZE = sizeof(uint16_t) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint8_t); // 5 bytes

// The maximum amount of MAVLink data we can fit in a single fragment
constexpr size_t MAX_PAYLOAD_PER_FRAGMENT = MAX_CDP_PACKET_SIZE - MAVLINK_FRAGMENT_HEADER_SIZE; // 224 bytes

#pragma pack(push, 1) // remove padding
struct MavlinkFragmentPacket
{
    uint16_t message_id;                    // ID for the original MAVLink message
    uint8_t total_fragments;                // Total fragments for this message
    uint8_t fragment_sequence;              // This fragment's number (0-indexed)
    uint8_t payload_length;                 // Length of the data in this fragment
    uint8_t data[MAX_PAYLOAD_PER_FRAGMENT]; // The actual data chunk
};
#pragma pack(pop)
