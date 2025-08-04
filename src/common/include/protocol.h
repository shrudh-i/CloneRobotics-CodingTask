#pragma once

#include <cstdint>
#include <vector>
#include <cstddef>

namespace protocol {

    // --- Constants ---
    constexpr uint8_t START_BYTE = 0xA5;
    constexpr uint8_t END_BYTE = 0x5A;

    // --- Message Types ---
    // Defines the purpose of each message packet.
    enum class MessageType : uint8_t {
        // Commands from Consumer to Emulator
        CMD_START_STREAM = 0x01, // Command to start the data stream
        CMD_STOP_STREAM  = 0x02, // Command to stop the data stream
        CMD_SET_FREQ     = 0x10, // Command to set the data output frequency
        CMD_PING         = 0x11, // Command to check connectivity

        // Data from Emulator to Consumer
        DATA_IMU = 0x20,     // Packet containing IMU sensor data

        // Acknowledgements
        ACK_SUCCESS = 0x80,  // Generic success response
        ACK_FAILURE = 0x81,  // Generic failure response
    };


    // --- Message Structures ---
    // These structs define the exact byte layout of our messages.
    // The `__attribute__((packed))` directive is crucial to prevent the compiler
    // from adding padding, ensuring the size and layout are predictable.

    #pragma pack(push, 1)

    // Header that starts every message
    struct MessageHeader {
        uint8_t startByte;
        uint8_t type;
        uint8_t length; // Length of the payload ONLY
    };

    // Payload for setting the frequency
    struct PayloadSetFrequency {
        float frequency_hz;
    };

    // Payload for IMU data
    struct PayloadImuData {
        // Accelerometer data [mg, g=9.81]
        float acc_x;
        float acc_y;
        float acc_z;
        uint32_t timestamp_acc_ms; // Time stamp of accelerometer measurement [ms]

        // Gyroscope data [mDeg/s]
        int32_t gyro_x;
        int32_t gyro_y;
        int32_t gyro_z;
        uint32_t timestamp_gyro_ms; // Time stamp of gyro measurement [ms]

        // Magnetometer data [mGauss]
        float mag_x;
        float mag_y;
        float mag_z;
        uint32_t timestamp_mag_ms; // Time stamp of magnetometer measurement [ms]
    };

    // Footer that ends every message
    struct MessageFooter {
        uint8_t checksum;
        uint8_t endByte;
    };

    #pragma pack(pop)


    // --- Checksum Calculation ---
    /**
     * @brief Calculates a simple XOR checksum over a block of data.
     * @param data Pointer to the start of the data buffer.
     * @param length The number of bytes to include in the checksum.
     * @return The 8-bit checksum value.
     */
    inline uint8_t calculate_checksum(const uint8_t* data, size_t length) {
        uint8_t checksum = 0;
        for (size_t i = 0; i < length; ++i) {
            checksum ^= data[i];
        }
        return checksum;
    }

} // namespace protocol
