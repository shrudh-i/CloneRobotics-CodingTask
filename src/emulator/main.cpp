#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <random>
#include <csignal>

// 3rd-party libraries
#include "CLI/CLI.hpp"
#include "spdlog/spdlog.h"
#include "serial/serial.h"

// Common project headers
#include "protocol.h"

// --- Global State ---
// Used for graceful shutdown
static volatile bool keep_running = true;
// State machine for data streaming
static bool stream_active = false;

void process_command(const protocol::MessageHeader& header, const uint8_t* payload_data, serial::Serial& ser, double& frequency, std::chrono::milliseconds& loop_period);
void handle_incoming_data(serial::Serial& ser, double& frequency, std::chrono::milliseconds& loop_period);
std::vector<uint8_t> serialize_simple_message(protocol::MessageType type);

/**
 * @brief Handles interrupt signals (like Ctrl+C) for a clean shutdown.
 */
void int_handler(int) {
    spdlog::warn("Interrupt signal received. Shutting down...");
    keep_running = false;
}

/**
 * @brief Generates a single packet of random IMU data.
 * @return A fully populated PayloadImuData struct.
 */
protocol::PayloadImuData generate_imu_data() {
    // Use a static random engine for better performance
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> acc_dist(-2000.0, 2000.0); // mg
    static std::uniform_int_distribution<> gyro_dist(-32768, 32767);   // mDeg/s
    static std::uniform_real_distribution<> mag_dist(-500.0, 500.0);   // mGauss

    protocol::PayloadImuData data{};

    // Fill accelerometer data
    data.acc_x = acc_dist(gen);
    data.acc_y = acc_dist(gen);
    data.acc_z = acc_dist(gen);
    data.timestamp_acc_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();

    // Fill gyroscope data
    data.gyro_x = gyro_dist(gen);
    data.gyro_y = gyro_dist(gen);
    data.gyro_z = gyro_dist(gen);
    data.timestamp_gyro_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();

    // Fill magnetometer data
    data.mag_x = mag_dist(gen);
    data.mag_y = mag_dist(gen);
    data.mag_z = mag_dist(gen);
    data.timestamp_mag_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();

    return data;
}

/**
 * @brief Serializes a message (header, payload, footer) into a byte vector.
 * @tparam PayloadType The type of the payload struct.
 * @param type The message type from the protocol enum.
 * @param payload The payload data struct.
 * @return A std::vector<uint8_t> containing the full serialized message.
 */
template<typename PayloadType>
std::vector<uint8_t> serialize_message(protocol::MessageType type, const PayloadType& payload) {
    protocol::MessageHeader header;
    header.startByte = protocol::START_BYTE;
    header.type = static_cast<uint8_t>(type);
    header.length = sizeof(PayloadType);

    protocol::MessageFooter footer;
    footer.endByte = protocol::END_BYTE;

    // Create a buffer and copy header and payload to it for checksum calculation
    std::vector<uint8_t> checksum_buffer(sizeof(header) + sizeof(payload));
    memcpy(checksum_buffer.data(), &header, sizeof(header));
    memcpy(checksum_buffer.data() + sizeof(header), &payload, sizeof(payload));

    footer.checksum = protocol::calculate_checksum(checksum_buffer.data(), checksum_buffer.size());

    // Create the final message buffer
    std::vector<uint8_t> message_buffer(checksum_buffer.size() + sizeof(footer));
    memcpy(message_buffer.data(), checksum_buffer.data(), checksum_buffer.size());
    memcpy(message_buffer.data() + checksum_buffer.size(), &footer, sizeof(footer));

    return message_buffer;
}

/**
 * @brief Serializes a simple message (no payload) like an ACK or PING.
 */
std::vector<uint8_t> serialize_simple_message(protocol::MessageType type) {
    protocol::MessageHeader header;
    header.startByte = protocol::START_BYTE;
    header.type = static_cast<uint8_t>(type);
    header.length = 0; // No payload

    protocol::MessageFooter footer;
    footer.endByte = protocol::END_BYTE;

    // Checksum is calculated only over the header
    footer.checksum = protocol::calculate_checksum(reinterpret_cast<const uint8_t*>(&header), sizeof(header));

    std::vector<uint8_t> message_buffer(sizeof(header) + sizeof(footer));
    memcpy(message_buffer.data(), &header, sizeof(header));
    memcpy(message_buffer.data() + sizeof(header), &footer, sizeof(footer));

    return message_buffer;
}

/**
 * @brief Reads the serial port, parses incoming data, and dispatches commands.
 */
void handle_incoming_data(serial::Serial& ser, double& frequency, std::chrono::milliseconds& loop_period) {
    static std::vector<uint8_t> s_buffer; // Static buffer to handle partial reads

    try {
        size_t bytes_available = ser.available();
        if (bytes_available > 0) {
            std::vector<uint8_t> new_data(bytes_available);
            ser.read(new_data, bytes_available);
            s_buffer.insert(s_buffer.end(), new_data.begin(), new_data.end());
            spdlog::trace("Read {} bytes from serial. Buffer size is now {}.", bytes_available, s_buffer.size());
        }
    } catch (const std::exception& e) {
        spdlog::error("Error reading from serial port: {}", e.what());
        return;
    }

    // Loop until we can't find any more full messages in the buffer
    while (true) {
        // 1. Find the start of a potential message
        auto start_it = std::find(s_buffer.begin(), s_buffer.end(), protocol::START_BYTE);
        if (start_it == s_buffer.end()) {
            if (!s_buffer.empty()) {
                 spdlog::trace("No start byte found, clearing {} bytes of junk.", s_buffer.size());
                 s_buffer.clear();
            }
            return; // No message start found
        }
        
        // Discard any junk data before the start byte
        if (start_it != s_buffer.begin()) {
            spdlog::trace("Discarding {} bytes of junk before start byte.", std::distance(s_buffer.begin(), start_it));
            s_buffer.erase(s_buffer.begin(), start_it);
        }

        // 2. Check if we have enough bytes for a header
        if (s_buffer.size() < sizeof(protocol::MessageHeader)) {
            return; // Not enough data for a full header yet
        }

        // 3. De-serialize the header to find the payload length
        protocol::MessageHeader header;
        memcpy(&header, s_buffer.data(), sizeof(header));

        // 4. Check if the full message is in the buffer
        size_t full_message_size = sizeof(protocol::MessageHeader) + header.length + sizeof(protocol::MessageFooter);
        if (s_buffer.size() < full_message_size) {
            return; // Not enough data for the full message yet
        }

        // 5. We have a full message. Verify its integrity.
        std::vector<uint8_t> message_bytes(s_buffer.begin(), s_buffer.begin() + full_message_size);
        protocol::MessageFooter footer;
        memcpy(&footer, message_bytes.data() + sizeof(protocol::MessageHeader) + header.length, sizeof(protocol::MessageFooter));
        uint8_t calculated_checksum = protocol::calculate_checksum(message_bytes.data(), sizeof(protocol::MessageHeader) + header.length);

        if (footer.checksum != calculated_checksum || footer.endByte != protocol::END_BYTE) {
            spdlog::warn("Invalid message received. Bad checksum or end byte. Discarding message.");
            // Remove the invalid message and try parsing again from the next byte
            s_buffer.erase(s_buffer.begin());
            continue;
        }

        // 6. Message is valid, process it
        spdlog::debug("Valid message received (type: {:#04x}).", header.type);
        process_command(header, message_bytes.data() + sizeof(protocol::MessageHeader), ser, frequency, loop_period);

        // 7. Remove the processed message from the buffer
        s_buffer.erase(s_buffer.begin(), s_buffer.begin() + full_message_size);
    }
}

/**
 * @brief Processes a validated command from the consumer.
 */
void process_command(const protocol::MessageHeader& header, const uint8_t* payload_data, serial::Serial& ser, double& frequency, std::chrono::milliseconds& loop_period) {
    bool success = true;
    protocol::MessageType type = static_cast<protocol::MessageType>(header.type);

    switch (type) {
        case protocol::MessageType::CMD_START_STREAM:
            spdlog::info("Received START_STREAM command.");
            stream_active = true;
            break;

        case protocol::MessageType::CMD_STOP_STREAM:
            spdlog::info("Received STOP_STREAM command.");
            stream_active = false;
            break;

        case protocol::MessageType::CMD_SET_FREQ: {
            spdlog::info("Received SET_FREQ command.");
            if (header.length == sizeof(protocol::PayloadSetFrequency)) {
                protocol::PayloadSetFrequency payload;
                memcpy(&payload, payload_data, sizeof(payload));
                if (payload.frequency_hz > 0 && payload.frequency_hz <= 1000) { // Sanity check
                    frequency = payload.frequency_hz;
                    loop_period = std::chrono::milliseconds(static_cast<long>(1000.0 / frequency));
                    spdlog::info("Updated frequency to {} Hz.", frequency);
                } else {
                    success = false;
                    spdlog::warn("Received invalid frequency: {}", payload.frequency_hz);
                }
            } else {
                success = false;
                spdlog::warn("Received SET_FREQ with incorrect payload size.");
            }
            break;
        }

        case protocol::MessageType::CMD_PING:
            spdlog::info("Received PING command. Sending ACK.");
            break;

        default:
            spdlog::warn("Received unknown or unsupported command type: {:#04x}", header.type);
            success = false;
            break;
    }

    // Send ACK/NACK
    auto ack_type = success ? protocol::MessageType::ACK_SUCCESS : protocol::MessageType::ACK_FAILURE;
    auto ack_message = serialize_simple_message(ack_type);
    try {
        ser.write(ack_message);
    } catch (const std::exception& e) {
        spdlog::error("Failed to send ACK message: {}", e.what());
    }
}

int main(int argc, char** argv) {
    // --- Signal Handling for Graceful Shutdown ---
    signal(SIGINT, int_handler);
    signal(SIGTERM, int_handler);

    // --- Command-Line Argument Parsing ---
    CLI::App app{"Robotics Sensor Emulator"};

    std::string port_path = "/dev/virtual0";
    uint32_t baudrate = 4096;
    double frequency = 10.0; // Hz
    std::string log_level_str = "info";

    app.add_option("--port-path", port_path, "Path to the serial port (e.g., /dev/ttyUSB0)")->check(CLI::ExistingFile);
    app.add_option("--baudrate", baudrate, "Serial port baudrate");
    app.add_option("--frequency", frequency, "Frequency to send IMU data in Hz");
    app.add_option("--log-level", log_level_str, "Set log level (trace, debug, info, warn, error, critical, off)");

    CLI11_PARSE(app, argc, argv);

    // --- Logger Setup ---
    try {
        spdlog::set_level(spdlog::level::from_str(log_level_str));
        spdlog::info("Logger initialized with level: {}", log_level_str);
    } catch (const spdlog::spdlog_ex& ex) {
        spdlog::error("Failed to set log level: {}. Defaulting to 'info'.", ex.what());
        spdlog::set_level(spdlog::level::info);
    }

    // --- Serial Port Initialization ---
    serial::Serial ser;
    try {
        ser.setPort(port_path);
        ser.setBaudrate(baudrate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000); // 1 second timeout
        ser.setTimeout(timeout);
        ser.open();
        spdlog::info("Successfully opened serial port: {} at {} baud.", port_path, baudrate);
    } catch (const std::exception& e) {
        spdlog::critical("Failed to open serial port {}: {}. Shutting down.", port_path, e.what());
        return 1;
    }

    // --- Main Application Loop ---
    spdlog::info("Starting emulator main loop. Press Ctrl+C to exit.");
    auto loop_period = std::chrono::milliseconds(static_cast<long>(1000.0 / frequency));

    while (keep_running) {
        auto loop_start_time = std::chrono::steady_clock::now();

        // Handle any incoming commands from the consumer
        handle_incoming_data(ser, frequency, loop_period);

        if (stream_active) {
            // 1. Generate new IMU data
            protocol::PayloadImuData imu_data = generate_imu_data();
            spdlog::debug("Generated IMU data: acc_x={:.2f}", imu_data.acc_x);

            // 2. Serialize the data into a message packet
            std::vector<uint8_t> message = serialize_message(protocol::MessageType::DATA_IMU, imu_data);

            // 3. Write the message to the serial port
            try {
                size_t bytes_written = ser.write(message);
                spdlog::trace("Wrote {} bytes to serial port.", bytes_written);
            } catch (const std::exception& e) {
                spdlog::error("Failed to write to serial port: {}", e.what());
                // In a real scenario, you might try to re-open the port here
            }
        }

        auto loop_end_time = std::chrono::steady_clock::now();
        auto loop_duration = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time - loop_start_time);

        // Sleep to maintain the desired frequency
        if (loop_duration < loop_period) {
            std::this_thread::sleep_for(loop_period - loop_duration);
        }
    }

    // --- Cleanup ---
    if (ser.isOpen()) {
        ser.close();
    }
    spdlog::info("Emulator has shut down cleanly.");

    return 0;
}
