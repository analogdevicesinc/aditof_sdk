#include "calibration.h"

#include <glog/logging.h>

#define EEPROM_SIZE 131072

Calibration::Calibration() {
    std::unordered_map<float, param_struct> Header;
    Header[EEPROM_VERSION].value = {0};
    Header[EEPROM_VERSION].size =
        sizeof(Header[EEPROM_VERSION].value.size()) * 4;
    Header[TOTAL_SIZE].value = {1000};
    Header[TOTAL_SIZE].size = sizeof(Header[TOTAL_SIZE].value.size()) * 4;
    Header[NUMBER_OF_MODES].value = {3};
    Header[NUMBER_OF_MODES].size = sizeof(Header[TOTAL_SIZE].value.size()) * 4;

    std::unordered_map<float, param_struct> CameraIntrinsic;
    CameraIntrinsic[EEPROM_VERSION].value = {0};
    CameraIntrinsic[EEPROM_VERSION].size =
        (CameraIntrinsic[EEPROM_VERSION].value.size() * 4);
    CameraIntrinsic[CAL_SER_NUM].value = {0};
    CameraIntrinsic[CAL_SER_NUM].size =
        (CameraIntrinsic[CAL_SER_NUM].value.size() * 4);
    CameraIntrinsic[CAL_DATE].value = {12042019};
    CameraIntrinsic[CAL_DATE].size =
        (CameraIntrinsic[CAL_SER_NUM].value.size() * 4);
    CameraIntrinsic[INTRINSIC].value = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    CameraIntrinsic[INTRINSIC].size =
        (CameraIntrinsic[INTRINSIC].value.size() * 4);

    m_calibration_map[HEADER].size = getPacketSize(Header);
    m_calibration_map[HEADER].packet = Header;
    m_calibration_map[CAMERA_INTRINSIC].size = getPacketSize(CameraIntrinsic);
    m_calibration_map[CAMERA_INTRINSIC].packet = CameraIntrinsic;
    m_calibration_map[HEADER].packet[TOTAL_SIZE].value = {
        getMapSize(m_calibration_map)};
}

Calibration::~Calibration() = default;

//! DisplayCalMap - Display the entire calibration map
/*!
    DisplayCalMap - Displays the entire calibration map.
    Calibratin map is nested hash map consiting of primary key( packet type key)
    and secondary key( parameter type key)
*/
aditof::Status Calibration::displayCalMap() const {
    using namespace aditof;

    for (const auto &mapElement : m_calibration_map) {
        std::cout << "Key: " << mapElement.first;
        const packet_struct &sub_packet_map = mapElement.second;
        std::cout << "\t Size: " << sub_packet_map.size << std::endl;

        for (const auto &packet : sub_packet_map.packet) {
            std::cout << "\tSub Key: " << packet.first;
            std::cout << "\tSub Size: " << packet.second.size;
            std::cout << "\tSub Value: ";

            for (const auto &value : packet.second.value)
                std::cout << value << " ";
            std::cout << std::endl;
        }
    }

    return Status::OK;
}

aditof::Status Calibration::getAfeFirmware(const std::string &mode,
                                           std::vector<uint16_t> &data) const {
    using namespace aditof;

    uint8_t cal_mode;

    if (mode == "near") {
        cal_mode = 3;
    } else if (mode == "medium") {
        cal_mode = 5;
    } else if (mode == "far") {
        cal_mode = 7;
    } else {
        LOG(WARNING) << "Invalid firmware mode " << mode.c_str();
        return Status::INVALID_ARGUMENT;
    }

    for (const auto &mapElement : m_calibration_map) {
        float key = mapElement.first;
        const packet_struct &sub_packet_map = mapElement.second;

        if (cal_mode == key) {
            for (const auto &packet : sub_packet_map.packet) {
                if (packet.first == 5) {
                    for (const auto &value : packet.second.value) {
                        data.push_back(static_cast<uint16_t>(value));
                    }
                    return Status::OK;
                }
            }
        }
    }

    return Status::GENERIC_ERROR;
}

aditof::Status Calibration::getGainOffset(const std::string &mode, float &gain,
                                          float &offset) const {
    using namespace aditof;

    uint8_t cal_mode;

    if (mode == "near") {
        cal_mode = 2;
    } else if (mode == "medium") {
        cal_mode = 4;
    } else if (mode == "far") {
        cal_mode = 6;
    } else {
        LOG(WARNING) << "Invalid firmware mode " << mode.c_str();
        return Status::INVALID_ARGUMENT;
    }

    for (const auto &mapElement : m_calibration_map) {
        float key = mapElement.first;
        const packet_struct &sub_packet_map = mapElement.second;

        if (cal_mode == key) {
            for (const auto &packet : sub_packet_map.packet) {
                if (packet.first == 26) {
                    gain = packet.second.value.front();
                }
                if (packet.first == 27) {
                    offset = packet.second.value.front();
                }
            }
            return Status::OK;
        }
    }

    return Status::GENERIC_ERROR;
}

//! SaveCalMap - Save the entire calibration map
/*!
    SaveCalMap - Saves the entire calibration map as binary to a file.
    \param calibration_map - Calibarion map to be displayed
    \param file_name - File where the calibration map needs to be stored
*/
aditof::Status
Calibration::saveCalMap(std::shared_ptr<aditof::DeviceInterface> device) {
    using namespace aditof;

    std::vector<float> data;
    for (const auto &mapElement : m_calibration_map) {
        data.push_back(mapElement.first);
        const packet_struct &sub_packet_map = mapElement.second;
        data.push_back(sub_packet_map.size);

        for (const auto &packet : sub_packet_map.packet) {
            data.push_back(packet.first);       // write parameter key
            data.push_back(packet.second.size); // write size of parameter

            for (const auto &value : packet.second.value) {
                data.push_back(
                    static_cast<float>(value)); // write parameter values
            }
        }
    }

    size_t size = data.size() * sizeof(uint32_t);
    device->writeEeprom((uint32_t)0, (uint8_t *)&size, (size_t)4);
    device->writeEeprom((uint32_t)4, (uint8_t *)data.data(), size);

    return Status::OK;
}

//! ReadCalMap - Read the entire calibration map
/*!
    ReadCalMap - Read the entire calibration map from a binary file
    \param calibration_map - Calibarion map to be read
    \param file_name - File from where the calibration map needs to be read
*/
aditof::Status
Calibration::readCalMap(std::shared_ptr<aditof::DeviceInterface> device) {
    using namespace aditof;

    Status status = Status::OK;
    uint8_t *data;
    float read_size = 100;
    uint32_t j = 0;
    float key;

    device->writeEeprom(EEPROM_SIZE - 5, (uint8_t *)&read_size, 4);

    device->readEeprom(0, (uint8_t *)&read_size, 4);
    LOG(INFO) << "EEPROM calibration data size " << read_size << " bytes";

    if (read_size > EEPROM_SIZE) {
        LOG(WARNING) << "Invalid calibration data size";
        return Status::GENERIC_ERROR;
    }

    data = (uint8_t *)malloc(read_size);

    status = device->readEeprom(4, data, (size_t)read_size);
    if (status != Status::OK) {
        free(data);
        LOG(WARNING) << "Failed to read from eeprom";
        return status;
    }

    while (j < read_size) {
        key = *(float *)(data + j);
        j += 4;

        packet_struct sub_packet_map;

        sub_packet_map.size = *(float *)(data + j);
        j += 4;

        for (unsigned int i = 0;
             i < sub_packet_map.size /
                     (sizeof(float));) // Parse all the sub-packets
        {
            float parameter_key;
            parameter_key =
                *(float *)(data + j); // Parse key of parameter from sub packet
            j += 4;
            i++;
            sub_packet_map.packet[parameter_key].size =
                *(float *)(data + j); // Parse size of parameter from sub packet
            j += 4;
            i++;

            uint32_t number_elements =
                sub_packet_map.packet[parameter_key].size / sizeof(float);
            std::list<float> elements;
            for (unsigned int k = 0; k < number_elements; k++) {
                sub_packet_map.packet[parameter_key].value.push_back(
                    *(float *)(data +
                               j)); // Parse size of parameter from sub packet
                j += 4;
                i++;
            }
        }
        m_calibration_map[key].size = sub_packet_map.size;
        m_calibration_map[key].packet = sub_packet_map.packet;
    }

    free(data);

    return Status::OK;
}

// Calculate and return the total size of calibration map
float Calibration::getMapSize(
    const std::unordered_map<float, packet_struct> &calibration_map) const {
    float total_size = 0;
    // Calculate total size of calibration map
    for (const auto &mapElement : calibration_map) {
        total_size += mapElement.second.size; // Add size of all the sub packets
    }
    return total_size;
}

// Calculate and return the size of a packet
float Calibration::getPacketSize(
    const std::unordered_map<float, param_struct> &packet) const {
    float packet_size = 0;
    for (const auto &mapElement : packet) {
        packet_size +=
            mapElement.second.size + 8; // Added 8 for size of key and size
    }
    return packet_size;
}
