#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <aditof/device_interface.h>
#include <aditof/status_definitions.h>
#include <iostream>
#include <list>
#include <memory>
#include <stdint.h>
#include <unordered_map>

// Hashmap key for Packet type
#define HEADER 0
#define CAMERA_INTRINSIC 1

// Hashmap key for common parameters
#define EEPROM_VERSION 1
#define CAL_SER_NUM 2
#define CAL_DATE 3
#define CHECKSUM 4

// Hashmap key for Header Parameters
#define TOTAL_SIZE 5
#define NUMBER_OF_MODES 6

// Hashmap key for Camera Intrinsic
#define INTRINSIC 5
#define DISTORTION_COEFFICIENTS 6

//! param_struct - Structure to hold the value of parameters
/*!
    param_struct provides structure to store the value of parameters.
    \param size - size of the parameter value
    \param value - the value of the parameter
*/
struct param_struct {
    uint32_t size;
    // float value; Fixed 1 value per key
    std::list<float> value; // list of value per key
};

//! packet_struct - Structure to hold the packet consisting of map of parameters
/*!
    packet_struct provides structure to hold the packet(sub map) of parameters
    \param size - size of the packet
    \param packet - the packet(sub map) for parameters of certain packet type
*/
struct packet_struct {
    uint32_t size;
    std::unordered_map<float, param_struct> packet;
};

class Calibration {
  public:
    Calibration();
    ~Calibration();

  public:
    aditof::Status saveCalMap(std::shared_ptr<aditof::DeviceInterface> device);
    aditof::Status readCalMap(std::shared_ptr<aditof::DeviceInterface> device);
    aditof::Status displayCalMap() const;
    aditof::Status getAfeFirmware(const std::string &mode,
                                  std::vector<uint16_t> &data) const;
    aditof::Status getGainOffset(const std::string &mode, float &gain,
                                 float &offset) const;

  private:
    float getMapSize(
        const std::unordered_map<float, packet_struct> &calibration_map) const;
    float
    getPacketSize(const std::unordered_map<float, param_struct> &packet) const;

  private:
    std::unordered_map<float, packet_struct> m_calibration_map;
};

#endif /*CALIBRATION_H*/
