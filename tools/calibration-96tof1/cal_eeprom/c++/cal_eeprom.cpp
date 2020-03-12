/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
// cal_eeprom.cpp : Defines the functions used to store/read/display calibration
// map.

#ifdef _MSC_VER
#include "stdafx.h"
#endif

#include "cal_eeprom.h"

// Display entire calibration map
void DisplayCalMap(std::unordered_map<float, packet_struct> calibration_map) {
    typedef std::unordered_map<float, packet_struct>::iterator
        calibration_map_iterator;
    for (calibration_map_iterator iter = calibration_map.begin();
         iter != calibration_map.end(); iter++) {
        std::cout << "Key: " << iter->first;
        packet_struct sub_packet_map = iter->second;
        std::cout << "\t Size: " << sub_packet_map.size << std::endl;
        typedef std::unordered_map<float, param_struct>::iterator sub_iterator;
        for (sub_iterator sub_iter = sub_packet_map.packet.begin();
             sub_iter != sub_packet_map.packet.end(); sub_iter++) {
            std::cout << "\tSub Key: " << sub_iter->first;
            std::cout << "\tSub Size: " << sub_iter->second.size;
            std::cout << "\tSub Value: ";
            for (std::list<float>::iterator itt =
                     sub_iter->second.value.begin();
                 itt != sub_iter->second.value.end(); itt++)
                std::cout << *itt << " ";
            std::cout << std::endl;
        }
    }
}

// Save the calibration map
void SaveCalMap(std::unordered_map<float, packet_struct> calibration_map,
                char *file_name) {
    // Save Calibration map
    typedef std::unordered_map<float, packet_struct>::iterator
        calibration_map_iterator;
    FILE *f = fopen(file_name, "wb");
    for (calibration_map_iterator iter = calibration_map.begin();
         iter != calibration_map.end(); iter++) {
        fwrite(&iter->first, 4, 1,
               f); // write float primary key (packet type - 4 bytes)

        packet_struct sub_packet_map = iter->second;

        fwrite(&sub_packet_map.size, 4, 1,
               f); // write size of packet ( size is written in float format)

        typedef std::unordered_map<float, param_struct>::iterator sub_iterator;
        for (sub_iterator sub_iter = sub_packet_map.packet.begin();
             sub_iter != sub_packet_map.packet.end(); sub_iter++) {
            fwrite(&sub_iter->first, 4, 1, f);       // write parameter key
            fwrite(&sub_iter->second.size, 4, 1, f); // write size of parameter
            for (std::list<float>::iterator itt =
                     sub_iter->second.value.begin();
                 itt != sub_iter->second.value.end(); itt++) {
                float value = *itt;
                fwrite(&value, 4, 1, f); // write parameter values
            }
        }
    }
    fclose(f);
}

// Read Calibration map from file
void ReadCalMap(std::unordered_map<float, packet_struct> &calibration_map,
                char *file_name) {

    // std::cout << "opening map.txt\n";
    FILE *fr = fopen(file_name, "rb");

    float key;
    while (fread(&key, 4, 1, fr) ==
           1) // Read key and check condition for end of file
    {
        float value;
        // std::cout << "Key " << key;
        packet_struct sub_packet_map;
        fread(&sub_packet_map.size, 4, 1,
              fr); // Read Size of sub packet (header,camera Intrinsic ..)
        // std::cout << "Size " << sub_packet_map.size << std::endl;

        float *sub_packet_value;
        sub_packet_value = (float *)malloc(sub_packet_map.size);
        fread(sub_packet_value, sub_packet_map.size, 1,
              fr); // Read Entire sub packet

        for (int i = 0; i < sub_packet_map.size /
                                (sizeof(float));) // Parse all the sub-packets
        {
            float parameter_key;
            parameter_key =
                sub_packet_value[i++]; // Parse key of parameter from sub packet
            sub_packet_map.packet[parameter_key].size =
                sub_packet_value[i++]; // Parse size of parameter from sub
                                       // packet

            int number_elements =
                sub_packet_map.packet[parameter_key].size / sizeof(float);
            std::list<float> elements;
            for (int j = 0; j < number_elements; j++) {
                sub_packet_map.packet[parameter_key].value.push_back(
                    sub_packet_value[i++]); // Parse size of parameter from sub
                                            // packet
            }

            /*std::cout << "\tSub Key " << parameter_key;
            std::cout << "\tSub Size " <<
            sub_packet_map.packet[parameter_key].size;
            std::cout << "\tSub Value " <<
            sub_packet_map.packet[parameter_key].value << std::endl;*/
        }
        calibration_map[key].size = sub_packet_map.size;
        calibration_map[key].packet = sub_packet_map.packet;

        free(sub_packet_value);
        sub_packet_value = NULL;
    }
    fclose(fr);
}

// Calculate and return the total size of calibration map
float GetMapSize(std::unordered_map<float, packet_struct> calibration_map) {
    float total_size = 0;
    // Calculate total size of calibration map
    typedef std::unordered_map<float, packet_struct>::iterator
        calibration_map_iterator;
    for (calibration_map_iterator iter = calibration_map.begin();
         iter != calibration_map.end(); iter++) {
        total_size =
            total_size + iter->second.size; // Add size of all the sub packets
    }
    return total_size;
}

float GetPacketSize(std::unordered_map<float, param_struct> packet) {
    typedef std::unordered_map<float, param_struct>::iterator sub_iterator;
    float packet_size = 0;
    for (sub_iterator sub_iter = packet.begin(); sub_iter != packet.end();
         sub_iter++) {
        packet_size = packet_size + sub_iter->second.size +
                      8; // Added 8 for size of key and size
    }
    return packet_size;
}

// Initialize the calibration map with some default parameters
void InitCalMap(std::unordered_map<float, packet_struct> &calibration_map) {
    std::unordered_map<float, param_struct> Header;
    Header[EEPROM_VERSION].value = {0};
    Header[EEPROM_VERSION].size =
        sizeof(Header[EEPROM_VERSION].value.size() * 4);
    Header[TOTAL_SIZE].value = {1000};
    Header[TOTAL_SIZE].size = sizeof(Header[TOTAL_SIZE].value.size() * 4);
    Header[NUMBER_OF_MODES].value = {3};
    Header[NUMBER_OF_MODES].size = sizeof(Header[TOTAL_SIZE].value.size() * 4);

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
    std::cout << CameraIntrinsic[INTRINSIC].value.size() << std::endl;
    CameraIntrinsic[INTRINSIC].size =
        (CameraIntrinsic[INTRINSIC].value.size() * 4);

    calibration_map[HEADER].size = GetPacketSize(Header);
    calibration_map[HEADER].packet = Header;
    calibration_map[CAMERA_INTRINSIC].size =
        GetPacketSize(CameraIntrinsic); // Added 8 for size of key and size
    calibration_map[CAMERA_INTRINSIC].packet = CameraIntrinsic;
    calibration_map[HEADER].packet[TOTAL_SIZE].value = {
        GetMapSize(calibration_map)};
}
