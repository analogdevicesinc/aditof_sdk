/********************************************************************************/
/*  									                                        */
/* @file	CalEEPROM.h								                            */
/*										                                        */
/* @brief	Defines structure of calibration map to be stored in EEPROM,        */
/*          provides functionality to read/write/display yhe calibration map    */
/*										                                        */
/* @author	Dhruvesh Gajaria	    				                            */
/*  										                                    */
/* @date	April 22, 2019		    			                                */
/*  										                                    */
/* Copyright(c) Analog Devices, Inc.		                            		*/
/*										                                        */
/********************************************************************************/

#pragma once

#include<iostream>
#include <unordered_map>

//Hashmap key for Packet type
#define HEADER 0
#define CAMERA_INTRINSIC 1

//Hashmap key for common parameters
#define EEPROM_VERSION 1
#define CAL_SER_NUM 2
#define CAL_DATE 3
#define CHECKSUM 4


//Hashmap key for Header Parameters
#define TOTAL_SIZE 5
#define NUMBER_OF_MODES 6

//Hashmap key for Camera Intrinsic
#define INTRINSIC 5
#define DISTORTION_COEFFICIENTS 6

//! param_struct - Structure to hold the value of parameters
/*!
    param_struct provides structure to store the value of parameters.
    \param size - size of the parameter value
    \param value - the value of the parameter 
*/
struct param_struct
{
    float size;
    //float value; Fixed 1 value per key
    std::list <float>value; //list of value per key
};

//! packet_struct - Structure to hold the packet consisting of map of parameters
/*!
    packet_struct provides structure to hold the packet(sub map) of parameters
    \param size - size of the packet
    \param packet - the packet(sub map) for parameters of certain packet type
*/
struct packet_struct
{
    float size;
    std::unordered_map<float, param_struct> packet;
};

//! DisplayCalMap - Display the entire calibration map
/*!
    DisplayCalMap - Displays the entire calibration map.
    Calibratin map is nested hash map consiting of primary key( packet type key)
    and secondary key( parameter type key)
    \param calibration_map - Calibarion map to be displayed
*/
void DisplayCalMap(std::unordered_map<float, packet_struct> calibration_map);

//! SaveCalMap - Save the entire calibration map
/*!
    SaveCalMap - Saves the entire calibration map as binary to a file.
    \param calibration_map - Calibarion map to be displayed
    \param file_name - File where the calibration map needs to be stored
*/
void SaveCalMap(std::unordered_map<float, packet_struct> calibration_map, char* file_name);

//! ReadCalMap - Read the entire calibration map
/*!
    ReadCalMap - Read the entire calibration map from a binary file
    \param calibration_map - Calibarion map to be read
    \param file_name - File from where the calibration map needs to be read
*/
void ReadCalMap(std::unordered_map<float, packet_struct> &calibration_map, char* file_name);

//! InitCalMap - Intialize the calibration map with some default value
/*!
    InitCalMap - Initializes the calibration map with some default values
    \param calibration_map - Calibarion map to be initialized
*/
void InitCalMap(std::unordered_map<float, packet_struct> &calibration_map);