#include "calibration_chicony_006.h"

#include <sys/types.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <sys/types.h>
#include <time.h>
#include <string.h>
#include <iostream>
#include <fstream>

#include <glog/logging.h>

static const char *EEPROM_REPLACEMENT_PATH = "/home/toybrick/workspace/github/aditof_sdk/chicony_firmware.bin";

CalibrationChicony006::CalibrationChicony006() {
}

CalibrationChicony006::~CalibrationChicony006() {
}

aditof::Status CalibrationChicony006::initialize(std::shared_ptr<aditof::DeviceInterface> device)
{
    using namespace aditof;

	aditof::Status status = Status::OK;

    m_device = device;

    

    return status;
}

aditof::Status CalibrationChicony006::close() {
    using namespace aditof;
    
    return Status::OK;;
}


//! setMode - Sets the mode to be used for depth calibration
/*!
setMode - Sets the mode to be used for depth calibration
\param mode - Camera depth mode
*/
aditof::Status CalibrationChicony006::setMode(const std::string& mode)
{
    using namespace aditof;
    
    aditof::Status status = Status::OK;

	LOG(INFO) << "Chosen mode: " << mode.c_str();

 	if (mode != "near") {
		LOG(WARNING) << "Unsupported mode";
		return Status::INVALID_ARGUMENT;
	}

	uint32_t firmwareLength = 0;
	status = readEeprom(
		0xFFFFFFFE, reinterpret_cast<uint8_t *>(&firmwareLength),
		sizeof(firmwareLength));
	if (status != Status::OK) {
		LOG(WARNING) << "Failed to read firmware size";
		return Status::UNREACHABLE;
	}

	uint8_t *firmwareData = new uint8_t[firmwareLength];
	status = readEeprom(0xFFFFFFFF, firmwareData, firmwareLength);

	if (status != Status::OK) {
		delete[] firmwareData;
		LOG(WARNING) << "Failed to read firmware";
		return Status::UNREACHABLE;
	} else {
		LOG(INFO) << "Found firmware for mode: " << mode;
	}

	LOG(INFO) << "Firmware size: " << firmwareLength << " bytes";
	status = m_device->program(firmwareData, firmwareLength);
	if (status != Status::OK) {
		delete[] firmwareData;
		LOG(WARNING) << "Failed to program AFE";
		return Status::UNREACHABLE;
	}

	delete[] firmwareData;

    return status;
}

aditof::Status CalibrationChicony006::readEeprom(uint32_t address, uint8_t *data,
												 size_t length) {
    using namespace aditof;
    Status status = Status::OK;

	switch (address) {
	case (0xFFFFFFFE): {
		std::ifstream file(EEPROM_REPLACEMENT_PATH,
						   std::ios::binary | std::ios::ate);
		uint32_t *size = reinterpret_cast<uint32_t *>(data);
		*size = static_cast<uint32_t>(file.tellg());
		file.close();
		LOG(INFO) << "ADDR: 0xFFFFFFFE";
		return Status::OK;
	}
	case (0xFFFFFFFF): {
		std::ifstream firmware_file(EEPROM_REPLACEMENT_PATH,
									std::ios::binary);
		firmware_file.read(reinterpret_cast<char *>(data), length);
		firmware_file.close();
		return Status::OK;
	}
	default: {
		LOG(WARNING) << "Unsupported address";
		return Status::INVALID_ARGUMENT;
	}
	} // switch (address)

    return status;
}
