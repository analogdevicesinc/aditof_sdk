#include "device_format.h"

//******************************************
//  CONSTRUCTOR/DESTRUCTOR
//******************************************
// Initialize the data members of the class.
DeviceFormat::DeviceFormat(void)
    : m_formatHeight(0), m_formatWidth(0), m_formatNumBands(0) {}

DeviceFormat::~DeviceFormat() {}

// Get/set frame height.
void DeviceFormat::setFormatHeight(int value) { m_formatHeight = value; }

int DeviceFormat::getFormatHeight() const { return m_formatHeight; }

// Get/set frame width.
void DeviceFormat::setFormatWidth(int value) { m_formatWidth = value; }

int DeviceFormat::getFormatWidth() const { return m_formatWidth; }

// Get/set number of bands.
void DeviceFormat::setFormatNumBands(int value) { m_formatNumBands = value; }

int DeviceFormat::getFormatNumBands() const { return m_formatNumBands; }

// Get/set frame type.
void DeviceFormat::setFormatFrameType(imaqkit::frametypes::FRAMETYPE value) {
    m_formatFrameType = value;
}

imaqkit::frametypes::FRAMETYPE DeviceFormat::getFormatFrameType() const {
    return m_formatFrameType;
}
