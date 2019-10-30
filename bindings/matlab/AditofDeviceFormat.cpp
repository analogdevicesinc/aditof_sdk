#include "AditofDeviceFormat.h"

//******************************************
//  CONSTRUCTOR/DESTRUCTOR
//******************************************
// Initialize the data members of the class.
AditofDeviceFormat::AditofDeviceFormat(void)
    : m_formatHeight(0), m_formatWidth(0), m_formatNumBands(0) {}

AditofDeviceFormat::~AditofDeviceFormat() {}

// Get/set frame height.
void AditofDeviceFormat::setFormatHeight(int value) { m_formatHeight = value; }

int AditofDeviceFormat::getFormatHeight() const { return m_formatHeight; }

// Get/set frame width.
void AditofDeviceFormat::setFormatWidth(int value) { m_formatWidth = value; }

int AditofDeviceFormat::getFormatWidth() const { return m_formatWidth; }

// Get/set number of bands.
void AditofDeviceFormat::setFormatNumBands(int value) {
    m_formatNumBands = value;
}

int AditofDeviceFormat::getFormatNumBands() const { return m_formatNumBands; }

// Get/set frame type.
void AditofDeviceFormat::setFormatFrameType(
    imaqkit::frametypes::FRAMETYPE value) {
    m_formatFrameType = value;
}

imaqkit::frametypes::FRAMETYPE AditofDeviceFormat::getFormatFrameType() const {
    return m_formatFrameType;
}
