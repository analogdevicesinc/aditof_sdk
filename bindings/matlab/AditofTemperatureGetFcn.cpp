#include "AditofTemperatureGetFcn.h"
#include "aditofimaq.h"
#include <assert.h>
#include <cstring>
#include <ctime>

void AditofTemperatureGetFcn::getValue(imaqkit::IPropInfo *propertyInfo,
                                       void *value) {

    switch (propertyInfo->getPropertyIdentifier()) {
    case aditof::ADITOF_PROPERTY_AFE_TEMP:
        *(reinterpret_cast<double *>(value)) = m_parent->readAfeTemp();
        break;
    case aditof::ADITOF_PROPERTY_LASER_TEMP:
        *(reinterpret_cast<double *>(value)) = m_parent->readLaserTemp();
        break;
    default:
        assert(false && "Unhandled property data type. Need to add a new "
                        "data type case.");
    }
}
