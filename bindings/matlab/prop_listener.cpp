#include "assert.h"

#include "prop_listener.h"

void PropListener::notify(imaqkit::IPropInfo *propertyInfo, void *newValue) {

    if (newValue) {
        // Store a handle to the imaqkit::IPropInfo object passed in.
        m_propInfo = propertyInfo;

        // Cast newValue to its appropriate type by checking the property type.
        switch (m_propInfo->getPropertyStorageType()) {

        // _lastDoubleValue contains the value for double properties.
        case imaqkit::propertytypes::DOUBLE:
            m_lastDoubleValue = *reinterpret_cast<double *>(newValue);
            break;

            // _lastIntValue contains value for integer properties.
            // For enumerated properties, _lastIntValue returns the ID number
            // associated with the property value.
        case imaqkit::propertytypes::INT:
            m_lastIntValue = *reinterpret_cast<int16_t *>(newValue);
            break;

            // _lastStrValue contains value for string properties.
        case imaqkit::propertytypes::STRING:
            m_lastStrValue = reinterpret_cast<char *>(newValue);
            break;

            // Since this demo adaptor only uses double, integer, or string
            // value properties, anything else should cause an assertion error.
        default:
            assert(false && "Unhandled property data type. Need to add a new "
                            "data type case.");
        }

        //	std::cout << _lastDoubleValue << " " << _lastIntValue << " "
        //		  << _lastStrValue << " \n";

        //	std::cout << _propInfo->getPropertyName() << "\n";

        // Do not re-configure the property value unless the device is already
        // opened.
        if (m_parent->isOpen() &&
            m_propInfo->getPropertyName() != aditof::AFE_TEMPERATURE_STR &&
            m_propInfo->getPropertyName() != aditof::LASER_TEMPERATURE_STR) {

            //	    std::cout << " applying value\n";

            applyValue();
        }
    }
}

void PropListener::applyValue() {

    // If device cannot be configured while acquiring data, stop the device,
    // configure the feature, then restart the device.
    bool wasAcquiring = m_parent->isAcquiring();
    if (wasAcquiring) {
        // Note: calling stop() will change the acquiring flag to false.
        // When the device tries to restart it invokes
        // DemoAdaptor::startCapture() which triggers notification for all
        // property listeners. Since the device is not acquiring data during
        // this second notification, the device will not stop and restart
        // again.
        m_parent->stop();
    }

    switch (m_propInfo->getPropertyIdentifier()) {
    case aditof::ADITOF_PROPERTY_MODE:
        m_parent->setMode(m_lastIntValue);
        break;
    case aditof::ADITOF_PROPERTY_FRAME_TYPE:
        m_parent->setDisplayedFrameType(m_lastIntValue);
        break;
    case aditof::ADITOF_PROPERTY_SMALL_SIGNAL:
        m_parent->setSmallSignalValue(m_lastIntValue);
        break;
    }

    // ************************************************
    // TODO Add code here to configure the property to its new value.
    // propName contains the name of the property to update.
    // The new value of that property is either stored in
    // _lastDoubleValue, _lastIntValue, or _lastStrValue depending on the
    // property type.
    // ************************************************

    // Restart the device if it was momentarily stopped to update the feature.
    if (wasAcquiring) {
        // Restart the device. This invokes DemoAdaptor::startCapture() which
        // invoke all property listeners.
        m_parent->restart();
    }
}
