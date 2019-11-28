#include "source_listener.h"

// TODO: delete:
#include <iostream>

void SourceListener::notify(imaqkit::IEnginePropInfo *propertyInfo,
                            void *newValue) {

    if (newValue) {
        // Get the ID of the new source requested by the user.
        m_source = *static_cast<const int *>(newValue);

        // Do not re-configure the property value unless the device is already
        // opened.
        if (m_parent->isOpen()) {
            // Apply the value to the hardware.
            applyValue();
        }
    }
}

void SourceListener::applyValue(void) {

    // If device cannot be configured while acquiring stop the device,
    // configure the source input, then restart the device.
    bool wasAcquiring = m_parent->isAcquiring();
    if (wasAcquiring) {
        m_parent->stop();
    }

    // ************************************************
    // TODO Include code here to update the device to the
    // source specified by the value of _source.
    // ************************************************

    // Restart the device if it was momentarily stopped to update the source.
    if (wasAcquiring) {
        m_parent->restart();
    }
}
