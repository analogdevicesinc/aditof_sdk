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
#ifndef __DEMO_PROP_LISTENER_HEADER__
#define __DEMO_PROP_LISTENER_HEADER__

#include "aditofimaq.h"
#include "mwadaptorimaq.h"
#include "source_adaptor.h"
#include <string>

/**
 * Class PropListener
 *
 * @brief:  Listens for changes in device-specific properties.
 *
 */
class PropListener : public imaqkit::IPropPostSetListener {

  public:
    // **************************************
    // CONSTRUCTOR/DESTRUCTOR
    // **************************************
    /**
     * Constructor for PropListener class.
     *
     * @param parent: Handle to the instance of the IAdaptor class
     *                that is the parent of this object.
     */
    PropListener(SourceAdaptor *parent) : m_parent(parent), m_lastIntValue(0) {}
    virtual ~PropListener(){};

    // *******************************************************************
    // METHODS FOR CONFIGURING AND UPDATING DEMO FEATURE PROPERTY VALUES.
    // *******************************************************************
    /**
     * This is the method the engine calls when a property value
     * changes. notify() casts the new property value to the appropriate
     * type and then calls the PropListener::applyValue() method to
     * configure the property.
     *
     * @param propertyInfo: The property information object.
     * @param newValue: The new value of the property.
     *
     * @return void:
     */
    virtual void notify(imaqkit::IPropInfo *propertyInfo, void *newValue);

  private:
    /**
     * applyValue: Find the property to update and configure it.
     *
     * @return void:
     */
    virtual void applyValue(void);

    /// The instance of the parent class that created this listener.
    SourceAdaptor *m_parent;

    /// Property Information object.
    imaqkit::IPropInfo *m_propInfo;

    /// The new value for integer properties.
    int16_t m_lastIntValue;

    /// The new value for double properties.
    double m_lastDoubleValue;

    /// The new value for string properties.
    std::string m_lastStrValue;
};

#endif
