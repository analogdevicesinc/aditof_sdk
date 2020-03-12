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
#ifndef __DEMO_TIMESTAMP_GET_FCN_HEADER__
#define __DEMO_TIMESTAMP_GET_FCN_HEADER__

#include "mwadaptorimaq.h"
#include "source_adaptor.h"

/**
 * Class TemperatureGetFcn
 *
 * @brief:  Provides a custom get function for the timestamp properties.
 *
 */
class TemperatureGetFcn : public imaqkit::IPropCustomGetFcn {

  public:
    //**************************************
    //* CONSTRUCTOR/DESTRUCTOR
    //**************************************
    /**
     * Constructor for TemperatureGetFcn class.
     *
     * @param parent: Handle to the instance of the IAdaptor class
     *                that is the parent of this object.
     */
    TemperatureGetFcn(SourceAdaptor *parent) : m_parent(parent){};

    // Destructor
    virtual ~TemperatureGetFcn(void){};

    // *******************************************************************
    // METHODS FOR CONFIGURING AND UPDATING DEMO SOURCE INPUTS.
    // *******************************************************************
    /**
     * This is the method the engine calls when the value of a property
     * that has an imaqkit::IPropCustomGetFcn associated with it.  This method
     * handles both the string and integer versions of the timestamp property.
     * It uses the imaqkit::IPropInfo to determine the storage type of the
     * property and then casts the value parameter appropriately.
     *
     * This method returns a timestamp as an example because it is easy to
     * generate and changes in a predictable manner.  A typical adaptor
     * would query the device's SDK to determine the current value of
     * a hardware parameter.
     *
     * @param propertyInfo: Property information object.
     * @param value: The current value of the property..
     *
     * @return void:
     */
    virtual void getValue(imaqkit::IPropInfo *propertyInfo, void *value);

  private:
    /// The instance of the parent class that created this listener.
    SourceAdaptor *m_parent;
};

#endif
