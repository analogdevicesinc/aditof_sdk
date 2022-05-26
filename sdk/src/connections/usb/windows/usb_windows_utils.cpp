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

#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif
#include <usb_windows_utils.h>

HRESULT
UsbWindowsUtils::UvcFindNodeAndGetControl(ExUnitHandle *handle,
                                          IBaseFilter **pVideoInputFilter) {
    HRESULT hr = E_FAIL;
    DWORD uiNumNodes;
    IKsNodeControl *pUnk = nullptr;
    GUID guidNodeType;

    handle->pKsTopologyInfo = nullptr;
    handle->pKsUnk = nullptr;

    if (*pVideoInputFilter != nullptr) {
        hr = (*pVideoInputFilter)
                 ->QueryInterface(__uuidof(IKsTopologyInfo),
                                  (VOID **)&handle->pKsTopologyInfo);

        if (FAILED(hr)) {
            LOG(WARNING) << "setVideoSetting - QueryInterface Error";
            (*pVideoInputFilter)->Release();
            *pVideoInputFilter = nullptr;

            return hr;
        }
    } else {
        LOG(ERROR) << "pVideoInputFilter should not be NULL";
        return E_POINTER;
    }

    // get nodes number in usb video device capture filter
    if (handle->pKsTopologyInfo->get_NumNodes(&uiNumNodes) == S_OK) {
        // go thru all nodes searching for the node of the
        // KSNODETYPE_DEV_SPECIFIC type, node of this type - represents
        // extension unit of the USB device
        for (UINT i = 0; i < uiNumNodes + 1; i++) {
            if (handle->pKsTopologyInfo->get_NodeType(i, &guidNodeType) ==
                S_OK) {
                if (guidNodeType == KSNODETYPE_DEV_SPECIFIC) {
                    // create node instance
                    hr = handle->pKsTopologyInfo->CreateNodeInstance(
                        i, __uuidof(IKsNodeControl), (void **)&pUnk);

                    // get IKsControl interface from node
                    if (hr == S_OK) {
                        hr = pUnk->QueryInterface(__uuidof(IKsControl),
                                                  (VOID **)&handle->pKsUnk);
                    }

                    // trying to read first control of the extension unit
                    if (hr == S_OK) {
                        handle->node = i;
                        return hr;
                    }
                }
            }
        }
    }

    return E_FAIL;
}

HRESULT UsbWindowsUtils::UvcExUnitSetProperty(ExUnitHandle *handle,
                                              ULONG selector,
                                              const uint8_t *buffer,
                                              ULONG nbBytes) {
    KSP_NODE s;
    ULONG ulBytesReturned;

    s.Property.Set = EXT_UNIT_GUID;
    s.Property.Id = selector;
    s.Property.Flags = KSPROPERTY_TYPE_SET | KSPROPERTY_TYPE_TOPOLOGY;
    s.NodeId = handle->node;

    uint8_t *nonConstBuffer = const_cast<uint8_t *>(buffer);
    return handle->pKsUnk->KsProperty(
        reinterpret_cast<PKSPROPERTY>(&s), sizeof(s),
        static_cast<LPVOID>(nonConstBuffer), nbBytes, &ulBytesReturned);
}

HRESULT UsbWindowsUtils::UvcExUnitGetProperty(ExUnitHandle *handle,
                                              ULONG selector, uint8_t *buffer,
                                              ULONG nbBytes) {
    KSP_NODE s;
    ULONG ulBytesReturned;

    s.Property.Set = EXT_UNIT_GUID;
    s.Property.Id = selector;
    s.Property.Flags = KSPROPERTY_TYPE_GET | KSPROPERTY_TYPE_TOPOLOGY;
    s.NodeId = handle->node;

    return handle->pKsUnk->KsProperty(reinterpret_cast<PKSPROPERTY>(&s),
                                      sizeof(s), static_cast<LPVOID>(buffer),
                                      nbBytes, &ulBytesReturned);
}

HRESULT UsbWindowsUtils::UvcExUnitReadBuffer(IBaseFilter *pVideoInputFilter,
                                             ULONG selector, int16_t id,
                                             uint32_t address, uint8_t *data,
                                             uint32_t bufferLength) {
    if (id < -1 || id > 255) {
        LOG(ERROR)
            << id
            << " is greater than the maximum size (255) accepted for an id";
        return E_INVALIDARG;
    }

    ExUnitHandle handle;
    HRESULT hr =
        UsbWindowsUtils::UvcFindNodeAndGetControl(&handle, &pVideoInputFilter);
    if (hr != S_OK) {
        LOG(WARNING) << "Failed to find node and get control";
        return hr;
    }

    uint8_t nbWrPacketBytes = sizeof(address) + (id > -1 ? 1 : 0);
    uint8_t posAddrInPacket = id > -1 ? 1 : 0;
    uint8_t packet[MAX_BUF_SIZE];
    size_t readBytes = 0;
    size_t readlength = 0;
    size_t addr = address;

    while (readBytes < bufferLength) {
        if (id > -1) {
            packet[0] = static_cast<uint8_t>(id);
        }
        *(reinterpret_cast<uint32_t *>(&packet[posAddrInPacket])) =
            static_cast<uint32_t>(addr);
        readlength = bufferLength - readBytes < MAX_BUF_SIZE
                         ? bufferLength - readBytes
                         : MAX_BUF_SIZE;
        packet[nbWrPacketBytes] = static_cast<uint8_t>(readlength);

        hr = UvcExUnitSetProperty(&handle, selector, packet, MAX_BUF_SIZE);
        if (FAILED(hr)) {
            LOG(WARNING) << "Failed to set property via UVC extension unit";
            return hr;
        }

        hr = UvcExUnitGetProperty(&handle, selector, packet, MAX_BUF_SIZE);
        if (FAILED(hr)) {
            LOG(WARNING) << "Failed to read a property via UVC extension unit";
            return hr;
        }
        memcpy(&data[readBytes], packet, readlength);
        readBytes += readlength;
        addr += readlength;
    }

    return S_OK;
}

HRESULT UsbWindowsUtils::UvcExUnitWriteBuffer(IBaseFilter *pVideoInputFilter,
                                              ULONG selector, int16_t id,
                                              uint32_t address,
                                              const uint8_t *data,
                                              uint32_t bufferLength) {
    if (id < -1 || id > 255) {
        LOG(ERROR)
            << id
            << " is greater than the maximum size (255) accepted for an id";
        return E_INVALIDARG;
    }

    ExUnitHandle handle;
    HRESULT hr =
        UsbWindowsUtils::UvcFindNodeAndGetControl(&handle, &pVideoInputFilter);
    if (hr != S_OK) {
        LOG(WARNING) << "Failed to find node and get control";
        return hr;
    }

    uint8_t nbLeadingBytes = sizeof(address) + (id > -1 ? 1 : 0);
    uint8_t posAddrInPacket = id > -1 ? 1 : 0;
    uint8_t packet[MAX_BUF_SIZE];
    size_t writeLen = 0;
    size_t writtenBytes = 0;

    if (id > -1) {
        packet[0] = static_cast<uint8_t>(id);
    }

    while (writtenBytes < bufferLength) {
        *(reinterpret_cast<uint32_t *>(&packet[posAddrInPacket])) = address;
        writeLen =
            bufferLength - writtenBytes > MAX_BUF_SIZE - (nbLeadingBytes + 1)
                ? MAX_BUF_SIZE - (nbLeadingBytes + 1)
                : bufferLength - writtenBytes;
        packet[nbLeadingBytes] = static_cast<uint8_t>(writeLen);
        memcpy(&packet[nbLeadingBytes + 1], data + writtenBytes, writeLen);

        hr = UvcExUnitSetProperty(&handle, selector, packet, MAX_BUF_SIZE);
        if (FAILED(hr)) {
            LOG(WARNING) << "Failed to write a packet via UVC extension unit";
            return hr;
        }
        writtenBytes += writeLen;
        address += static_cast<uint32_t>(writeLen);
    }

    return S_OK;
}
