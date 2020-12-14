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
#pragma once

#include <dshow.h>
#include <initguid.h>
#include <strmif.h>
// begin of RDK_extension unit
// Headers added to support extension unit
#include <iostream>
#include <ks.h>
#include <ksmedia.h>
#include <ksproxy.h>
#include <vidcap.h>
// end of RDK_extension unit

#define MAX_PACKET_SIZE 58
#define MAX_BUF_SIZE (MAX_PACKET_SIZE + 2)

DEFINE_GUID(CLSID_SampleGrabber, 0xc1f400a0, 0x3f08, 0x11d3, 0x9f, 0x0b, 0x00,
            0x60, 0x08, 0x03, 0x9e, 0x37);
DEFINE_GUID(IID_ISampleGrabber, 0x6b652fff, 0x11fe, 0x4fce, 0x92, 0xad, 0x02,
            0x66, 0xb5, 0xd7, 0xc7, 0x8f);
DEFINE_GUID(CLSID_NullRenderer, 0xc1f400a4, 0x3f08, 0x11d3, 0x9f, 0x0b, 0x00,
            0x60, 0x08, 0x03, 0x9e, 0x37);
DEFINE_GUID(MEDIASUBTYPE_BY8, 0x20385942, 0x0000, 0x0010, 0x80, 0x00, 0x00,
            0xaa, 0x00, 0x38, 0x9b, 0x71);
DEFINE_GUID(MEDIASUBTYPE_Y16, 0x20363159, 0x0000, 0x0010, 0x80, 0x00, 0x00,
            0xAA, 0x00, 0x38, 0x9B, 0x71);
DEFINE_GUID(MEDIASUBTYPE_GREY, 0x59455247, 0x0000, 0x0010, 0x80, 0x00, 0x00,
            0xaa, 0x00, 0x38, 0x9b, 0x71);
DEFINE_GUID(MEDIASUBTYPE_Y8, 0x20203859, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xaa,
            0x00, 0x38, 0x9b, 0x71);
DEFINE_GUID(MEDIASUBTYPE_Y800, 0x30303859, 0x0000, 0x0010, 0x80, 0x00, 0x00,
            0xaa, 0x00, 0x38, 0x9b, 0x71);

static const GUID EXT_UNIT_GUID = {0xFFFFFFFF, 0xFFFF, 0xFFFF, 0xFF, 0xFF, 0xFF,
                                   0xFF,       0xFF,   0xFF,   0xFF, 0xFF};

#pragma comment(lib, "strmiids")

interface ISampleGrabberCB : public IUnknown {
    virtual HRESULT STDMETHODCALLTYPE SampleCB(double SampleTime,
                                               IMediaSample *pSample) = 0;

    virtual HRESULT STDMETHODCALLTYPE BufferCB(double SampleTime, BYTE *pBuffer,
                                               LONG BufferLen) = 0;

    virtual ~ISampleGrabberCB() {}
};

interface ISampleGrabber : public IUnknown {
    virtual HRESULT STDMETHODCALLTYPE SetOneShot(BOOL OneShot) = 0;

    virtual HRESULT STDMETHODCALLTYPE SetMediaType(
        const AM_MEDIA_TYPE *pType) = 0;

    virtual HRESULT STDMETHODCALLTYPE GetConnectedMediaType(AM_MEDIA_TYPE *
                                                            pType) = 0;

    virtual HRESULT STDMETHODCALLTYPE SetBufferSamples(BOOL BufferThem) = 0;

    virtual HRESULT STDMETHODCALLTYPE GetCurrentBuffer(LONG * pBufferSize,
                                                       LONG * pBuffer) = 0;

    virtual HRESULT STDMETHODCALLTYPE GetCurrentSample(IMediaSample *
                                                       *ppSample) = 0;

    virtual HRESULT STDMETHODCALLTYPE SetCallback(
        ISampleGrabberCB * pCallback, LONG WhichMethodToCallback) = 0;

    virtual ~ISampleGrabber() {}
};

////////////////////////////  CALLBACK  ////////////////////////////////
// Callback class
class SampleGrabberCallback : public ISampleGrabberCB {
  public:
    //------------------------------------------------
    SampleGrabberCallback() {
        InitializeCriticalSection(&critSection);
        newFrame = false;
    }

    //------------------------------------------------
    ~SampleGrabberCallback() { DeleteCriticalSection(&critSection); }

    //------------------------------------------------
    STDMETHODIMP_(ULONG) AddRef() { return 1; }
    STDMETHODIMP_(ULONG) Release() { return 2; }

    //------------------------------------------------
    STDMETHODIMP QueryInterface(REFIID riid, void **ppvObject) {
        *ppvObject = static_cast<ISampleGrabberCB *>(this);
        return S_OK;
    }

    // This method is meant to have less overhead
    //------------------------------------------------
    STDMETHODIMP SampleCB(double Time, IMediaSample *pSample) {
        EnterCriticalSection(&critSection);
        newFrame = true;
        LeaveCriticalSection(&critSection);
        return S_OK;
    }

    // This method is meant to have more overhead
    STDMETHODIMP BufferCB(double Time, BYTE *pBuffer, long BufferLen) {
        EnterCriticalSection(&critSection);
        newFrame = true;
        LeaveCriticalSection(&critSection);
        return S_OK;
    }

    bool newFrame;
    CRITICAL_SECTION critSection;
};

struct UsbHandle {
    ICaptureGraphBuilder2 *pCaptureGraph; // Capture graph builder object
    IGraphBuilder *pGraph;                // Graph builder object
    IMediaControl *pControl;              // Media control object
    IBaseFilter *pVideoInputFilter;       // Video Capture filter
    IBaseFilter *pGrabberF;
    IBaseFilter *pDestFilter;
    IAMStreamConfig *streamConf;
    ISampleGrabber *pGrabber; // Grabs frame
    AM_MEDIA_TYPE *pAmMediaType;
    IMediaEventEx *pMediaEvent;
    SampleGrabberCallback *pCB;
    GUID videoType;

    UsbHandle()
        : pCaptureGraph{nullptr}, pGraph{nullptr}, pControl{nullptr},
          pVideoInputFilter{nullptr}, pGrabberF{nullptr}, pDestFilter{nullptr},
          streamConf{nullptr}, pGrabber{nullptr}, pAmMediaType{nullptr},
          pMediaEvent{nullptr}, pCB{nullptr} {}
};

struct ExUnitHandle {
    IKsTopologyInfo *pKsTopologyInfo;
    IKsControl *pKsUnk;
    ULONG node;

    ExUnitHandle() : pKsTopologyInfo(nullptr), pKsUnk(nullptr), node(0) {}
    ~ExUnitHandle() {
        if (pKsTopologyInfo) {
            pKsTopologyInfo->Release();
            pKsTopologyInfo = nullptr;
        }
    }
};

class UsbWindowsUtils {
  public:
    static HRESULT UvcFindNodeAndGetControl(ExUnitHandle *handle,
                                            IBaseFilter **pVideoInputFilter);

    static HRESULT UvcExUnitSetProperty(ExUnitHandle *handle, ULONG selector,
                                        const uint8_t *buffer, ULONG nbBytes);

    static HRESULT UvcExUnitGetProperty(ExUnitHandle *handle, ULONG selector,
                                        uint8_t *buffer, ULONG nbBytes);

    static HRESULT UvcExUnitReadBuffer(IBaseFilter *pVideoInputFilter,
                                       ULONG selector, int16_t id,
                                       uint32_t address, uint8_t *data,
                                       uint32_t bufferLength);

    static HRESULT UvcExUnitWriteBuffer(IBaseFilter *pVideoInputFilter,
                                        ULONG selector, int16_t id,
                                        uint32_t address, const uint8_t *data,
                                        uint32_t bufferLength);
};
