/********************************************************************************/
/*																				*/
/* @file	Device.h (for Windows OS)
 */
/*																				*/
/* @brief	Platform/OS specific IO communication file
 */
/*																				*/
/* @author	Dhruvesh Gajaria
 */
/*																				*/
/* @date    November 15, 2016
 */
/*																				*/
/* Copyright(c) Analog Devices, Inc.
 */
/*																				*/
/********************************************************************************/
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
