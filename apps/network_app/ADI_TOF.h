/********************************************************************************/
/*  									                                        */
/* @file	ADI_TOF.h								                            */
/*										                                        */
/* @brief	ADI TOF SDK to capture raw data, process it and return S0,S1,BG,    */ 
/*			IR Image and the depth image		    		                    */
/*          									                                */
/*										                                        */
/* @author	Dhruvesh Gajaria	    				                            */
/*  										                                    */
/* @date	April 26, 2016		    			                                */
/*  										                                    */
/* Copyright(c) Analog Devices, Inc.		                            		*/
/*										                                        */
/********************************************************************************/
#pragma once

#define RAW_OUT

#ifndef Linux
#ifdef SDK_EXPORTS
#define SDK_API __declspec(dllexport) 
#else
#define SDK_API __declspec(dllimport) 
#endif
#else

#ifdef QTapp 
#include <QtCore/qglobal.h>
#endif
#ifdef ADI_TOF_LIBRARY
#  define ADI_TOFSHARED_EXPORT Q_DECL_EXPORT
#else
#  define ADI_TOFSHARED_EXPORT Q_DECL_IMPORT
#endif
#endif

/** An enum type
 * range - Depth range.
 */
enum range {
    NEAR_RANGE = 0,
    MID_RANGE = 1,
    FAR_RANGE = 2,
};


enum eFilterSel_t{
    FILT_SEL_NONE,
    FILT_SEL_GAUS5X5,
    FILT_SEL_GAUS5X5X2,
    FILT_SEL_BILATERAL7X7,
};

/** An enum type
 * cancellationPattern - Cancellation Pattern
 */
enum cancellationPattern{
    DISABLE_CANCELLATION = -1,
    ENABLE_CANCELLATION = 0,
    INTERFERENCE_CANCEL_MAX = 10, // SDK may not work for values 1-9, may add cancellation pattern 1-9 based on requirement
};




//! ADI_TOF - Top level SDK class
/*!
    ADI_TOF provides a programatic interface to the ADI ToF camera.  Through
    its set of APIs the camera can be configured and depth and IR images 
    are returned with various levels of processing performed.
*/
#ifndef Linux
class SDK_API ADI_TOF{
#else
#ifdef QTApp
class ADI_TOFSHARED_EXPORT ADI_TOF{
#else
class ADI_TOF{
#endif
#endif
protected:
	class Data;
	Data* data;
public:
	uint16 *punIR;
	uint16 *punDepth;
	uint16 *punConfidenceMap; //Currently not supported
	uint16 width; // Width of Depth, IR,
	uint16 height;
#ifdef RAW_OUT
	uint16 *punS0;
	uint16 *punS1;
	uint16 *punS2;
	uint16 *punRaw;
	uint16 rawWidth;
	uint16 rawHeight;
#endif

    /*************************************************************************************
     * \defgroup Initialization Constructor/destrutors
     * @{
     */

    ADI_TOF();
    ~ADI_TOF();



    //! Initializes the ToF system
    /*!
        @param playBackEn - bool - Enables playback which does not require hardware.
		@param cameraNum - int - Allows user to select camera when multiple cameras are connect
    */
    int Init(bool playBackEna = false, int cameraNum = -1);


    /**@}*/


    /*************************************************************************************
     * \defgroup FrameProcessing ToF Frame processing APIs
     * @{
     */

    //! ProcFrame - Called to receive the next frame image.
    /*!
        Upon return a depth image is available to use.  The buffer pointers punDepth 
        and punIR point to their respective image buffers which are arrays of 16 bit 
        unsigned integers values.  The buffers are valid until the next call to 
        ProcFrame is performed.  Also available are the image height and width.
        \param[out] punDepth - Pointer to the output detph image
        \param punIR - Pointer to the output IR image
        \param height - The output image height in pixels
        \param width - The output image width in pixels
        \return - Returns an integer data type error code. 0 => no error
                  A non-zero return values means the output data is not valid.
    */
    int ProcFrame(uint16 *punRawIn=NULL);

    //! rawProcFrame - Returns an image frame in raw format.
    /*!
        A raw frame is returned which has no processing performed.  The raw frame
        is one large image that includes all three exposure images S0, S1, and S2.
        The three exposure images have certain line and pixel interleaving which 
        typically requires separating before further processing is applied.  A
        buffer is allocated and a pointer is returned via function argument.
        \param punRawIn - Pointer to the returned raw image.
        \return - Returns an integer data type error code.  0 => No error.
                  A non-zero return values means the output data is not valid.
    */
    int rawProcFrame(uint16 *punRawIn);

    //! preProcFrame - Performs Pre-processing
    /*!
        Performs all the processing upto but not including depth calculation.
        This includes:
            Reorder - Re-arranges the pixel data from the raw input frame
            Defect pixel correction - Replaces defective pixels
            Grid conversion - Aligns the pixel data
			Background subtraction - Subtracts background from S0,S1 
            Roise reduction filter - 7x7 Bilateral or 7x7 Gaussian filters
            Small signal removal - Tags pixels with low signal for depth calculation
            
            
        is enabled through the Config APIs.
        \param punRawIn - Pointer to input raw frame from the AFE
        \param punS0 - Pointer to output S0 iamge
        \param punS1 - Pointer to output S1 iamge
        \param punBG - Pointer to output BG iamge
        \param punIR - Pointer to output IR iamge
    */
    int preProcFrame(uint16 *punRawIn=NULL);

    //! depthCalcProcFrame  - Performs the depth calculation, IR Calculation
    /*!
        There are two depth calculation methods.
        1. S1 / (S0 + S1) - Used when Sub pulse = LD pulse
            If (S0 + S1) is zero then depth = maximum depth
        2. S1/ S0 - Used when Sub pulse = 2 x LD pulse
            If S0 is zero then depth = maximum depth

		IR Caculation IR = S0 + S1 ;
			
        \param punS0 - Pointer to preprocessed S0 image
        \param punS1 - Pointer to preprocessed S1 image
        \param punDepth - Pointer  to output depth image
		
    */
    int depthProcFrame(uint16* S0 = NULL, uint16* S1 = NULL);

    //! postProcFrame - Performs the depth processing after depth calculation
    /*!
        The post processing includes non-linear correction, zero-point offset,
        shading offset correction and slope adjust.
        The input depth image is 14 bits and the output is 12 bits.
        \param punDepth - Pointer to input and output depth image.
    */
    int postProcFrame();

    //! Release - Free the current image buffers
    /*!
        This method is called when all processing for the current frame is complete.
        The frame image pointers are not gauranteed to be valid after this call.
        To preserve any frame data, the image buffers must be copied to user allocated
        buffers.

        TODO: What about the last depth buffer?  User cannot release and expect any 
        SDK offered temporal filter to work???
    */
    void Release();

    /**@}*/

    /*************************************************************************************
     * \defgroup ImgProc ToF image processing APIs
     * @{
     */



    //! setFilter - Enables a noise reduction filter
    /*!
        This method enables a noise reduction filter. There are two types of 
        filter that can be selected, a 7x7 bilateral or a 7x7 Gaussian.
        Background removal is performed during the NR filter. If no filter is
        selected then only the background removal is performed.

        Note :This function will be deprecated, replaced with setNRfilter()

        \Param filter - Noise reduction filter selection (values ??)
    */
    void setFilter(eFilterSel_t filter);

    //! setNRFilter - Enables/Disables the NR filter
    /*!This method enables/disbales noise reducing edge preserving filter.
    \Param bNRFilter - Turns On NR Filter if true, Turns OFF NR Filter if it is false
    */
    void setNRFilter(bool bNRFilter = true);

    //! setThreshold - Sets the small signal removal threshold
    /*!
        This method sets the threshold minimum IR level and used during 
        small signal removal. When the IR signal is below the threshold, 
        the output depth value is set to the maximum.

        \Param smSigRmvlThr - Threshold for minimum IR level
    */
    void setThreshold(uint16 smSigRmvlThr);


    /**@}*/


    /*************************************************************************************
     * \defgroup ConfigMode ToF mode configuration APIs
     * @{
     */

    //! setDepthRange - Sets the depth range
    /*!
        This method sets the depth range which loads the appropriate ISATG code and 
        calibration parameters for the given range.  There are currently 5 different
        ranges specified:
        0 - Near range (up to 2.5m)
        1 - Mid range (up to 6m)
        2 - Far range (up to 13m)
        3 - XFar range (up to 25m)
        4 - XNear range (up to 3.5m)

        \Param range - Range index value
        \Param level - Calibration parameter set index
    */
    void setDepthRange(range ecRange, uint16 level=0);

    //! getDepthRange - Get the current depth range
    /*!
        This method returns the currenly configured depth range.
    */
    range getDepthRange();

    //! getDepthMax - Get the maximum depth value of current depth range
    /*!
        This method returns maximum value of currently configured depth range.
    */
    uint16 getDepthMax();

    //!setCancellation - Configures the muliti-system cancellation feature
    /*!
    This method configures the multi-system cancellation with a 
    cancellation pattern index which selects a unique timing sequence 
    for the AFE. The cancellation feature is also activated by default, 
    but can be activated and deactivated without reloading the AFE code.

    \Param cCancellation - The cancellation sequence index

    \See activateCancellation
    */
    void setCancellation(cancellationPattern cCancellationNum = ENABLE_CANCELLATION);

    //!activateCancellation - Activate/deactivate multi-system cancellation
	/*!
        This method activates and deactivates the multi-system cancellation
        feature. Currently only supported by ADDI9030 raw mode. 
        
        Note: This function will be deprecated

        \Param activate - If true, activates or false, deactives the cancellation feature
    */
    void activateCancellation(bool activateCancellation = true);

    //!getCancellationStatus - Get the configured cancellation pattern index
    /*!
    This method returns the calcellation patter index number.Currently supported by ADDI9030/ADDI9033 raw mode and ADDI9043.
    */
    cancellationPattern getCancellationStatus(void); 

    //!setMultiRange - Enables/Disables the multi-range feature
    /*!
    This method enables or disables the nulti-range feature based on 
    the input argument setting. Multirange requires a file with the 
    filename "MultiRange.ini" to be in the path and the contents of
    the file must have valid data for the multi-range feature to be
    enabled.

    \Param bMR - Enable (= true) or disable (= false) the multi-range feature
    */
    void setMultiRange(bool bMR);

    //!getMultiRange - Get the index, pulse count and depth range of current Multi range pattern 
    /*!
    This method helps to get the index, pulse count and depth range of current Multi range pattern. 
    This is currently only supported by ADDI9030.
    */
    void getMultiRange(int *uiIndex = NULL, int *pulseCnt = NULL, int *depthRange = NULL);

    //!getCameraIntrinsic - Returns back Camera Intrinsic and Distortion Coefficient 
    /*!
    */
    void getCameraIntrinsic(float **pfIntrinsics, double **pDistortionCoeff);

    //! setRadialCorrection - Enable or disable TAL
    /*!
        This method enables or disabled the radial correction function.

        \Param isEnabled - True enables radial correction, false disables
    */
    void setRadialCorrection(bool isEnabled);

    /**@}*/


    /*************************************************************************************
     * \defgroup ConfigPulse ToF pulse timing configuration APIs
     * @{
     */

    //! setPulseCount - Sets the number of LD pulses per frame
    /*!
        This method sets the number of pulses per frame for both the laser
        and sub control.  The pulse count x6 is the actual number of pulses
        used for each sub phase and is an idication of the exposure time 
        for each frame.

        \Param value - The number of pulses per frame for the laser and sub signals.
                       The actual value is 6 x value
    */
    void setPulseCount(uint16 value);

    //! getPulseCount - Get the current pulse count setting
    /*!
        This method returns the current pulse count setting for the system which
        is 1/6 the actual count.
    */
    uint16 getPulseCount();

    //! setTALEnable - Enable or disable TAL
    /*!
        This method enables or disabled the TAL function.

        \Param bTALEna - True enables the TAL, false disables
    */
    void setTALEnable(bool bTALEna);

    //! getTALEnable - Get TAL status
    /*!
        This method returns the current status of TAL.
    */
    bool getTALEnable(void);

    //! setLaserRight - enables or disabled Right Laser or Laser Number 2
    /*!
        This method enables or disabled Right Laser or Laser Number 2
    */
    void setLaserRight(bool bLaserRightEna);

    //! setLaserLeft - enables or disabled Left Laser or Laser Number 1
    /*!
        This method enables or disabled Left Laser or Laser Number 1
    */
    void setLaserLeft(bool bLaserLeftEna);

    /**@}*/

    //! getErrorString - Returns correcponding error string for the error code
    /*!
        This method Returns correcponding error string for the error code
    */
    const char* getErrorString(int errCode);

    //! setLogConfig - This method is used to enable/disable logging and console printing
    /*!
        \Param consolePrintEnable - Switch to Enable/Disable error prints to console
        \Param logEnable - Switch to control if logs need to be recorded
    */
    void setLogConfig(bool consolePrintEnable=false, bool logEnable=true);

    //! programAFE - Program the AFE with a code block
    /*!
    This method downloads a code block to the AFE.  The code
    is specified by pointers to separate address and data arrays
    and the size of the code (number of entries). When this method
    is called the AFE is stopped, the new code is written and the
    AFE is restarted.

    \Param addr - Pointer to the array of code addresses
    \Param data - Pointer to the array of code data
    \Param noOfEntries - The number of code entries to be programmed.
    */
    void programAFE(uint16 *addr, uint16 *data, uint16 noOfEntries);

    unsigned short* getImage(unsigned short* punOutputBuffer = NULL);
};

