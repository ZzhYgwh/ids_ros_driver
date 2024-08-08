/*!
 * \file    autofeaturesmanager.h
 * \author  IDS Imaging Development Systems GmbH
 * \date    2022-06-01
 * \since   1.2.0
 *
 * \version 1.1.0
 *
 * Copyright (C) 2021 - 2022, IDS Imaging Development Systems GmbH.
 *
 * The information in this document is subject to change without notice
 * and should not be construed as a commitment by IDS Imaging Development Systems GmbH.
 * IDS Imaging Development Systems GmbH does not assume any responsibility for any errors
 * that may appear in this document.
 *
 * This document, or source code, is provided solely as an example of how to utilize
 * IDS Imaging Development Systems GmbH software libraries in a sample application.
 * IDS Imaging Development Systems GmbH does not assume any responsibility
 * for the use or reliability of any portion of this document.
 *
 * General permission to copy or modify is hereby granted.
 */

#ifndef AUTO_FEATURES_MANAGER_H
#define AUTO_FEATURES_MANAGER_H

#include <peak_ipl/peak_ipl.hpp>

#include <peak/peak.hpp>

#include <cstddef>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>

/*!
 * \brief The AutoFeaturesManager class controls ExposureAuto, GainAuto and BalanceWhiteAuto and all corresponding settings.
 *
 * \note: This class works directly on the remote devcice nodemap in a separate thread.
 *        When reading/writing gain values, the "GainSelector" node of the device is set to the correct value, then the
 *        actual "Gain" value is read/written. Afterwards, the "GainSelector" is reset to its previous value.
 *        To protect against race conditions while using remote device nodemap, make sure to hold
 *        the peak::core::NodeMap::Lock().
 */
class AutoFeaturesManager
{

public:
    /*!
     * \brief Creates an AutoFeaturesManager object. This class controls ExposureAuto, GainAuto and BalanceWhiteAuto and all corresponding settings.
     *
     * \param[in] nodemapRemoteDevice The nodemap of the remote device.
     *
     * \since 1.2
     */
    explicit AutoFeaturesManager(std::shared_ptr<peak::core::NodeMap> nodemapRemoteDevice = nullptr);

    /*!
     * \brief Destructor for the AutoFeaturesManager class.
     *
     * \since 1.2
     */
    ~AutoFeaturesManager() = default;

    /*!
     * \brief Saves the nodemap of the remote device for the access to the exposure and gain nodes.
     *
     * \param[in] nodemapRemoteDevice The nodemap of the remote device.
     *
     * \returns
     *          - true: Nodemap was saved and all read/write operations on the nodemap worked correctly
     *          - false: Nodemap was saved but the access to the nodemap failed
     *
     * \since 1.2
     */
    bool SetNodemapRemoteDevice(std::shared_ptr<peak::core::NodeMap> nodemapRemoteDevice);

    /*!
     * \brief Resets the image converter and releases all allocated buffers, e.g. to free memory after image size or format changes
     *
     * \since 2.1
     */
    void ResetImageConverter();

    /*!
     * \brief The three possible gain types in the remote device nodemap (AnalogGain, DigitalGain, Gain) and "None" if no gain is available
     *
     * \since 1.2
     */
    enum class GainType
    {
        Analog,
        Digital,
        Standard,
        Ipl,
        None
    };

    /*!
     * \brief The modes of the ExposureAuto calculation. "Once" calculates until the target is reached and then sets the calculation to "Off".
     *
     * \since 1.2
     */
    enum class ExposureAutoMode
    {
        Off,
        Continuous,
        Once
    };

    /*!
     * \brief The modes of the GainAuto calculation. "Once" calculates until the target is reached and then sets the calculation to "Off".
     *
     * \since 1.2
     */
    enum class GainAutoMode
    {
        Off,
        Continuous,
        Once
    };

    /*!
     * \brief The modes of the BalanceWhiteAuto calculation. "Once" calculates until the color gains are correct and then sets the calculation to "Off".
     *
     * \since 1.2
     */
    enum class BalanceWhiteAutoMode
    {
        Off,
        Continuous,
        Once
    };

    /*!
     * \brief The result of ProcessImage().
     *
     * - Success: ProcessImage() has successfully applied the auto features.
     * - Error: Remote device nodemap not set or all modes set to "Off" or not suitable source pixel format.
     * - Busy: ProcessImage() is called while calculator thread is still busy. ProcessImage() returns immediately.
     *
     * \since 1.2
     */
    enum class ProcessingResult
    {
        Success,
        Error,
        Busy
    };

    /*!
     * \brief Apply auto features. Use image to calculate correct values for exposure and gains.
     *
     * \param[in] image Image captured from device.
     *
     * \returns ProcessingResult. See description of ProcessingResult for further details.
     *
     * \since 1.2
     */
    ProcessingResult ProcessImage(const peak::ipl::Image& image);

    /*!
     * \brief Returns if AutoFeaturesManager is busy calculating the exposure and/or gain values.
     *
     * \returns
     *          - true: AutoFeatureManager is busy calculating ExposureAuto and/or GainAuto and/or BalanceWhiteAuto
     *          - false: AutoFeatureManager is not busy
     *
     * \since 1.2
     */
    bool IsCalculating() const;

    /*!
     * \brief Sets the number of images that are skipped between the calculation of the new exposure and gain values.
     *
     * \param[in] skipFrames Number of images to be skipped.
     *
     * \returns
     *          - true: Values from 1 to 10.
     *          - false: Values smaller than 1 or greater than 10.
     *
     * \since 1.2
     */
    bool SetSkipFrames(uint8_t skipFrames);

    /*!
     * \brief Returns the number of images that are skipped between the calculation of the new exposure and gain values.
     *
     * \returns Number of images that are skipped.
     *
     * \since 1.2
     */
    uint8_t GetSkipFrames() const;

    /*!
     * \brief Resets the SkipFramesCounter to 0.
     *
     * \since 1.2
     */
    void ResetSkipFramesCounter();

    /*!
     * \brief Returns the SkipFramesCounter.
     *
     * The SkipFramesCounter is incremented by 1 each time ProcessImage is called until it reaches the value (SkipFrames
     * + 1). Then it is reset to 0. A calculation is only done when the SkipFramesCounter = 0.
     *
     * \returns SkipFramesCounter.
     *
     * \since 1.2
     */
    uint8_t GetSkipFramesCounter() const;

    /********************************************/
    /* Functions for ExposureAuto and GainAuto  */
    /********************************************/

    /*!
     * \brief Sets the mode of the ExposureAuto calculation. See enum class ExposureAutoMode for further details.
     *
     * \param[in] exposureAutoMode Mode of the ExposureAuto calculation.
     *
     * \since 1.2
     */
    void SetExposureAutoMode(ExposureAutoMode exposureAutoMode);

    /*!
     * \brief Returns the mode of the ExposureAuto calculation. See enum class ExposureAutoMode for further details.
     *
     * \returns Mode of the ExposureAuto calculation.
     *
     * \since 1.2
     */
    ExposureAutoMode GetExposureAutoMode() const;

    /*!
     * \brief Returns if the ExposureAuto calculation is finished when it was started with mode ExposureAutoMode::Once.
     *
     * \returns
     *          - true: The calculation was started with ExposureAutoMode::Once and is finished. The return value
     *                  remains and is reset to false when a new ExposureAutoMode is set.
     *          - false: The calculation was started with ExposureAutoMode::Once and is still running.
     *          - false: The calculation was started with ExposureAutoMode::Continuous.
     *          - false: ExposureAutoMode was set to ExposureAutoMode::Off.
     *
     * \since 1.2
     */
    bool IsExposureAutoFinished() const;

    /*!
     * \brief Sets the mode of the GainAuto calculation. See enum class GainAutoMode for further details.
     *
     * \param[in] gainAutoMode Mode of the GainAuto calculation.
     *
     * \returns
     *          - true: Mode was set.
     *          - false: Mode could not be set because GainTypeAll = GainType::None
     *
     * \since 1.2
     */
    bool SetGainAutoMode(GainAutoMode gainAutoMode);

    /*!
     * \brief Returns the mode of the GainAuto calculation. See enum class GainAutoMode for further details.
     *
     * \returns Mode of the GainAuto calculation.
     *
     * \since 1.2
     */
    GainAutoMode GetGainAutoMode() const;

    /*!
     * \brief Returns if the GainAuto calculation is finished when it was started with mode GainAutoMode::Once.
     *
     * \returns
     *          - true: The calculation was started with GainAutoMode::Once and is finished. The return value remains
     *                  and is reset to false when a new GainAutoMode is set.
     *          - false: The calculation was started with GainAutoMode::Once and is still running.
     *          - false: The calculation was started with GainAutoMode::Continuous.
     *          - false: GainAutoMode was set to GainAutoMode::Off.
     *
     * \since 1.2
     */
    bool IsGainAutoFinished() const;

    /*!
     * \brief Returns the gain type used for the GainAuto calculation. See enum class GainType for further details.
     *
     * \returns Type used for the GainAuto calculation (Analog, Digital, Standard, None).
     *
     * \since 1.2
     */
    GainType GetGainTypeAll() const;

    /*!
     * \brief Sets the target value for the BrightnessAuto calculation.
     *
     * \param[in] target Target value between 0 and 255.
     *
     * \returns
     *          - true: Value is correct.
     *          - false: Value is outside range.
     *
     * \since 1.2
     */
    bool SetBrightnessAutoTarget(uint8_t target);

    /*!
     * \brief Returns the target value for the BrightnessAuto calculation.
     *
     * \returns BrightnessAutoTarget.
     *
     * \since 1.2
     */
    uint8_t GetBrightnessAutoTarget() const;

    /*!
     * \brief Returns the minimum of the target value for the BrightnessAuto calculation.
     *
     * \returns BrightnessAutoTargetMin.
     *
     * \since 1.2
     */
    uint8_t GetBrightnessAutoTargetMin() const;

    /*!
     * \brief Returns the maximum of the target value for the BrightnessAuto calculation.
     *
     * \returns BrightnessAutoTargetMax.
     *
     * \since 1.2
     */
    uint8_t GetBrightnessAutoTargetMax() const;

    /*!
     * \brief Returns the increment of the target value for the BrightnessAuto calculation.
     *
     * \returns BrightnessAutoTargetInc.
     *
     * \since 1.2
     */
    uint8_t GetBrightnessAutoTargetInc() const;

    /*!
     * \brief Sets the +- tolerance for the BrightnessAuto target value.
     *
     * \param[in] targetTolerance Tolerance value between 1 and 32.
     *
     * \returns
     *          - true: Value is correct.
     *          - false: Value is outside range.
     *
     * \since 1.2
     */
    bool SetBrightnessAutoTargetTolerance(uint8_t targetTolerance);

    /*!
     * \brief Returns the +- tolerance for the BrightnessAuto target value.
     *
     * \returns BrightnessAutoTargetTolerance.
     *
     * \since 1.2
     */
    uint8_t GetBrightnessAutoTargetTolerance() const;

    /*!
     * \brief Returns the minimum of the +- tolerance for the BrightnessAuto target value.
     *
     * \returns BrightnessAutoTargetToleranceMin.
     *
     * \since 1.2
     */
    uint8_t GetBrightnessAutoTargetToleranceMin() const;

    /*!
     * \brief Returns the maximum of the +- tolerance for the BrightnessAuto target value.
     *
     * \returns BrightnessAutoTargetToleranceMax.
     *
     * \since 1.2
     */
    uint8_t GetBrightnessAutoTargetToleranceMax() const;

    /*!
     * \brief Returns the increment of the +- tolerance for the BrightnessAuto target value.
     *
     * \returns BrightnessAutoTargetToleranceInc.
     *
     * \since 1.2
     */
    uint8_t GetBrightnessAutoTargetToleranceInc() const;

    /*!
     * \brief Sets the percentile value for the BrightnessAuto calculation.
     *
     * \param[in] percentile Percentile value between 0% and 100%.
     *
     * \returns
     *          - true: Value is correct.
     *          - false: Value is outside range.
     *
     * \since 1.2
     */
    bool SetBrightnessAutoPercentile(double percentile);

    /*!
     * \brief Returns the percentile value for the BrightnessAuto calculation.
     *
     * \returns BrightnessAutoPercentile.
     *
     * \since 1.2
     */
    double GetBrightnessAutoPercentile() const;

    /*!
     * \brief Returns the minimum of the percentile value for the BrightnessAuto calculation.
     *
     * \returns BrightnessAutoPercentileMin.
     *
     * \since 1.2
     */
    double GetBrightnessAutoPercentileMin() const;

    /*!
     * \brief Returns the maximum of the percentile value for the BrightnessAuto calculation.
     *
     * \returns BrightnessAutoPercentileMax.
     *
     * \since 1.2
     */
    double GetBrightnessAutoPercentileMax() const;

    /*!
     * \brief Returns the increment of the percentile value for the BrightnessAuto calculation.
     *
     * \returns BrightnessAutoPercentileInc.
     *
     * \since 1.2
     */
    double GetBrightnessAutoPercentileInc() const;

    /*!
     * \brief Sets the ROI for the BrightnessAuto calculation relative to the current camera ROI.
     *
     * \param[in] x OffsetX
     * \param[in] y OffsetY
     * \param[in] width Width
     * \param[in] height Height
     *
     * \returns
     *          - true: ROI was set.
     *          - false: ROI is outside the image ROI or nodemap access error.
     *
     * \since 1.2
     */
    bool SetBrightnessAutoRoi(size_t x, size_t y, size_t width, size_t height);

    /*!
     * \brief Returns the ROI for the BrightnessAuto calculation relative to the current camera ROI.
     *
     * \param[out] x OffsetX
     * \param[out] y OffsetY
     * \param[out] width Width
     * \param[out] height Height
     *
     * \since 1.2
     */
    void GetBrightnessAutoRoi(size_t& x, size_t& y, size_t& width, size_t& height) const;

    /*!
     * \brief Returns the median value of the last BrightnessAuto calculation.
     *
     * \returns Median value.
     *
     * \since 1.2
     */
    uint8_t GetLastBrightnessAutoMedian() const;

    /****************************************/
    /* Functions for BalanceWhiteAuto       */
    /****************************************/

    /*!
     * \brief Sets the mode of the BalanceWhiteAuto calculation. See enum class BalanceWhiteAutoMode for further details.
     *
     * \param[in] balanceWhiteAutoMode Mode of the BalanceWhiteAuto calculation.
     *
     * \returns
     *          - true: Mode was set.
     *          - false: Mode could not be set because GainTypeColor = GainType::None
     *
     * \since 1.2
     */
    bool SetBalanceWhiteAutoMode(BalanceWhiteAutoMode balanceWhiteAutoMode);

    /*!
     * \brief Returns the mode of the BalanceWhiteAuto calculation. See enum class BalanceWhiteAutoMode for further details.
     *
     * \returns Mode of the BalanceWhiteAuto calculation.
     *
     * \since 1.2
     */
    BalanceWhiteAutoMode GetBalanceWhiteAutoMode() const;

    /*!
     * \brief Returns if the BalanceWhiteAuto calculation is finished when it was started with mode BalanceWhiteAutoMode::Once.
     *
     * \returns
     *          - true: The calculation was started with BalanceWhiteAutoMode::Once and is finished. The return value
     *                  remains and is reset to false when a new BalanceWhiteAutoMode is set.
     *          - false: The calculation was started with BalanceWhiteAutoMode::Once and is still running.
     *          - false: The calculation was started with BalanceWhiteAutoMode::Continuous.
     *          - false: BalanceWhiteAutoMode was set to BalanceWhiteAutoMode::Off.
     *
     * \since 1.2
     */
    bool IsBalanceWhiteAutoFinished() const;

    /*!
     * \brief Returns the gain type used for the BalanceWhiteAuto calculation. See enum class GainType for further details.
     *
     * \returns Type used for the BalanceWhiteAuto calculation (Analog, Digital, Standard, None).
     *
     * \since 1.2
     */
    GainType GetGainTypeColor() const;

    /*!
     * \brief Sets the ROI for the BalanceWhiteAuto calculation relative to the current camera ROI.
     *
     * \param[in] x OffsetX
     * \param[in] y OffsetY
     * \param[in] width Width
     * \param[in] height Height
     *
     * \returns
     *          - true: ROI was set.
     *          - false: BalanceWhiteAuto calculation not possible because GainTypeColor = GainType::None.
     *          - false: ROI is outside the image ROI.
     *          - false: Nodemap access error.
     *
     * \since 1.2
     */
    bool SetBalanceWhiteAutoRoi(size_t x, size_t y, size_t width, size_t height);

    /*!
     * \brief Returns the ROI for the BalanceWhiteAuto calculation relative to the current camera ROI.
     *
     * \param[out] x OffsetX
     * \param[out] y OffsetY
     * \param[out] width Width
     * \param[out] height Height
     *
     * \returns
     *          - true: ROI can be read.
     *          - false: BalanceWhiteAuto calculation not possible because GainTypeColor = GainType::None.
     *
     * \since 1.2
     */
    bool GetBalanceWhiteAutoRoi(size_t& x, size_t& y, size_t& width, size_t& height) const;

    /*!
     * \brief Returns the red/green/blue average values of the last BalanceWhiteAuto calculation.
     *
     * \param[out] averageRed Last red average value.
     * \param[out] averageGreen Last green average value.
     * \param[out] averageBlue Last blue average value.
     *
     * \returns
     *          - true: Values are valid.
     *          - false: BalanceWhiteAuto calculation not possible because GainTypeColor = GainType::None
     *
     * \since 1.2
     */
    bool GetLastBalanceWhiteAutoAverages(uint8_t& averageRed, uint8_t& averageGreen, uint8_t& averageBlue) const;

    /*!
     * \brief Sets an IPL gain controller pointer for IPL BalanceWhiteAuto calculation.
     *
     * \param[in] gainIPL pointer to IPL gain controller.
     *
     * \since 2.1
     */
    void SetGainControllerIPL(peak::ipl::Gain* gainIPL);

private:
    mutable std::recursive_mutex m_publicInterfaceMutex;
    std::mutex m_gainMutex;

    std::shared_ptr<peak::core::NodeMap> m_nodemapRemoteDevice;

    std::future<void> m_calculationTask;

    // Copy of the last image for median calculation
    peak::ipl::Image m_image;

    // Skip frames to avoid oscillation
    uint8_t m_skipFrames;
    uint8_t m_skipFramesCounter;

    // Image converter for internal conversions to other formats
    std::unique_ptr<peak::ipl::ImageConverter> m_imageConverter;

    /******************************************/
    /* Features for ExposureAuto and GainAuto */
    /******************************************/

    ExposureAutoMode m_exposureAutoMode;

    // This flag indicates that the ExposureAuto calculation has finished after "once" was started
    bool m_exposureAutoFinished;

    GainAutoMode m_gainAutoMode;

    // This flag indicates that the GainAuto calculation has finished after "once" was started
    bool m_gainAutoFinished;

    // The destination brightness value and the +- tolerance
    uint8_t m_brightnessAutoTarget;
    uint8_t m_brightnessAutoTargetTolerance;

    // The percentage of pixels with a value lower than target
    double m_brightnessAutoPercentile;

    // The calculated brightness median of the last image
    uint8_t m_brightnessAutoMedian;

    // Exposure limits
    double m_exposureTimeMin;
    double m_exposureTimeMax;
    double m_exposureTimeLimit;

    // Gains for brightness
    GainType m_gainTypeAll;
    double m_gainMin;
    double m_gainMax;
    double m_gainLimit;

    // Brightness ROI inside the sensor ROI
    size_t m_brightnessAutoOffsetX;
    size_t m_brightnessAutoOffsetY;
    size_t m_brightnessAutoWidth;
    size_t m_brightnessAutoHeight;

    /****************************************/
    /* Features for  BalanceWhiteAuto       */
    /****************************************/

    BalanceWhiteAutoMode m_balanceWhiteAutoMode;

    // This flag indicates that the BalanceWhiteAuto calculation has finished after "once" was started
    bool m_balanceWhiteAutoFinished;

    // The calculated averages of the last image
    uint8_t m_balanceWhiteAutoAverageRed;
    uint8_t m_balanceWhiteAutoAverageGreen;
    uint8_t m_balanceWhiteAutoAverageBlue;

    GainType m_gainTypeColor;
    double m_gainRedMin;
    double m_gainRedMax;
    double m_gainGreenMin;
    double m_gainGreenMax;
    double m_gainBlueMin;
    double m_gainBlueMax;

    // Brightness ROI inside the sensor ROI
    size_t m_balanceWhiteAutoOffsetX;
    size_t m_balanceWhiteAutoOffsetY;
    size_t m_balanceWhiteAutoWidth;
    size_t m_balanceWhiteAutoHeight;

    /****************************************/
    /* Internal                             */
    /****************************************/

    peak::ipl::Gain* m_gainIPL{};

    static void CalculatorThread(AutoFeaturesManager* parent);

    uint8_t CalculateBrightnessAutoMedian(const peak::ipl::Image& image);

    bool IsPixelFormatSupported(const peak::ipl::Image& image);

    void CalculateExposureAutoAndGainAuto();
    void CalculateExposureAuto();
    void CalculateGainAuto();
    void CalculateBalanceWhiteAuto();
};


#endif // AUTO_FEATURES_MANAGER_H
