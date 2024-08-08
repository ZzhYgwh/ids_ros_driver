/*!
 * \file    autofeaturesmanager.cpp
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

#include "autofeaturesmanager.h"

#include <peak_ipl/peak_ipl.hpp>

#include <peak/peak.hpp>

#include <cstddef>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>

namespace
{
template <class T>
bool are_almost_equal(T x, T y, int ulp)
{
    return std::abs(x - y) <= std::numeric_limits<T>::epsilon() * std::max(std::abs(x), std::abs(y)) * ulp;
}
} // namespace

template <typename CallableType>
class RaiiGuard
{
public:
    RaiiGuard(const CallableType& funcy)
        : m_func{ funcy }
    {}
    ~RaiiGuard()
    {
        m_func();
    }

private:
    CallableType m_func;
};

template <typename CallableType>
auto makeRaiiGuard(const CallableType& funcy) -> RaiiGuard<CallableType>
{
    return RaiiGuard<CallableType>(funcy);
}


AutoFeaturesManager::AutoFeaturesManager(std::shared_ptr<peak::core::NodeMap> nodemapRemoteDevice)
{
    m_skipFrames = 2;
    m_skipFramesCounter = 0;

    m_imageConverter = std::make_unique<peak::ipl::ImageConverter>();

    /****************************************/
    /* Settings for ExposureAuto/GainAuto   */
    /****************************************/

    m_exposureAutoMode = ExposureAutoMode::Off;
    m_gainAutoMode = GainAutoMode::Off;

    m_brightnessAutoTarget = 150;
    m_brightnessAutoTargetTolerance = 3;
    m_brightnessAutoPercentile = 13.0;

    m_exposureTimeMin = 0;
    m_exposureTimeMax = 0;
    m_exposureTimeLimit = 0;

    m_brightnessAutoOffsetX = 0;
    m_brightnessAutoOffsetY = 0;
    m_brightnessAutoWidth = 0;
    m_brightnessAutoHeight = 0;

    m_gainTypeAll = GainType::None;

    m_brightnessAutoMedian = 0;

    m_exposureAutoFinished = false;
    m_gainAutoFinished = false;

    /****************************************/
    /* Settings for BalanceWhiteAuto        */
    /****************************************/

    m_balanceWhiteAutoMode = BalanceWhiteAutoMode::Off;

    m_balanceWhiteAutoOffsetX = 0;
    m_balanceWhiteAutoOffsetY = 0;
    m_balanceWhiteAutoWidth = 0;
    m_balanceWhiteAutoHeight = 0;

    m_gainTypeColor = GainType::None;

    m_balanceWhiteAutoAverageRed = 0;
    m_balanceWhiteAutoAverageGreen = 0;
    m_balanceWhiteAutoAverageBlue = 0;

    m_balanceWhiteAutoFinished = false;

    SetNodemapRemoteDevice(nodemapRemoteDevice);
}


bool AutoFeaturesManager::SetNodemapRemoteDevice(std::shared_ptr<peak::core::NodeMap> nodemapRemoteDevice)
{
    const std::lock_guard<std::recursive_mutex> lock(m_publicInterfaceMutex);
    m_nodemapRemoteDevice = nodemapRemoteDevice;

    if (!m_nodemapRemoteDevice)
    {
        return true;
    }

    const auto nodeMapLock = m_nodemapRemoteDevice->Lock();

    /****************************************/
    /* Get exposure values                  */
    /****************************************/

    try
    {
        // Get min and max exposure time
        m_exposureTimeMin = m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("ExposureTime")
                                ->Minimum();
        m_exposureTimeMax = m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("ExposureTime")
                                ->Maximum();
    }
    catch (const std::exception&)
    {
        return false;
    }

    {
        const auto gainSelectorValueGuard = [this] {
            // restore original gain selection when exiting scope
            const auto orginalGainSelection = m_nodemapRemoteDevice
                                                  ->FindNode<peak::core::nodes::EnumerationNode>(
                                                      "GainSelector")
                                                  ->CurrentEntry();

            return makeRaiiGuard([this, orginalGainSelection]() noexcept {
                try
                {
                    m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                        ->SetCurrentEntry(orginalGainSelection);
                }
                catch (const std::exception&)
                {}
            });
        }();


        /********************************/
        /* Get all supported gain types */
        /********************************/

        bool hasAnalogAll = false;
        bool hasDigitalAll = false;
        bool hasAll = false;
        bool hasAnalogColor = false;
        bool hasDigitalColor = false;
        bool hasColor = false;

        try
        {
            for (const auto& entry :
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")->Entries())
            {
                if ("AnalogAll" == entry->StringValue())
                {
                    hasAnalogAll = true;
                }
                else if ("DigitalAll" == entry->StringValue())
                {
                    hasDigitalAll = true;
                }
                else if ("All" == entry->StringValue())
                {
                    hasAll = true;
                }
                else if ("AnalogRed" == entry->StringValue())
                {
                    hasAnalogColor = true;
                }
                else if ("DigitalRed" == entry->StringValue())
                {
                    hasDigitalColor = true;
                }
                else if ("Red" == entry->StringValue())
                {
                    hasColor = true;
                }
            }
        }
        catch (const std::exception&)
        {}

        m_gainTypeAll = GainType::Ipl;

        try
        {
            if (hasAnalogAll)
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                    ->SetCurrentEntry("AnalogAll");
                m_gainTypeAll = GainType::Analog;
            }
            else if (hasDigitalAll)
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                    ->SetCurrentEntry("DigitalAll");
                m_gainTypeAll = GainType::Digital;
            }
            else if (hasAll)
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                    ->SetCurrentEntry("All");
                m_gainTypeAll = GainType::Standard;
            }
        }
        catch (const std::exception&)
        {}

        if (GainType::Ipl != m_gainTypeAll)
        {
            try
            {
                m_gainMin = m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain")->Minimum();
                m_gainMax = m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain")->Maximum();
                m_gainLimit = m_gainMax;
            }
            catch (const std::exception&)
            {
                m_gainMin = 1.0;
                m_gainMax = 1.0;
                m_gainLimit = 1.0;
            }
        }
        else
        {
            if (m_gainIPL)
            {
                m_gainMin = static_cast<double>(m_gainIPL->MasterGainMin());
                m_gainMax = static_cast<double>(m_gainIPL->MasterGainMax());
                m_gainLimit = m_gainMax;
            }
            else
            {
                m_gainMin = 1.0;
                m_gainMax = 1.0;
                m_gainLimit = 1.0;
            }
        }

        /****************************************/
        /* Red gain                             */
        /****************************************/

        m_gainTypeColor = GainType::Ipl;

        try
        {
            if (hasAnalogColor)
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                    ->SetCurrentEntry("AnalogRed");
                m_gainTypeColor = GainType::Analog;
            }
            else if (hasDigitalColor)
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                    ->SetCurrentEntry("DigitalRed");
                m_gainTypeColor = GainType::Digital;
            }
            else if (hasColor)
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                    ->SetCurrentEntry("Red");
                m_gainTypeColor = GainType::Standard;
            }
        }
        catch (const std::exception&)
        {}

        if (GainType::Ipl != m_gainTypeColor)
        {
            try
            {
                m_gainRedMin = m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain")
                                   ->Minimum();
                m_gainRedMax = m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain")
                                   ->Maximum();
            }
            catch (const std::exception&)
            {
                m_gainRedMin = 1.0;
                m_gainRedMax = 1.0;
            }
        }
        else
        {
            if (m_gainIPL)
            {
                m_gainRedMin = static_cast<double>(m_gainIPL->RedGainMin());
                m_gainRedMax = static_cast<double>(m_gainIPL->RedGainMax());
            }
            else
            {
                m_gainRedMin = 1.0;
                m_gainRedMax = 1.0;
            }
        }

        /****************************************/
        /* Green gain                           */
        /****************************************/

        if (GainType::Ipl != m_gainTypeColor)
        {
            try
            {
                if (hasAnalogColor)
                {
                    m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                        ->SetCurrentEntry("AnalogGreen");
                }
                else if (hasDigitalColor)
                {
                    m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                        ->SetCurrentEntry("DigitalGreen");
                }
                else if (hasColor)
                {
                    m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                        ->SetCurrentEntry("Green");
                }
            }
            catch (const std::exception&)
            {}

            try
            {
                m_gainGreenMin = m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain")
                                     ->Minimum();
                m_gainGreenMax = m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain")
                                     ->Maximum();
            }
            catch (const std::exception&)
            {
                m_gainGreenMin = 1.0;
                m_gainGreenMax = 1.0;
            }
        }
        else
        {
            if (m_gainIPL)
            {
                m_gainGreenMin = static_cast<double>(m_gainIPL->GreenGainMin());
                m_gainGreenMax = static_cast<double>(m_gainIPL->GreenGainMax());
            }
            else
            {
                m_gainGreenMin = 1.0;
                m_gainGreenMax = 1.0;
            }
        }

        /****************************************/
        /* Blue gain                            */
        /****************************************/

        if (GainType::Ipl != m_gainTypeColor)
        {
            try
            {
                if (hasAnalogColor)
                {
                    m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                        ->SetCurrentEntry("AnalogBlue");
                }
                else if (hasDigitalColor)
                {
                    m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                        ->SetCurrentEntry("DigitalBlue");
                }
                else if (hasColor)
                {
                    m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                        ->SetCurrentEntry("Blue");
                }
            }
            catch (const std::exception&)
            {}

            try
            {
                m_gainBlueMin = m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain")
                                    ->Minimum();
                m_gainBlueMax = m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain")
                                    ->Maximum();
            }
            catch (const std::exception&)
            {
                m_gainBlueMin = 1.0;
                m_gainBlueMax = 1.0;
            }
        }
        else
        {
            if (m_gainIPL)
            {
                m_gainBlueMin = static_cast<double>(m_gainIPL->BlueGainMin());
                m_gainBlueMax = static_cast<double>(m_gainIPL->BlueGainMax());
            }
            else
            {
                m_gainBlueMin = 1.0;
                m_gainBlueMax = 1.0;
            }
        }
    }

    /****************************************/
    /* Get ROI values                       */
    /****************************************/

    try
    {
        m_brightnessAutoOffsetX = size_t{ 0 };
        m_brightnessAutoOffsetY = size_t{ 0 };

        // Get current image size
        m_brightnessAutoWidth = static_cast<size_t>(
            m_nodemapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Width")->Value());
        m_brightnessAutoHeight = static_cast<size_t>(
            m_nodemapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Height")->Value());

        m_balanceWhiteAutoOffsetX = m_brightnessAutoOffsetX;
        m_balanceWhiteAutoOffsetY = m_brightnessAutoOffsetY;
        m_balanceWhiteAutoWidth = m_brightnessAutoWidth;
        m_balanceWhiteAutoHeight = m_brightnessAutoHeight;
    }
    catch (const std::exception&)
    {
        return false;
    }

    return true;
}


void AutoFeaturesManager::ResetImageConverter()
{
    // reset image converter and free all allocated buffers by creating a new image converter object
    m_imageConverter = std::make_unique<peak::ipl::ImageConverter>();
}


bool AutoFeaturesManager::SetSkipFrames(uint8_t skipFrames)
{
    std::lock_guard<std::recursive_mutex> lock(m_publicInterfaceMutex);

    if ((skipFrames < 1) || (skipFrames > 10))
    {
        return false;
    }

    m_skipFrames = skipFrames;

    return true;
}


uint8_t AutoFeaturesManager::GetSkipFrames() const
{
    std::lock_guard<std::recursive_mutex> lock(m_publicInterfaceMutex);

    return m_skipFrames;
}


void AutoFeaturesManager::ResetSkipFramesCounter()
{
    m_skipFramesCounter = 0;
}


uint8_t AutoFeaturesManager::GetSkipFramesCounter() const
{
    return m_skipFramesCounter;
}

AutoFeaturesManager::ProcessingResult AutoFeaturesManager::ProcessImage(const peak::ipl::Image& image)
{
    std::lock_guard<std::recursive_mutex> lock(m_publicInterfaceMutex);

    if (!m_nodemapRemoteDevice)
    {
        return ProcessingResult::Error;
    }

    if ((ExposureAutoMode::Off == m_exposureAutoMode) && (GainAutoMode::Off == m_gainAutoMode)
        && (BalanceWhiteAutoMode::Off == m_balanceWhiteAutoMode))
    {
        return ProcessingResult::Error;
    }

    if (!IsPixelFormatSupported(image))
    {
        return ProcessingResult::Error;
    }

    if (IsCalculating())
    {
        return ProcessingResult::Busy;
    }

    if (0 == m_skipFramesCounter)
    {
        m_image = image.Clone();

        m_calculationTask = std::async(std::launch::async, AutoFeaturesManager::CalculatorThread, this);
    }

    m_skipFramesCounter++;

    if ((m_skipFrames + 1) == m_skipFramesCounter)
    {
        m_skipFramesCounter = 0;

        if (m_exposureAutoFinished)
        {
            m_exposureAutoMode = ExposureAutoMode::Off;
        }

        if (m_gainAutoFinished)
        {
            m_gainAutoMode = GainAutoMode::Off;
        }

        if (m_balanceWhiteAutoFinished)
        {
            m_balanceWhiteAutoMode = BalanceWhiteAutoMode::Off;
        }
    }

    return ProcessingResult::Success;
}


bool AutoFeaturesManager::IsCalculating() const
{
    std::lock_guard<std::recursive_mutex> lock(m_publicInterfaceMutex);

    return m_calculationTask.valid()
        && (m_calculationTask.wait_for(std::chrono::seconds{ 0 }) != std::future_status::ready);
}


/****************************************/
/* Functions for ExposureAuto/GainAuto  */
/****************************************/

void AutoFeaturesManager::SetExposureAutoMode(ExposureAutoMode exposureAutoMode)
{
    m_exposureAutoMode = exposureAutoMode;

    m_exposureAutoFinished = false;
}


AutoFeaturesManager::ExposureAutoMode AutoFeaturesManager::GetExposureAutoMode() const
{
    return m_exposureAutoMode;
}


bool AutoFeaturesManager::IsExposureAutoFinished() const
{
    return (m_exposureAutoFinished && (ExposureAutoMode::Off == m_exposureAutoMode));
}


bool AutoFeaturesManager::SetGainAutoMode(GainAutoMode gainAutoMode)
{
    if (GainType::None == m_gainTypeAll)
    {
        return false;
    }

    m_gainAutoMode = gainAutoMode;

    m_gainAutoFinished = false;

    return true;
}


AutoFeaturesManager::GainAutoMode AutoFeaturesManager::GetGainAutoMode() const
{
    return m_gainAutoMode;
}


bool AutoFeaturesManager::IsGainAutoFinished() const
{
    return (m_gainAutoFinished && (GainAutoMode::Off == m_gainAutoMode));
}


AutoFeaturesManager::GainType AutoFeaturesManager::GetGainTypeAll() const
{
    std::lock_guard<std::recursive_mutex> lock(m_publicInterfaceMutex);

    return m_gainTypeAll;
}


bool AutoFeaturesManager::SetBrightnessAutoTarget(uint8_t target)
{
    std::lock_guard<std::recursive_mutex> lock(m_publicInterfaceMutex);

    if ((target < GetBrightnessAutoTargetMin()) || (target > GetBrightnessAutoTargetMax()))
    {
        return false;
    }

    m_brightnessAutoTarget = target;

    return true;
}


uint8_t AutoFeaturesManager::GetBrightnessAutoTarget() const
{
    std::lock_guard<std::recursive_mutex> lock(m_publicInterfaceMutex);

    return m_brightnessAutoTarget;
}


uint8_t AutoFeaturesManager::GetBrightnessAutoTargetMin() const
{
    return 0;
}


uint8_t AutoFeaturesManager::GetBrightnessAutoTargetMax() const
{
    return 255;
}


uint8_t AutoFeaturesManager::GetBrightnessAutoTargetInc() const
{
    return 1;
}


bool AutoFeaturesManager::SetBrightnessAutoTargetTolerance(uint8_t targetTolerance)
{
    std::lock_guard<std::recursive_mutex> lock(m_publicInterfaceMutex);

    if ((targetTolerance < GetBrightnessAutoTargetToleranceMin())
        || (targetTolerance > GetBrightnessAutoTargetToleranceMax()))
    {
        return false;
    }

    m_brightnessAutoTargetTolerance = targetTolerance;

    return true;
}


uint8_t AutoFeaturesManager::GetBrightnessAutoTargetTolerance() const
{
    std::lock_guard<std::recursive_mutex> lock(m_publicInterfaceMutex);

    return m_brightnessAutoTargetTolerance;
}


uint8_t AutoFeaturesManager::GetBrightnessAutoTargetToleranceMin() const
{
    return 1;
}


uint8_t AutoFeaturesManager::GetBrightnessAutoTargetToleranceMax() const
{
    return 32;
}


uint8_t AutoFeaturesManager::GetBrightnessAutoTargetToleranceInc() const
{
    return 1;
}


bool AutoFeaturesManager::SetBrightnessAutoPercentile(double percentile)
{
    std::lock_guard<std::recursive_mutex> lock(m_publicInterfaceMutex);

    if ((percentile < GetBrightnessAutoPercentileMin()) || (percentile > GetBrightnessAutoPercentileMax()))
    {
        return false;
    }

    m_brightnessAutoPercentile = percentile;

    return true;
}


double AutoFeaturesManager::GetBrightnessAutoPercentile() const
{
    std::lock_guard<std::recursive_mutex> lock(m_publicInterfaceMutex);

    return m_brightnessAutoPercentile;
}


double AutoFeaturesManager::GetBrightnessAutoPercentileMin() const
{
    return 0;
}


double AutoFeaturesManager::GetBrightnessAutoPercentileMax() const
{
    return 100;
}


double AutoFeaturesManager::GetBrightnessAutoPercentileInc() const
{
    return 0.1;
}


bool AutoFeaturesManager::SetBrightnessAutoRoi(size_t x, size_t y, size_t width, size_t height)
{
    std::lock_guard<std::recursive_mutex> lock(m_publicInterfaceMutex);

    try
    {
        // Get current image pos and size
        size_t imageWidth = static_cast<size_t>(
            m_nodemapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Width")->Value());
        size_t imageHeight = static_cast<size_t>(
            m_nodemapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Height")->Value());

        if (x > imageWidth)
        {
            return false;
        }

        if ((x + width) > imageWidth)
        {
            return false;
        }

        if (y > imageHeight)
        {
            return false;
        }

        if ((y + height) > imageHeight)
        {
            return false;
        }

        m_brightnessAutoOffsetX = x;
        m_brightnessAutoOffsetY = y;
        m_brightnessAutoWidth = width;
        m_brightnessAutoHeight = height;

        return true;
    }
    catch (const std::exception&)
    {}

    return false;
}


void AutoFeaturesManager::GetBrightnessAutoRoi(size_t& x, size_t& y, size_t& width, size_t& height) const
{
    std::lock_guard<std::recursive_mutex> lock(m_publicInterfaceMutex);

    x = m_brightnessAutoOffsetX;
    y = m_brightnessAutoOffsetY;
    width = m_brightnessAutoWidth;
    height = m_brightnessAutoHeight;
}


uint8_t AutoFeaturesManager::GetLastBrightnessAutoMedian() const
{
    std::lock_guard<std::recursive_mutex> lock(m_publicInterfaceMutex);

    return m_brightnessAutoMedian;
}


/****************************************/
/* Functions for BalanceWhiteAuto       */
/****************************************/

bool AutoFeaturesManager::SetBalanceWhiteAutoMode(BalanceWhiteAutoMode balanceWhiteAutoMode)
{
    if (GainType::None == m_gainTypeColor)
    {
        return false;
    }

    m_balanceWhiteAutoMode = balanceWhiteAutoMode;

    m_balanceWhiteAutoFinished = false;

    return true;
}


AutoFeaturesManager::BalanceWhiteAutoMode AutoFeaturesManager::GetBalanceWhiteAutoMode() const
{
    return m_balanceWhiteAutoMode;
}


bool AutoFeaturesManager::IsBalanceWhiteAutoFinished() const
{
    return (m_balanceWhiteAutoFinished && (BalanceWhiteAutoMode::Off == m_balanceWhiteAutoMode));
}


AutoFeaturesManager::GainType AutoFeaturesManager::GetGainTypeColor() const
{
    std::lock_guard<std::recursive_mutex> lock(m_publicInterfaceMutex);

    return m_gainTypeColor;
}


bool AutoFeaturesManager::SetBalanceWhiteAutoRoi(size_t x, size_t y, size_t width, size_t height)
{
    if (GainType::None == m_gainTypeColor)
    {
        return false;
    }

    std::lock_guard<std::recursive_mutex> lock(m_publicInterfaceMutex);

    try
    {
        // Get current image size
        size_t imageWidth = static_cast<size_t>(
            m_nodemapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Width")->Value());
        size_t imageHeight = static_cast<size_t>(
            m_nodemapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Height")->Value());

        if (x > imageWidth)
        {
            return false;
        }

        if ((x + width) > imageWidth)
        {
            return false;
        }

        if (y > imageHeight)
        {
            return false;
        }

        if ((y + height) > imageHeight)
        {
            return false;
        }

        m_balanceWhiteAutoOffsetX = x;
        m_balanceWhiteAutoOffsetY = y;
        m_balanceWhiteAutoWidth = width;
        m_balanceWhiteAutoHeight = height;

        return true;
    }
    catch (const std::exception&)
    {}

    return false;
}


bool AutoFeaturesManager::GetBalanceWhiteAutoRoi(size_t& x, size_t& y, size_t& width, size_t& height) const
{
    if (GainType::None == m_gainTypeColor)
    {
        return false;
    }

    std::lock_guard<std::recursive_mutex> lock(m_publicInterfaceMutex);

    x = m_balanceWhiteAutoOffsetX;
    y = m_balanceWhiteAutoOffsetY;
    width = m_balanceWhiteAutoWidth;
    height = m_balanceWhiteAutoHeight;

    return true;
}


bool AutoFeaturesManager::GetLastBalanceWhiteAutoAverages(
    uint8_t& averageRed, uint8_t& averageGreen, uint8_t& averageBlue) const
{
    if (GainType::None == m_gainTypeColor)
    {
        return false;
    }

    std::lock_guard<std::recursive_mutex> lock(m_publicInterfaceMutex);

    averageRed = m_balanceWhiteAutoAverageRed;
    averageGreen = m_balanceWhiteAutoAverageGreen;
    averageBlue = m_balanceWhiteAutoAverageBlue;

    return true;
}


/****************************************/
/* Internal                             */
/****************************************/

void AutoFeaturesManager::CalculatorThread(AutoFeaturesManager* parent)
{
    if ((ExposureAutoMode::Off != parent->m_exposureAutoMode) && (GainAutoMode::Off != parent->m_gainAutoMode))
    {
        parent->CalculateExposureAutoAndGainAuto();
    }
    else if (ExposureAutoMode::Off != parent->m_exposureAutoMode)
    {
        parent->CalculateExposureAuto();
    }
    else if (GainAutoMode::Off != parent->m_gainAutoMode)
    {
        parent->CalculateGainAuto();
    }

    if (BalanceWhiteAutoMode::Off != parent->m_balanceWhiteAutoMode)
    {
        parent->CalculateBalanceWhiteAuto();
    }
}


bool AutoFeaturesManager::IsPixelFormatSupported(const peak::ipl::Image& image)
{
    if ("Bayer" == image.PixelFormat().Name().substr(0, 5))
    {
        return true;
    }
    else if (("Mono" == image.PixelFormat().Name().substr(0, 4))
        && (BalanceWhiteAutoMode::Off == m_balanceWhiteAutoMode))
    {
        return true;
    }
    else
    {
        return false;
    }
}


uint8_t AutoFeaturesManager::CalculateBrightnessAutoMedian(const peak::ipl::Image& image)
{
    uint16_t median = 0;

    uint8_t* source = image.Data();
    size_t pitch = image.Width();

    size_t roiWidth = std::min(m_brightnessAutoWidth, image.Width() - m_brightnessAutoOffsetX);
    size_t roiHeight = std::min(m_brightnessAutoHeight, image.Height() - m_brightnessAutoOffsetY);

    uint8_t* cur = source + (m_brightnessAutoOffsetY * pitch) + m_brightnessAutoOffsetX;
    uint8_t* end = source + ((m_brightnessAutoOffsetY + roiHeight - 1) * pitch) + (m_brightnessAutoOffsetX + roiWidth);

    uint64_t numberOfPixel = static_cast<uint64_t>(
        static_cast<double>(roiWidth * roiHeight) * ((100 - m_brightnessAutoPercentile) / 100.0));

    uint64_t histo[256];
    memset(histo, 0, sizeof(histo));

    while (cur < end)
    {
        uint8_t* endLine = cur + roiWidth;

        while (cur < endLine)
        {
            histo[*cur]++;
            cur++;
        }

        cur += (pitch - roiWidth);
    }

    uint64_t sum = 0;

    while ((median < 256) && (sum < numberOfPixel))
    {
        sum += histo[median];
        median++;
    }

    if ((median - 1) > 0)
    {
        m_brightnessAutoMedian = static_cast<uint8_t>(median - 1);
    }
    else
    {
        m_brightnessAutoMedian = 0;
    }

    return m_brightnessAutoMedian;
}


void AutoFeaturesManager::CalculateExposureAutoAndGainAuto()
{
    // Convert image to Mono8
    auto image = m_imageConverter->Convert(m_image, peak::ipl::PixelFormatName::Mono8);

    uint8_t median = CalculateBrightnessAutoMedian(image);

    try
    {
        double framerate_fps =
            m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")->Value();

        m_exposureTimeMax = m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("ExposureTime")
                                ->Maximum();

        // Limit the maximum epxosure to 95% of 1/FPS
        m_exposureTimeLimit = 0.95 * 1000000.0 / framerate_fps;

        if (m_exposureTimeLimit > m_exposureTimeMax)
        {
            m_exposureTimeLimit = m_exposureTimeMax;
        }

        double currentExposure = m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("ExposureTime")
                                     ->Value();

        const auto nodeMapLock = m_nodemapRemoteDevice->Lock();
        const auto gainSelectorValueGuard = [this] {
            // restore original gain selection when exiting scope
            const auto orginalGainSelection = m_nodemapRemoteDevice
                                                  ->FindNode<peak::core::nodes::EnumerationNode>(
                                                      "GainSelector")
                                                  ->CurrentEntry();

            return makeRaiiGuard([this, orginalGainSelection]() noexcept {
                try
                {
                    m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                        ->SetCurrentEntry(orginalGainSelection);
                }
                catch (const std::exception&)
                {}
            });
        }();

        if (GainType::Analog == m_gainTypeAll)
        {
            m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                ->SetCurrentEntry("AnalogAll");
        }
        else if (GainType::Digital == m_gainTypeAll)
        {
            m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                ->SetCurrentEntry("DigitalAll");
        }
        else if (GainType::Standard == m_gainTypeAll)
        {
            m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                ->SetCurrentEntry("All");
        }

        double currentGain = m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain")->Value();

        bool forceUpdate = false;

        // Exposure is not at the limit
        if (((currentExposure - m_exposureTimeLimit) > 100) || ((m_exposureTimeLimit - currentExposure) > 100))
        {
            // if the gain is not 1 ==> it is an inconsistent state -> we set gain to minimum
            if (!are_almost_equal(currentGain, m_gainMin, 4))
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain")->SetValue(m_gainMin);

                forceUpdate = true;
            }
        }

        if (((m_brightnessAutoTarget - median) > m_brightnessAutoTargetTolerance)
            || ((median - m_brightnessAutoTarget) > m_brightnessAutoTargetTolerance) || forceUpdate)
        {
            double factor = static_cast<double>(m_brightnessAutoTarget) / static_cast<double>(median);

            // If the median clips at maximum
            if (255 == median)
            {
                factor = 0.5;
            }

            // Exposure is at the limit
            if (((currentExposure - m_exposureTimeLimit) <= 100) && ((m_exposureTimeLimit - currentExposure) <= 100))
            {
                // We must go on with gain
                double gain = currentGain * factor;

                if (gain < m_gainMin)
                {
                    gain = m_gainMin;
                }
                else if (gain > m_gainLimit)
                {
                    gain = m_gainLimit;
                }

                if (!are_almost_equal(currentGain, gain, 4))
                {
                    m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain")->SetValue(gain);
                }
            }

            if (are_almost_equal(m_gainMin, currentGain, 4))
            {
                double exp = currentExposure * factor;

                if (exp > m_exposureTimeLimit)
                {
                    exp = m_exposureTimeLimit;
                }
                else if (exp < m_exposureTimeMin)
                {
                    exp = m_exposureTimeMin;
                }

                m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("ExposureTime")->SetValue(exp);
            }
        }
        else
        {
            if (ExposureAutoMode::Once == m_exposureAutoMode)
            {
                m_exposureAutoFinished = true;
            }

            if (GainAutoMode::Once == m_gainAutoMode)
            {
                m_gainAutoFinished = true;
            }
        }
    }
    catch (const std::exception&)
    {}
}


void AutoFeaturesManager::CalculateExposureAuto()
{
    // Convert image to Mono8
    auto image = m_imageConverter->Convert(m_image, peak::ipl::PixelFormatName::Mono8);

    uint8_t median = CalculateBrightnessAutoMedian(image);

    try
    {
        double framerate_fps =
            m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")->Value();

        m_exposureTimeMax = m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("ExposureTime")
                                ->Maximum();

        // Limit the maximum epxosure to 95% of 1/FPS
        m_exposureTimeLimit = 0.95 * 1000000.0 / framerate_fps;

        if (m_exposureTimeLimit > m_exposureTimeMax)
        {
            m_exposureTimeLimit = m_exposureTimeMax;
        }
    }
    catch (const std::exception&)
    {}

    if (((m_brightnessAutoTarget - median) > m_brightnessAutoTargetTolerance)
        || ((median - m_brightnessAutoTarget) > m_brightnessAutoTargetTolerance))
    {
        double factor = static_cast<double>(m_brightnessAutoTarget) / static_cast<double>(median);

        try
        {
            double currentExposure =
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("ExposureTime")->Value();

            double exp = currentExposure * factor;

            if (exp > m_exposureTimeLimit)
            {
                exp = m_exposureTimeLimit;
            }
            else if (exp < m_exposureTimeMin)
            {
                exp = m_exposureTimeMin;
            }

            m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("ExposureTime")->SetValue(exp);
        }
        catch (const std::exception&)
        {}
    }
    else
    {
        if (ExposureAutoMode::Once == m_exposureAutoMode)
        {
            m_exposureAutoFinished = true;
        }
    }
}


void AutoFeaturesManager::CalculateGainAuto()
{
    // Convert image to Mono8
    auto image = m_imageConverter->Convert(m_image, peak::ipl::PixelFormatName::Mono8);

    uint8_t median = CalculateBrightnessAutoMedian(image);

    if (((m_brightnessAutoTarget - median) > m_brightnessAutoTargetTolerance)
        || ((median - m_brightnessAutoTarget) > m_brightnessAutoTargetTolerance))
    {
        double factor = static_cast<double>(m_brightnessAutoTarget) / static_cast<double>(median);

        try
        {
            const auto nodeMapLock = m_nodemapRemoteDevice->Lock();
            const auto gainSelectorValueGuard = [this] {
                // restore original gain selection when exiting scope
                const auto orginalGainSelection = m_nodemapRemoteDevice
                                                      ->FindNode<peak::core::nodes::EnumerationNode>(
                                                          "GainSelector")
                                                      ->CurrentEntry();

                return makeRaiiGuard([this, orginalGainSelection]() noexcept {
                    try
                    {
                        m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                            ->SetCurrentEntry(orginalGainSelection);
                    }
                    catch (const std::exception&)
                    {}
                });
            }();

            if (GainType::Analog == m_gainTypeAll)
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                    ->SetCurrentEntry("AnalogAll");
            }
            else if (GainType::Digital == m_gainTypeAll)
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                    ->SetCurrentEntry("DigitalAll");
            }
            else if (GainType::Standard == m_gainTypeAll)
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                    ->SetCurrentEntry("All");
            }

            double currentGain = m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain")
                                     ->Value();

            double gain = currentGain * factor;

            if (gain < m_gainMin)
            {
                gain = m_gainMin;
            }
            else if (gain > m_gainLimit)
            {
                gain = m_gainLimit;
            }

            if (!are_almost_equal(currentGain, gain, 4))
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain")->SetValue(gain);
            }
        }
        catch (const std::exception&)
        {}
    }
    else
    {
        if (GainAutoMode::Once == m_gainAutoMode)
        {
            m_gainAutoFinished = true;
        }
    }
}


void AutoFeaturesManager::CalculateBalanceWhiteAuto()
{
    // Convert image to BGR8
    auto image = m_imageConverter->Convert(m_image, peak::ipl::PixelFormatName::BGR8);

    double avgRed = 1.0;
    double avgGreen = 1.0;
    double avgBlue = 1.0;

    uint8_t* source = static_cast<uint8_t*>(image.Data());
    size_t width = image.Width();
    size_t bpp = static_cast<size_t>(image.PixelFormat().CalculateStorageSizeOfPixels(1));

    size_t roiWidth = std::min(m_balanceWhiteAutoWidth, image.Width() - m_balanceWhiteAutoOffsetX);
    size_t roiHeight = std::min(m_balanceWhiteAutoHeight, image.Height() - m_balanceWhiteAutoOffsetY);

    uint8_t* cur = source + (m_balanceWhiteAutoOffsetY * (width * bpp)) + (m_balanceWhiteAutoOffsetX * bpp);
    uint8_t* end = source + ((m_balanceWhiteAutoOffsetY + roiHeight - 1) * (width * bpp))
        + ((m_balanceWhiteAutoOffsetX + roiWidth) * bpp);

    uint64_t red = 0;
    uint64_t green = 0;
    uint64_t blue = 0;

    // Calculate the sum of the red, green and blue pixels
    while (cur < end)
    {
        uint8_t* endLine = cur + (roiWidth * bpp);

        while (cur < endLine)
        {
            blue += cur[0];
            green += cur[1];
            red += cur[2];
            cur += bpp;
        }

        cur += ((width - roiWidth) * bpp);
    }

    uint64_t numberOfPixel = roiWidth * roiHeight;

    // Calculate average of each color channel
    avgRed = static_cast<double>(red) / static_cast<double>(numberOfPixel);
    avgGreen = static_cast<double>(green) / static_cast<double>(numberOfPixel);
    avgBlue = static_cast<double>(blue) / static_cast<double>(numberOfPixel);

    if (are_almost_equal(0.0, avgRed, 4))
    {
        avgRed = 1;
    }

    if (are_almost_equal(0.0, avgGreen, 4))
    {
        avgGreen = 1;
    }

    if (are_almost_equal(0.0, avgBlue, 4))
    {
        avgBlue = 1;
    }

    m_balanceWhiteAutoAverageRed = static_cast<uint8_t>(avgRed);
    m_balanceWhiteAutoAverageGreen = static_cast<uint8_t>(avgGreen);
    m_balanceWhiteAutoAverageBlue = static_cast<uint8_t>(avgBlue);

    // Calculate global average
    double gray = (avgRed + avgGreen + avgBlue) / 3;

    // Calculate scaling factor for each channel
    double scalingFactorRed = gray / avgRed;
    double scalingFactorGreen = gray / avgGreen;
    double scalingFactorBlue = gray / avgBlue;

    int maxDiff = 1;

    if ((abs(static_cast<int>(avgRed) - static_cast<int>(avgGreen)) <= maxDiff)
        && (abs(static_cast<int>(avgRed) - static_cast<int>(avgBlue)) <= maxDiff)
        && (abs(static_cast<int>(avgGreen) - static_cast<int>(avgBlue)) <= maxDiff))
    {
        if (BalanceWhiteAutoMode::Once == m_balanceWhiteAutoMode)
        {
            m_balanceWhiteAutoFinished = true;
        }

        return;
    }

    if (GainType::Ipl != m_gainTypeColor)
    {
        try
        {
            const auto nodeMapLock = m_nodemapRemoteDevice->Lock();
            const auto gainSelectorValueGuard = [this] {
                // restore original gain selection when exiting scope
                const auto orginalGainSelection = m_nodemapRemoteDevice
                                                      ->FindNode<peak::core::nodes::EnumerationNode>(
                                                          "GainSelector")
                                                      ->CurrentEntry();

                return makeRaiiGuard([this, orginalGainSelection]() noexcept {
                    try
                    {
                        m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                            ->SetCurrentEntry(orginalGainSelection);
                    }
                    catch (const std::exception&)
                    {}
                });
            }();

            if (GainType::Digital == m_gainTypeColor)
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                    ->SetCurrentEntry("DigitalRed");
            }
            else if (GainType::Standard == m_gainTypeColor)
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                    ->SetCurrentEntry("Red");
            }

            double currentGainRed = m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain")
                                        ->Value();
            double gainRed = currentGainRed * scalingFactorRed;

            if (GainType::Digital == m_gainTypeColor)
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                    ->SetCurrentEntry("DigitalGreen");
            }
            else if (GainType::Standard == m_gainTypeColor)
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                    ->SetCurrentEntry("Green");
            }

            double currentGainGreen = m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain")
                                          ->Value();
            double gainGreen = currentGainGreen * scalingFactorGreen;

            if (GainType::Digital == m_gainTypeColor)
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                    ->SetCurrentEntry("DigitalBlue");
            }
            else if (GainType::Standard == m_gainTypeColor)
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                    ->SetCurrentEntry("Blue");
            }

            double currentGainBlue = m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain")
                                         ->Value();
            double gainBlue = currentGainBlue * scalingFactorBlue;

            if ((gainGreen <= gainRed) && (gainGreen <= gainBlue))
            {
                gainRed /= gainGreen;
                gainBlue /= gainGreen;
                gainGreen = 1.0;
            }
            else if ((gainRed <= gainGreen) && (gainRed <= gainBlue))
            {
                gainGreen /= gainRed;
                gainBlue /= gainRed;
                gainRed = 1.0;
            }
            else if ((gainBlue <= gainGreen) && (gainBlue <= gainRed))
            {
                gainGreen /= gainBlue;
                gainRed /= gainBlue;
                gainBlue = 1.0;
            }

            if (gainRed < m_gainRedMin)
            {
                gainRed = m_gainRedMin;
            }
            else if (gainRed > m_gainRedMax)
            {
                gainRed = m_gainRedMax;
            }


            if (gainGreen < m_gainGreenMin)
            {
                gainGreen = m_gainGreenMin;
            }
            else if (gainGreen > m_gainGreenMax)
            {
                gainGreen = m_gainGreenMax;
            }


            if (gainBlue < m_gainBlueMin)
            {
                gainBlue = m_gainBlueMin;
            }
            else if (gainBlue > m_gainBlueMax)
            {
                gainBlue = m_gainBlueMax;
            }

            if (GainType::Digital == m_gainTypeColor)
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                    ->SetCurrentEntry("DigitalRed");
            }
            else if (GainType::Standard == m_gainTypeColor)
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                    ->SetCurrentEntry("Red");
            }

            m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain")->SetValue(gainRed);

            if (GainType::Digital == m_gainTypeColor)
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                    ->SetCurrentEntry("DigitalGreen");
            }
            else if (GainType::Standard == m_gainTypeColor)
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                    ->SetCurrentEntry("Green");
            }

            m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain")->SetValue(gainGreen);

            if (GainType::Digital == m_gainTypeColor)
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                    ->SetCurrentEntry("DigitalBlue");
            }
            else if (GainType::Standard == m_gainTypeColor)
            {
                m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
                    ->SetCurrentEntry("Blue");
            }

            m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain")->SetValue(gainBlue);
        }
        catch (const std::exception&)
        {}
    }
    else
    {
        if (m_gainIPL)
        {
            double currentGainRed = static_cast<double>(m_gainIPL->RedGainValue());
            double gainRed = currentGainRed * scalingFactorRed;

            double currentGainGreen = static_cast<double>(m_gainIPL->GreenGainValue());
            double gainGreen = currentGainGreen * scalingFactorGreen;

            double currentGainBlue = static_cast<double>(m_gainIPL->BlueGainValue());
            double gainBlue = currentGainBlue * scalingFactorBlue;

            if ((gainGreen <= gainRed) && (gainGreen <= gainBlue))
            {
                gainRed /= gainGreen;
                gainBlue /= gainGreen;
                gainGreen = 1.0;
            }
            else if ((gainRed <= gainGreen) && (gainRed <= gainBlue))
            {
                gainGreen /= gainRed;
                gainBlue /= gainRed;
                gainRed = 1.0;
            }
            else if ((gainBlue <= gainGreen) && (gainBlue <= gainRed))
            {
                gainGreen /= gainBlue;
                gainRed /= gainBlue;
                gainBlue = 1.0;
            }

            if (gainRed < m_gainRedMin)
            {
                gainRed = m_gainRedMin;
            }
            else if (gainRed > m_gainRedMax)
            {
                gainRed = m_gainRedMax;
            }


            if (gainGreen < m_gainGreenMin)
            {
                gainGreen = m_gainGreenMin;
            }
            else if (gainGreen > m_gainGreenMax)
            {
                gainGreen = m_gainGreenMax;
            }


            if (gainBlue < m_gainBlueMin)
            {
                gainBlue = m_gainBlueMin;
            }
            else if (gainBlue > m_gainBlueMax)
            {
                gainBlue = m_gainBlueMax;
            }

            m_gainIPL->SetRedGainValue(static_cast<float>(gainRed));
            m_gainIPL->SetGreenGainValue(static_cast<float>(gainGreen));
            m_gainIPL->SetBlueGainValue(static_cast<float>(gainBlue));
        }
    }
}


void AutoFeaturesManager::SetGainControllerIPL(peak::ipl::Gain* gainIPL)
{
    m_gainIPL = gainIPL;
}
