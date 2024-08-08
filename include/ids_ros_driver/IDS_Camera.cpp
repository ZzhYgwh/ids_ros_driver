#ifndef __IDS_CAMERA_CPP__
#define __IDS_CAMERA_CPP__
#include "IDS_Camera.h"

// construct function
IDS_Camera::IDS_Camera(ros::NodeHandle nh)
{
    this->nh_ = std::make_shared<ros::NodeHandle>(nh);
    this->node_name = ros::this_node::getName();
    this->m_autoFeaturesManager = new AutoFeaturesManager();

    // Init IDS peak Libraries
    peak::Library::Initialize();

    read_param();


    ROS_INFO("Successfully load parameters, try to catch image!");
}

IDS_Camera::~IDS_Camera()
{
    delete m_autoFeaturesManager;
    peak::Library::Close();
}

// open camera
bool IDS_Camera::OpenCamera()
{
    ROS_INFO("Start to Open Camera!");
    try
    {
        // Create instance of the device manager
        auto &deviceManager = peak::DeviceManager::Instance();

        // Update the device manager
        deviceManager.Update();

        // open the first openable device in the device manager's device list
        size_t deviceCount = deviceManager.Devices().size();
        ROS_INFO("number of camera is %d", deviceCount);

        // Return if no device was found
        if (deviceCount == 0)
        {
            ROS_WARN("Have not a camera!");
            return false;
        }

        for (const auto &descriptor : deviceManager.Devices())
        {
            if (descriptor->UserDefinedName() == cam_name)
            {
                m_device_left = descriptor->OpenDevice(peak::core::DeviceAccessType::Control);
                ROS_INFO("successful to open camera: %s", m_device_left->UserDefinedName().c_str());
                m_nodemapRemoteDevice_left = m_device_left->RemoteDevice()->NodeMaps().at(0);
                // break;
                return true;
            }
        }
        ROS_ERROR("Fail to open camera! Please Check camera name!");
        return false;
    }
    catch (std::exception &e)
    {
        // ...
        ROS_ERROR("Can't open camera! Missing camera: %s", cam_name.c_str());
    }

    return false;
}

// approximately set camera frame Rate to 200Hz
bool IDS_Camera::setFrameRate()
{
    try
    {
        double minFrameRate = 0;
        double maxFrameRate = 0;
        double incFrameRate = 0;

        // set Exposure time to the least,in order to unlimit frame rate
        double exposureTime_dev = m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::FloatNode>("ExposureTime")->Minimum();
        exposureTime_dev = std::max(camera_exposure_time, exposureTime_dev);
        m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::FloatNode>("ExposureTime")->SetValue(exposureTime_dev);

        // Get frame rate range. All values in fps.
        minFrameRate = m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")->Minimum();
        maxFrameRate = m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")->Maximum();

        if (m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")->HasConstantIncrement())
        {
            incFrameRate = m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")->Increment();
        }
        else
        {
            // If there is no increment, it might be useful to choose a suitable increment for a GUI control element (e.g. a slider)
            incFrameRate = 0.1;
        }

        ROS_INFO("Frame Rate ");
        ROS_INFO("\t |-min: %f", minFrameRate);
        ROS_INFO("\t |-max: %f", maxFrameRate);
        ROS_INFO("\t |-inc: %f", incFrameRate);

        // Get the current frame rate
        double frameRate = m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")->Value();
        ROS_INFO("\t |-cur: %f", frameRate);

        // Set frame rate to maximum or 200 Hz (Try to reach 200Hz)
        frameRate = std::min(maxFrameRate, camera_frame_rate);
        m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")->SetValue(frameRate);
        ROS_INFO("\t |-set: %f", frameRate);
        ROS_INFO("Successful Set Frame Rate!");
        return true;
    }
    catch (const std::exception &e)
    {
        std::string strError = e.what();
        ROS_ERROR("Failed to set Frame Rate to this camera!");
    }
    return false;
}

// set some parameters for cameras
bool IDS_Camera::SetParam()
{
    try
    {
        // approximately set camera frame Rate to 200Hz
        if (!setFrameRate())
        {
            return false;
        }

        m_autoFeaturesManager->SetNodemapRemoteDevice(m_nodemapRemoteDevice_left);

        // the rest parameters set to auto adjustment mode, in order to unlimit parameters' value
        m_autoFeaturesManager->SetExposureAutoMode(AutoFeaturesManager::ExposureAutoMode::Continuous);
        ROS_INFO("Set Exposure Auto!");
        m_autoFeaturesManager->SetGainAutoMode(AutoFeaturesManager::GainAutoMode::Continuous);
        ROS_INFO("Set Gain Auto!");

        // m_autoFeaturesManager.SetGainControllerIPL()
        m_autoFeaturesManager->SetBalanceWhiteAutoMode(AutoFeaturesManager::BalanceWhiteAutoMode::Continuous);

        // Set current value of "ExposureTime" value to 9.5 (Exposure Time)
        m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::FloatNode>("ExposureTime")->SetValue(camera_exposure_time);

        // Set current value of "Gain" value to 9.5     (Brightness Gain)
        //m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::FloatNode>("Gain")->SetValue(camera_brightness_gain);

        // Set Brightness & WhiteBalance parameters
        /*
        m_autoFeaturesManager->SetBrightnessAutoTarget(camera_brightness_target);
        m_autoFeaturesManager->SetBrightnessAutoRoi(camera_brightness_ROI_start_X, camera_brightness_ROI_start_Y, camera_brightness_ROI_end_X, camera_brightness_ROI_end_Y);
        m_autoFeaturesManager->SetBrightnessAutoPercentile(camera_brightness_percentile);
        m_autoFeaturesManager->SetBrightnessAutoTargetTolerance(camera_brightness_target_tolerance);
        m_autoFeaturesManager->SetBalanceWhiteAutoRoi(camera_write_balance_ROS_start_X, camera_write_balance_ROS_start_Y, camera_write_balance_ROS_end_X, camera_write_balance_ROS_end_Y);
        */

        // Print information about camera parameters
        ROS_INFO("Successful Set Camera Params!");
        ROS_INFO("Camera Params:");
        ROS_INFO("Camera fps: %f", m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")->Value());
        ROS_INFO("ExposureTime: %f", m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::FloatNode>("ExposureTime")->Value());
        ROS_INFO("Gain: %f", m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::FloatNode>("Gain")->Value());
        return true;
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Failed to Set Camera Params!");
        return false;
    }
}

// PrepareAcquisition
bool IDS_Camera::PrepareAcquisition()
{
    try
    {
        auto dataStreams = m_device_left->DataStreams();
        if (dataStreams.empty())
        {
            // no data streams available
            ROS_ERROR("no data streams available");
            return false;
        }
        // otherwize open data streams
        m_dataStream_left = m_device_left->DataStreams().at(0)->OpenDataStream();

        return true;
    }
    catch (std::exception &e)
    {
        ROS_ERROR("Prepare Image Acquisition Wrong!");
    }

    return false;
}

// setROI
bool IDS_Camera::SetRoi()
{
    try
    {
        int64_t x = camera_brightness_ROI_start_X;
        int64_t y = camera_brightness_ROI_start_Y;
        int64_t width = camera_brightness_ROI_end_X - camera_brightness_ROI_start_X;
        int64_t height = camera_brightness_ROI_end_Y - camera_brightness_ROI_start_Y;

        // Get the minimum ROI and set it. After that there are no size restrictions anymore
        int64_t x_min = m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::IntegerNode>("OffsetX")->Minimum();
        int64_t y_min = m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::IntegerNode>("OffsetY")->Minimum();
        int64_t w_min = m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::IntegerNode>("Width")->Minimum();
        int64_t h_min = m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::IntegerNode>("Height")->Minimum();
        /*
             m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::IntegerNode>("OffsetX")->SetValue(x_min);
             m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::IntegerNode>("OffsetY")->SetValue(y_min);
             m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::IntegerNode>("Width")->SetValue(w_min);
             m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::IntegerNode>("Height")->SetValue(h_min);
       */
        // Get the maximum ROI values
        int64_t x_max = m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::IntegerNode>("OffsetX")->Maximum();
        int64_t y_max = m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::IntegerNode>("OffsetY")->Maximum();
        int64_t w_max = m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::IntegerNode>("Width")->Maximum();
        int64_t h_max = m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::IntegerNode>("Height")->Maximum();

        ROS_INFO("MIN = [%d,%d,%d,%d]", x_min, y_min, w_min, h_min);
        ROS_INFO("MAX = [%d,%d,%d,%d]", x_max, y_max, w_max, h_max);
        ROS_INFO("SET = [%d,%d,%d,%d]", x, y, width, height);

        if ((x < x_min) || (y < y_min) || (x > x_max) || (y > y_max))
        {

            ROS_ERROR("x & y outside of the range!");
            return false;
        }
        else if ((width < w_min) || (height < h_min) || ((x + width) > w_max) || ((y + height) > h_max))
        {
            ROS_ERROR("width or height outside of the range!");
            return false;
        }
        else
        {
            // Now, set final AOI
            m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::IntegerNode>("OffsetX")->SetValue(x);
            m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::IntegerNode>("OffsetY")->SetValue(y);
            m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::IntegerNode>("Width")->SetValue(width);
            m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::IntegerNode>("Height")->SetValue(height);
            ROS_INFO("Successful Set Roi!");
            return true;
        }
    }
    catch (std::exception &e)
    {
        ROS_ERROR("Set Roi Wrong!");
    }

    return false;
}

// Alloc and Announce Buffers
bool IDS_Camera::AllocAndAnnounceBuffers()
{
    try
    {
        if (m_dataStream_left)
        {
            // Flush queue and prepare all buffers for revoking
            m_dataStream_left->Flush(peak::core::DataStreamFlushMode::DiscardAll);

            // Clear all old buffers
            for (const auto &buffer : m_dataStream_left->AnnouncedBuffers())
            {
                m_dataStream_left->RevokeBuffer(buffer);
            }

            int64_t payloadSize = m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::IntegerNode>("PayloadSize")->Value();

            // Get number of minimum required buffers
            int numBuffersMinRequired = m_dataStream_left->NumBuffersAnnouncedMinRequired();

            // Alloc buffers
            for (size_t count = 0; count < numBuffersMinRequired; count++)
            {
                auto buffer = m_dataStream_left->AllocAndAnnounceBuffer(static_cast<size_t>(payloadSize), nullptr);
                m_dataStream_left->QueueBuffer(buffer);
            }

            ROS_INFO("Successful Alloc and Announce Buffers!");
            return true;
        }
    }
    catch (std::exception &e)
    {
        ROS_ERROR("alloc and announce buffers wrong!");
    }

    return false;
}

//   Start to Acquisite
bool IDS_Camera::StartAcquisition()
{
    try
    {
        m_dataStream_left->StartAcquisition(peak::core::AcquisitionStartMode::Default, peak::core::DataStream::INFINITE_NUMBER);
        m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked")->SetValue(1);
        m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::CommandNode>("AcquisitionStart")->Execute();
        ROS_INFO("Start Acquisition Image!");
        return true;
    }
    catch (std::exception &e)
    {
        ROS_ERROR("Start Acquisition Image Wrong!");
    }

    return false;
}

//   Stop to Acquisite
bool IDS_Camera::StopAcquisition()
{
    try
    {
        m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::CommandNode>("AcquisitionStop")->Execute();
        m_nodemapRemoteDevice_left->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked")->SetValue(0);
        m_dataStream_left->StopAcquisition(peak::core::AcquisitionStopMode::Default);
        ROS_INFO("Stop Acquisition Image!");
        return true;
    }
    catch (std::exception &e)
    {
        ROS_ERROR("Stop Acquisition Image Wrong!");
    }

    return false;
}

// Image Strength using CLAHE
void IDS_Camera::CLAHE_Process(cv::Mat &img)
{
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(4);
    clahe->setTilesGridSize(cv::Size(8, 8));
    clahe->apply(img, img);
}

// Receive Image
void IDS_Camera::ReceiveImage()
{
    while (ros::ok())
    {
        try
        {
            // Get buffer from device's DataStream. Wait 5000 ms. The buffer is automatically locked until it is queued again.
            const auto buffer = m_dataStream_left->WaitForFinishedBuffer(wait_for_buffer_time); // const auto

            // locked buffer image data
            //*buffer_image = *buffer;

            // Create IDS peak IPL image from buffer ...
            // peak::ipl::
            // image = peak::ipl::Image(peak::BufferTo<peak::ipl::Image>(buffer));
            {
                std::unique_lock<std::mutex> lock(mtx_left);
                image_left = peak::BufferTo<peak::ipl::Image>(buffer).ConvertTo(peak::ipl::PixelFormatName::Mono8, peak::ipl::ConversionMode::Fast);
            }
            // image = image_buffer.Clone();

            // const auto imageProcessed = image.ConvertTo(peak::ipl::PixelFormatName::Mono10, peak::ipl::ConversionMode::HighQuality);

            // Process buffer ...
            buffer_ready = true;

            // Queue buffer so that it can be used again
            m_dataStream_left->QueueBuffer(buffer);

            // ROS_INFO("Process has catch an image!");
        }
        catch (const std::exception &e)
        {
            // ROS_INFO("Process catch image data had a mistack!");
        }
    }
}

// load parameter from launch 
void IDS_Camera::read_param()
{
    nh_->param<std::string>(node_name+"/camera_name",cam_name,"cam0");
    ROS_INFO("camera_name = ",cam_name);
    nh_->param<std::string>(node_name+"/camera_topic",cam_topic,"/"+cam_name+"/image_raw");
    // get frame rate of this camera
    nh_->param<double>(node_name+"/frame_rate",camera_frame_rate,30.0);
    // get exposure time 
    nh_->param<double>(node_name+"/exposure_time",camera_exposure_time,4739.66);

     // use CLAHE 
    nh_->param<bool>(node_name+"/use_CLAHE",use_CLAHE,true);

    // get camera brightness gai
    nh_->param<double>(node_name+"/bright_gain",camera_brightness_gain,5);
    // get camera brightness auto target
    nh_->param<double>(node_name+"/bright_target",camera_brightness_target,150);    
    // get camera brightness ROI
    nh_->param<double>(node_name+"/bright_ROI_start_X",camera_brightness_ROI_start_X,5);
    nh_->param<double>(node_name+"/bright_ROI_start_Y",camera_brightness_ROI_start_Y,5);
    nh_->param<double>(node_name+"/bright_ROI_end_X",camera_brightness_ROI_end_X,1448);
    nh_->param<double>(node_name+"/bright_ROI_end_Y",camera_brightness_ROI_end_Y,1084);
    // get camera brightness percentile
    nh_->param<double>(node_name+"/bright_percentile",camera_brightness_target,150); 
    // get camera brightness target tolerance
    nh_->param<double>(node_name+"/bright_target_toler",camera_brightness_target_tolerance,3.0); 

    // get camera write balance ROI
    nh_->param<double>(node_name+"/write_balance_ROI_start_X",camera_brightness_ROI_start_X,0);
    nh_->param<double>(node_name+"/write_balance_ROI_start_Y",camera_brightness_ROI_start_Y,0);
    nh_->param<double>(node_name+"/write_balance_ROI_end_X",camera_brightness_ROI_end_X,1448);
    nh_->param<double>(node_name+"/write_balance_ROI_end_Y",camera_brightness_ROI_end_Y,1084);

    // wait for buffer time (ms)
    nh_->param<double>(node_name+"/wait_for_buffer_time",wait_for_buffer_time,10);
}

// name a camera
void IDS_Camera::setUsername(std::string Usrname)
{
    //Set a usename for camera and check   
    cam_name = Usrname;
    // //open camera
    // if(!OpenCamera(cam_name))
    // {
    //     peak::Library::Close();
    //     return -1;
    // }

    auto& deviceManager_user = peak::DeviceManager::Instance();
    deviceManager_user.Update();

    std::shared_ptr<peak::core::Device> m_device_user = deviceManager_user.Devices().at(0)->OpenDevice(peak::core::DeviceAccessType::Control);

    // Get NodeMap of the RemoteDevice for all accesses to the GenICam NodeMap tree
    std::shared_ptr<peak::core::NodeMap> m_nodemapRemoteDevice_User = m_device_user->RemoteDevice()->NodeMaps().at(0);
    m_nodemapRemoteDevice_User->FindNode<peak::core::nodes::StringNode>("DeviceUserID")->SetValue(cam_name);
}

// Reture Image
peak::ipl::Image IDS_Camera::Return_Image()
{
    return this->image_left;
}

// Reture DataStream
std::shared_ptr<peak::core::DataStream> IDS_Camera::Return_DataStream()
{
    return this->m_dataStream_left;
}
#endif