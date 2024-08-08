#ifndef __IDS_CAMERA_H__
#define __IDS_CAMERA_H__

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <thread>
#include <mutex>

// Add IDS peak library
#include <peak/peak.hpp>
#include <peak_ipl/peak_ipl.hpp>
#include <peak/peak_buffer_converter.hpp>
#include <peak/converters/peak_buffer_converter_ipl.hpp>

#include <peak/node_map/peak_node_map.hpp>
#include <autofeaturesmanager.h>
#include <autofeaturesmanager.cpp>

// OpenCV Headers
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>


class IDS_Camera
{
public:
    IDS_Camera(ros::NodeHandle nh);
    ~IDS_Camera();

    // open camera
    bool OpenCamera();

    // approximately set camera frame Rate to 200Hz
    bool setFrameRate();

    // set some parameters for cameras
    bool SetParam();

    // PrepareAcquisition
    bool PrepareAcquisition();

    // setROI
    bool SetRoi();

    // Alloc and Announce Buffers
    bool AllocAndAnnounceBuffers();

    // Start to Acquisite
    bool StartAcquisition();

    // Stop to Acquisite
    bool StopAcquisition();

    // Image Strength using CLAHE
    void CLAHE_Process(cv::Mat &img);

    // Receive image
    void ReceiveImage();

    // load parameter from launch 
    void read_param();

    // Reture Image
    peak::ipl::Image Return_Image();

    // Reture DataStream
    std::shared_ptr<peak::core::DataStream> Return_DataStream();

    // name a camera
    void setUsername(std::string Usrname);

public:
    // camera name
    std::string cam_name;

    //  camera topic 
    std::string cam_topic;

    // ROS NodeHandle
    std::shared_ptr<ros::NodeHandle> nh_;
    std::string node_name;

    // Buffer completion flag bit
    bool buffer_ready = false;

    // buffer mutex
    std::mutex mtx_left;

    // whether to use CLAHE
    bool use_CLAHE;


private:
    // camera frame rate
    // |During the imaging process, there may be frame reduction and frame loss, |
    // |so the frame rate is slightly higher than 200Hz                          |
    double camera_frame_rate;//200.0;

    // camera exposure time (us)
    double camera_exposure_time;


    // camera brightness gain (main parameter to effect brightness of Image)
    double camera_brightness_gain;

    // camera brightness auto target
    double camera_brightness_target;

    // camera brightness ROI
    double camera_brightness_ROI_start_X;
    double camera_brightness_ROI_start_Y;
    double camera_brightness_ROI_end_X;
    double camera_brightness_ROI_end_Y;

    // camera brightness percentile
    double camera_brightness_percentile;

    // camera brightness target tolerance
    double camera_brightness_target_tolerance;

    // camera write balance ROI
    double camera_write_balance_ROS_start_X;
    double camera_write_balance_ROS_start_Y;
    double camera_write_balance_ROS_end_X;
    double camera_write_balance_ROS_end_Y;

    // wait for buffer time (ms)
    double wait_for_buffer_time;

    // Image DataStructure
    sensor_msgs::Image left_image;

    // devices for left camera
    std::shared_ptr<peak::core::Device> m_device_left = nullptr;

    // datastream for left camera
    std::shared_ptr<peak::core::DataStream> m_dataStream_left = nullptr;

    // nodemap for left camera
    std::shared_ptr<peak::core::NodeMap> m_nodemapRemoteDevice_left = nullptr;

    // peak image data
    peak::ipl::Image image_left;

    // feature manager
    AutoFeaturesManager *m_autoFeaturesManager; 

    // // Start thread for catching image
    // std::shared_ptr<std::thread> m_acquisitionThread;

};

#endif