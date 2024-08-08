#ifndef _IDS_ROS_DRIVER_CPP_
#define _IDS_ROS_DRIVER_CPP_

#include "IDS_Camera.cpp"

// ros main function
int main(int argc, char **argv)
{
    // ros init
    ros::init(argc, argv, "~");
    ros::NodeHandle nh;

    IDS_Camera cam(nh);

    // ros publisher init
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>(cam.cam_topic, 10);

    // open camera
    if (!cam.OpenCamera())
    {
        cam.~IDS_Camera();
        return -1;
    }
    
    // set some parameters value for camera
    if (!cam.SetParam())
    {
        cam.~IDS_Camera();
    return -1;
    }

    // set Roi
    if (!cam.SetRoi())
    {
        cam.~IDS_Camera();
        return -1;
    }

    //
    if (!cam.PrepareAcquisition())
    {
        cam.~IDS_Camera();
        return -1;
    }

    if (!cam.AllocAndAnnounceBuffers())
    {
        cam.~IDS_Camera();
        return -1;
    }

    if (!cam.StartAcquisition())
    {
        cam.~IDS_Camera();
        return -1;
    }

    // Start thread for catching image
    std::thread m_acquisitionThread(&IDS_Camera::ReceiveImage, &cam);
    m_acquisitionThread.detach();

    int frame_count = 0;
    int wrong_frame_count = 0;
    if(cam.use_CLAHE)
    {
        ROS_INFO("Use CLAHE");
    }
    ROS_INFO("Loop for catch image!");

    while (ros::ok())
    {
        try
        {
            // buffer is ready to convert
            if (cam.buffer_ready)
            {
                // ReceiveImage(m_dataStream_left);
                peak::ipl::Image image_left = cam.Return_Image();
                cv::Mat cvImage;

                cvImage = cv::Mat::zeros(image_left.Height(), image_left.Width(), CV_8UC1);
                int sizeBuffer = static_cast<int>(image_left.ByteCount());

                cam.buffer_ready = false;

                // Device buffer is being copied into cv_bridge format
                {
                    std::unique_lock<std::mutex> lock(cam.mtx_left);

                    std::memcpy(cvImage.data, image_left.Data(), sizeBuffer);
                }

                // cv_bridge Image is converted to sensor_msgs/Image to publish on ROS Topic
                cv_bridge::CvImage cvBridgeImage;
                cvBridgeImage.header.stamp = ros::Time::now();
                cvBridgeImage.header.frame_id = cam.cam_name;
                cvBridgeImage.encoding = sensor_msgs::image_encodings::MONO8;

                //CLAHE Process
                if(cam.use_CLAHE)
                {
                    cam.CLAHE_Process(cvImage);
                }
                cvBridgeImage.image = cvImage;
                sensor_msgs::ImagePtr img_msg_ptr(new sensor_msgs::Image());

                img_msg_ptr = cvBridgeImage.toImageMsg();
                img_msg_ptr->header.stamp = ros::Time::now();
                img_msg_ptr->header.seq = frame_count++;
                img_msg_ptr->header.frame_id = "left";
                img_msg_ptr->height = 1084;
                img_msg_ptr->width = 1448;

                //if ((frame_count % 10) == 0)
                image_pub.publish(*img_msg_ptr);
                ros::spinOnce();

                //ROS_INFO("Successfully Publishing data!");
            }
        }
        catch (const std::exception &e)
        {
            ROS_WARN("Warning: Image Process has a mistack!");
            wrong_frame_count++;
        }
    }

    if (!cam.StopAcquisition())
    {
        cam.~IDS_Camera();
        return -1;
    }

    peak::Library::Close();

    return 0;
}

#endif
