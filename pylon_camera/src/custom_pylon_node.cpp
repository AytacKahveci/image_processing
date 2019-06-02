/* @author Aytaç Kahveci */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <image_transport/image_transport.h>
#include <mutex>

#include <pylon/PylonIncludes.h>

#include <iostream>

using namespace Pylon;
using namespace GenApi;

int frameWidth = 1920;
int frameHeight = 1200;

CGrabResultPtr ptrGrabResult;

class videoObject
{

public:
    videoObject(ros::NodeHandle& nh):nh_(nh)
    {
        std::string ns_ = nh_.getNamespace();
        sub = nh_.subscribe(ns_ + "/image_raw_roi",1,&videoObject::roiCb, this);

        camera = new CInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());
        std::cout << "Using device " << camera->GetDeviceInfo().GetModelName() << std::endl;

        camera->RegisterConfiguration(new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);

        camera->GrabCameraEvents = false;

        INodeMap& nodemap = camera->GetNodeMap();
        imap = &nodemap;
        camera->Open();

        // Set the pixel format to RGB8 if available.
        CEnumerationPtr pixelFormat(nodemap.GetNode("PixelFormat"));
        width =  CIntegerPtr(camera->GetNodeMap().GetNode("Width"));
        height = CIntegerPtr(camera->GetNodeMap().GetNode("Height"));
        offsetX = CIntegerPtr(camera->GetNodeMap().GetNode("OffsetX"));
        offsetY = CIntegerPtr(camera->GetNodeMap().GetNode("OffsetY"));

        String_t oldPixelFormat = pixelFormat->ToString();
        if (IsAvailable(pixelFormat->GetEntryByName("mono8")))
        {
          pixelFormat->FromString("mono8");
          std::cout << "mono8" << std::endl;
        }
        pixelFormat->FromString("RGB8");

        int64_t offX = 10;
        int64_t offY = 0;
        if ( IsWritable( offsetX))
            {
                std::cout << "offsetX \n";
            }
            if ( IsWritable( offsetY))
            {
                std::cout << "offsetY \n";
            }

        std::cout << "offsetX" << offsetX->GetMax() << std::endl;
        std::cout << "offsetY" << offsetY->GetMin() << std::endl;
        std::cout << "inc" << offsetY->GetInc() << std::endl;
        std::cout << "inc" << offsetX->GetInc() << std::endl;

    }

    ~videoObject(){}

    void roiCb(const sensor_msgs::RegionOfInterest& roiMsg)
    {
        if(lock_.try_lock())
        {
        camera->StopGrabbing();
        ROS_INFO("*******roiCb********");

        width->SetValue(640);
        ROS_INFO("*******roiCb********");
        height->SetValue(480);
        ROS_INFO("*******roiCb********");
        offsetX->SetValue(static_cast<int64_t>(704));
        ROS_INFO("*******roiCb********");
        offsetY->SetValue(static_cast<int64_t>(540));
        ROS_INFO("*******roiCb********");
        frameWidth = 640;
        frameHeight = 480;
        ROS_INFO("*******roiCb********");
        camera->StartGrabbing();
        lock_.unlock();
        }
        else{
            ROS_INFO("lock error");
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub;
    INodeMap* imap;
    CIntegerPtr width;
    CIntegerPtr height;
    CIntegerPtr offsetX;
    CIntegerPtr offsetY;
    CInstantCamera* camera;
    std::mutex lock_;
};



int main(int argc, char *argv[])
{
    Pylon::PylonAutoInitTerm autoInitTerm;

    ros::init(argc, argv, "custom_pylon_node");
    ros::NodeHandle nh;
    std::string ns = nh.getNamespace();

    image_transport::ImageTransport it_(nh);
    image_transport::Publisher pub;
    pub = it_.advertise("/pylon_camera_node/image_raw",1);

    sensor_msgs::Image image;

    videoObject video(nh);

    video.camera->StartGrabbing();

    std::cout << "Using device " << video.camera->GetDeviceInfo().GetModelName() << std::endl;

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        video.camera->ExecuteSoftwareTrigger();

        // Retrieve grab results and notify the camera event and image event handlers.
        video.camera->RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);
        // Nothing to do here with the grab result, the grab results are handled by the registered event handler.
        if (ptrGrabResult->GrabSucceeded())
        {
            image.header.stamp = ros::Time::now();
            image.header.frame_id = "raw";
            uint8_t *pImageBuffer = reinterpret_cast<uint8_t*>(ptrGrabResult->GetBuffer());
            image.width = frameWidth;
            image.height = frameHeight;
            image.step = frameWidth*3;

            std::cout << "SizeX: " << ptrGrabResult->GetWidth() << std::endl;
            std::cout << "SizeY: " << ptrGrabResult->GetHeight() << std::endl;
            std::cout << "SizeIm: " << ptrGrabResult->GetImageSize() << std::endl;

            image.data.assign(reinterpret_cast<uint8_t*>(ptrGrabResult->GetBuffer()), reinterpret_cast<uint8_t*>(ptrGrabResult->GetBuffer()) + frameWidth*frameHeight*3);
            //image.data.assign(pImageBuffer, pImageBuffer + 6912000);
            image.encoding = "rgb8";
            ROS_INFO_STREAM("Here");
            pub.publish(image);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
