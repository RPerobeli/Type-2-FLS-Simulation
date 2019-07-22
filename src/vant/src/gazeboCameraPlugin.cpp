#include <ros/ros.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/distortion_models.h>

#include <iostream>

#undef PixelFormat

using std::cout;
using std::endl;

sensor_msgs::ImagePtr im(new sensor_msgs::Image);
sensor_msgs::CameraInfo info;

/////////////////////////////////////////////////
void cb(ConstImagesStampedPtr &_msg)
{
    unsigned char *rgbData = NULL;
    unsigned int rgbDataSize = 0;

    for (int i = 0; i < _msg->image_size(); ++i)
      {
        rgbData = NULL;
        rgbDataSize = 0;

        // Convert the image data to RGB
        gazebo::common::Image img;
        img.SetFromData(
            (unsigned char *)(_msg->image(i).data().c_str()),
            _msg->image(i).width(),
            _msg->image(i).height(),
            (gazebo::common::Image::PixelFormat)(_msg->image(i).pixel_format()));

        img.GetRGBData(&rgbData, rgbDataSize);

        // Store the image data
        sensor_msgs::fillImage(*im, sensor_msgs::image_encodings::RGB8, _msg->image(i).height(),
                               _msg->image(i).width(), 3*_msg->image(i).width(),
                               reinterpret_cast<const void*>(rgbData));
        delete [] rgbData;
      }
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "gazeboCameraPlugin");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Publisher pub = n.advertise<sensor_msgs::Image>("/gazebo/camera/image_raw",1);
    ros::Publisher pub_info = n.advertise<sensor_msgs::CameraInfo>("/gazebo/camera/camera_info",1);
    
    // Load gazebo
    gazebo::client::setup(_argc, _argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo camera topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/iris/camera/link/camera/image", cb);

    int seq = 0;
    // Busy wait loop...replace with your own code as needed.
    while (ros::ok())
    {
        im->header.stamp = ros::Time::now();
        im->header.seq = ++seq;
        im->header.frame_id = "camera";

        info.header.stamp = im->header.stamp;
        info.header.seq = im->header.seq;
        info.header.frame_id = im->header.frame_id;
        info.height = im->height;
        info.width = im->width;
        info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

        // Plumb bob model parameters
        info.D.resize(5);
        info.D[0] = 0.041594;
        info.D[1] = -0.121639;
        info.D[2] = -0.004713;
        info.D[3] = -0.004730;
        info.D[4] = 0.000000;

        // Camera matrix
        info.K[0] = (im->width/2)/tan(1.047/2); // focal_length = (width/2)/tan(horizontal_fov/2);
        info.K[1] = 0;
        info.K[2] = im->width/2; // principal point
        info.K[3] = 0;
        info.K[4] = info.K[0];
        info.K[5] = im->height/2; // principal point
        info.K[6] = 0;
        info.K[7] = 0;
        info.K[8] = 1;

        // Rectification matrix
        info.R[0] = 1;
        info.R[1] = 0;
        info.R[2] = 0;
        info.R[3] = 0;
        info.R[4] = 1;
        info.R[5] = 0;
        info.R[6] = 0;
        info.R[7] = 0;
        info.R[8] = 1;

        // Projection matrix
        info.P[0] = info.K[0];
        info.P[1] = 0;
        info.P[2] = info.K[2];
        info.P[3] = 0;
        info.P[4] = 0;
        info.P[5] = info.K[4];
        info.P[6] = info.K[5];
        info.P[7] = 0;
        info.P[8] = 0;
        info.P[9] = 0;
        info.P[10] = 1;
        info.P[11] = 0;

        // Fonte: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html

        pub_info.publish(info);
        pub.publish(im);
        loop_rate.sleep();
    }

    // Make sure to shut everything down.
    gazebo::client::shutdown();
}
