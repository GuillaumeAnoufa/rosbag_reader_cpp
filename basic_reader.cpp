#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char* argv[])
{
    rosbag::Bag bag;
    bag.open("/home/ganoufa/data/rosbags/camera_2021-12-10-16-46-42.bag", rosbag::bagmode::Read);

    cv_bridge::CvImagePtr cv_ptr;
    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        if (m.getTopic() == "/camera/image_raw/compressed") {
            sensor_msgs::CompressedImage::ConstPtr msg_img = m.instantiate<sensor_msgs::CompressedImage>();
            cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8);
            cv::imshow("test", cv_ptr->image);
            cv::waitKey(0);
        } else if (m.getTopic() == "/camera/image_raw") {
            sensor_msgs::Image::ConstPtr msg_img = m.instantiate<sensor_msgs::Image>();
            cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8);
            cv::imshow("test", cv_ptr->image);
            cv::waitKey(0);
        }
    }
    bag.close();
    return 0;
}