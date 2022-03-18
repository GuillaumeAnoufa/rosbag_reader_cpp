#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "tools/cropTools.hpp"
#include "ellipsesDetection.hpp"
#include "algoTypes.h"
#include "cudaMemory/CudaMemoryAllocator.h"
#include <signal.h>

static void sig_handler(int signo)
{
    switch (signo)
    {
    case SIGINT: // Windows & Linux well defined (ctrl + V signal)
        LOG(WARNING) << "caught SIGINT";
        exit(0);
        break;
    default:
        LOG(WARNING) << "unexpected signal caught: " << signo;
    }
}

int main(int argc, char* argv[])
{
    google::InstallFailureSignalHandler();
    google::InitGoogleLogging(argv[0]);

    struct sigaction sa_new;

    sa_new.sa_handler = sig_handler;
    sigemptyset(&sa_new.sa_mask);
    sa_new.sa_flags = SA_RESTART;
    sigaction(SIGINT, &sa_new, nullptr);

    // INIT
    algo::CudaMemoryAllocator *cudaMemory = algo::CudaMemoryAllocator::Instance();
    std::vector<algo::StdEllipse> outputEllipses;

    // CONFIG
    int colorMode = 1; // Yellow
    int brushMode = 1;
    int nbThumbnailsPerFrame = 3;
    algo::AlgoParam algoConfig(colorMode, brushMode, nbThumbnailsPerFrame);

    ellipsesDetection *ellipseDetector = new ellipsesDetection(cudaMemory->getMaxPoints(), cudaMemory->getMaxTriplets(), algoConfig);

    // BAG READ
    rosbag::Bag bag;
    bag.open("/media/ganoufa/GAnoufaSSD/vols_24_02/camera_lidar_vol_0.bag", rosbag::bagmode::Read);

    cv_bridge::CvImagePtr cv_ptr;
    bool first = true;
    cv::Mat view_img;
    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        if (m.getTopic() == "/camera/image_raw/compressed") {
            sensor_msgs::CompressedImage::ConstPtr msg_img = m.instantiate<sensor_msgs::CompressedImage>();
            cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8);
            if(first) {
                ellipseDetector->baseThumbnailsInit(cv_ptr->image.cols, cv_ptr->image.rows);
                first = false;
            }
            ellipseDetector->computeDetection(cv_ptr->image, cudaMemory->d_dataPoints,
                                                cudaMemory->d_tripletPoints,
                                                cudaMemory->d_nbTriplets, cudaMemory->d_houghCenter,
                                                cudaMemory->d_totalVotes, outputEllipses);
            cv_ptr->image.copyTo(view_img);
            for (size_t i = 0; i < outputEllipses.size(); i++)
            {
                if (outputEllipses[i].x > 0 && outputEllipses[i].y > 0 &&
                    outputEllipses[i].x < cv_ptr->image.cols && outputEllipses[i].y < cv_ptr->image.rows)
                {
                    std::stringstream ellipse_string;
                    ellipse_string << "Detected ellipses[" << i << "] : " << outputEllipses[i];
                    LOG(INFO) << ellipse_string.str();

                    cv::RotatedRect ellipseToDraw(cv::Point2f(outputEllipses[i].x, outputEllipses[i].y),
                                                    cv::Size2f(outputEllipses[i].width, outputEllipses[i].height),
                                                    outputEllipses[i].angle);
                    cv::ellipse(view_img, ellipseToDraw, ColorTools::colorVector[0], 5);
                }
            }
            cv::imshow("test", view_img);
            cv::waitKey(1);
        }
    }
    bag.close();
    return 0;
}