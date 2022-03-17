#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>

int main(int argc, char* argv[])
{
    rosbag::Bag bag;
    bag.open("/home/ganoufa/data/rosbags/camera_2021-12-10-16-46-42.bag");

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
        if (i != nullptr)
            std::cout << i->data << std::endl;
    }

    bag.close();
    return 0;
}