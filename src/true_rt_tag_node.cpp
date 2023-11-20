#include <true_rt_tag/true_rt_tag.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "true_rt_tag_node");

    ros::NodeHandle nh;

    ros::Rate rate(10);

    TRUE_RT_TAG rt_tag(&nh, rate);

    while(ros::ok())
    {
        ros::spinOnce();

        rt_tag.getTrueRT();

        rate.sleep();
    }
}