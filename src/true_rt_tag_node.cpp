#include <true_rt_tag/true_rt_tag.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "true_rt_tag_node");

    ros::NodeHandle nh;

    ros::Rate rate(30);

    TRUE_RT_TAG rt_tag(&nh, rate);

    int counter = 0;
    while(ros::ok())
    {
        ros::spinOnce();

        rt_tag.getTrueRT();

        if(counter++ >= 30)
        {
            if(!rt_tag.broadcastTagTF())
                ROS_INFO("Not initialized...!");
            counter = 0;
        }


        rate.sleep();
    }
}