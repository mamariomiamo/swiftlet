#include "px4ctrl/px4fsm.h"
#include "px4_tf2/px4_tf2.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_ctrl");
    ros::NodeHandle nh("~");
    PX4FSM::PX4FSM px4fsm(nh);
    px4_tf2::px4_tf2 px4_tf2(nh);
    ros::spin();
    return 0;
}