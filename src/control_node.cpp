#include <kinova_movo_teleop/BIO_IK_Planner.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bio_ik_planner");
    ros::NodeHandle ph("~");
    double hz;
    ph.param("frequency", hz, 20.0);

    ros::Rate rate(hz);
    ros::Rate cpu_limitter(1000);
    
    BIO_IK_Planner planner(ph, hz);

    ros::AsyncSpinner spinner(8);
    spinner.start();

    while (ros::ok())
    {
        if (planner.ready() && planner.plan())
        {
            rate.sleep();
            planner.publish();
        }
        else
        {
            cpu_limitter.sleep();
        }
    }
    return 0;
}