#include <unistd.h>
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "assigmentRT2/serviceForAssigment.h"

int userInteraction();

int main(int argc, char **argv)
{
    int letter;
    ros::init(argc, argv, "robotUI_node");

    ros::ServiceClient robotReset;
    ros::ServiceClient velocityIncreaseClient;
    ros::ServiceClient velocityDecreaseClient;

    ros::NodeHandle nHandler;
    std_srvs::Empty resetServiceRequest;
    assigmentRT2::serviceForAssigment velocityServiceRequest;

    robotReset = nHandler.serviceClient<std_srvs::Empty>("/robotReset");
    velocityIncreaseClient = nHandler.serviceClient<assigmentRT2::serviceForAssigment>("/velocity_increase");
    velocityDecreaseClient = nHandler.serviceClient<assigmentRT2::serviceForAssigment>("/velocity_decrease");

    while (ros::ok())
    {
        letter = userInteraction();
        if (letter == 'w')
        {
            velocityIncreaseClient.waitForExistence();
            if (velocityIncreaseClient.call(velocityServiceRequest))
            {
                ROS_INFO("robotUI_> increase velocity");
                ROS_INFO("%s", velocityServiceRequest.response.message.c_str());
            }
            else
            {
                ROS_ERROR("robotUI_> Velocity increase message failed to send");
            }
        }
        else if (letter == 's')
        {
            velocityDecreaseClient.waitForExistence();
            if (velocityDecreaseClient.call(velocityServiceRequest))
            {
                ROS_INFO("robotUI_>  decrease velocity");
                ROS_INFO("%s", velocityServiceRequest.response.message.c_str());
            }
            else
            {
                ROS_ERROR("robotUI_> Velocity decrease message failed to send");
            }
        }
        else if (letter == 'r')
        {
            robotReset.waitForExistence();
            if (robotReset.call(resetServiceRequest))
            {
                ROS_INFO("robotUI_>  RESET");
            }
            else
            {
                ROS_ERROR("robotUI_> RESET message failed to send");
            }
        }
        ros::spinOnce();
    }
    return 0;
}

int userInteraction()
{
    char input;
    input = getchar();
    return input;
}
