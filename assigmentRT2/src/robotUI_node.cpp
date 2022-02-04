#include <unistd.h>
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "assigmentRT2/serviceForAssigment.h"
#include "stdlib.h"
#include <iostream>

int userInteraction();
void clear();

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
        fflush(stdout);
        // clear();
        letter = userInteraction();
        if (letter == 'w')
        {
            velocityIncreaseClient.waitForExistence();
            if (velocityIncreaseClient.call(velocityServiceRequest))
            {
                clear();
                std::cout << ("robotUI_> increase velocity");
                std::cout << ("\r%s", velocityServiceRequest.response.message.c_str());
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
                // clear();
                std::cout << ("robotUI_>  decrease velocity\r") << std::endl;
                std::cout << ("\r%s", velocityServiceRequest.response.message.c_str());
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
                std::cout << ("\rrobotUI_>  RESET");
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
    // TODO Better UI/UX
    char input;
    input = getchar();
    return input;
}

void clear()
{
#if defined(__linux__) || defined(__unix__) || defined(__APPLE__)
    system("clear");
#endif

#if defined(_WIN32) || defined(_WIN64)
    system("cls");
#endif
}
