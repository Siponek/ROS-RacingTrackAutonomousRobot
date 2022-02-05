#include <unistd.h>
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "assigmentRT2/serviceForAssigment.h"
#include "stdlib.h"
#include <iostream>

int userInteraction();
void clear();
namespace Color
{
    enum Code
    {
        FG_RED = 31,
        FG_GREEN = 32,
        FG_BLUE = 34,
        FG_DEFAULT = 39,
        BG_RED = 41,
        BG_GREEN = 42,
        BG_BLUE = 44,
        BG_DEFAULT = 49
    };
    class Modifier
    {
        Code code;

    public:
        Modifier(Code pCode) : code(pCode) {}
        friend std::ostream &
        operator<<(std::ostream &os, const Modifier &mod)
        {
            return os << "\033[" << mod.code << "m";
        }
    };
}

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
    clear();
    std::cout << ("robotUI_> The record is 109 units per sec\n");
    std::cout << ("robotUI_> $$$ \n");
    while (ros::ok())
    {
        fflush(stdout);

        letter = userInteraction();
        if (letter == 'w')
        {
            velocityIncreaseClient.waitForExistence();
            if (velocityIncreaseClient.call(velocityServiceRequest))
            {
                std::cout << ("robotUI_> increase velocity\r") << std::endl;
                std::cout << ("\r_>%s\n", velocityServiceRequest.response.message.c_str());
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
                std::cout << ("robotUI_>  decrease velocity\r") << std::endl;
                std::cout << ("\r_>%s\n", velocityServiceRequest.response.message.c_str());
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
    Color::Modifier green(Color::FG_GREEN);
    Color::Modifier blue(Color::FG_BLUE);
    Color::Modifier normal(Color::FG_DEFAULT);
    Color::Modifier redBackground(Color::BG_RED);
    Color::Modifier defaultBackground(Color::BG_DEFAULT);

    std::cout << green << "\nPlease type in input and confirm with enter :\n";
    std::cout << blue << "w - increase velocity, s - decrease velocity\n";
    std::cout << "r - reset robot position\n";
    std::cout << normal << defaultBackground;

    fflush(stdin);
    fflush(stdout);
    std::cin >> input;
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
