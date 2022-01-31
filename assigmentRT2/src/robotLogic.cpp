#include "ros/ros.h"

// Found service at rostopic list/ echo/ type type: std_srvs/LaserScan.h
//? Used for topic /base_scan - e.g. coordinates of arena
#include "sensor_msgs/LaserScan.h"
//? Used for cmd_vel - e.g. Moving robot
#include "geometry_msgs/Twist.h"
//? (TODO) Used for incrementing

// Found service at rosservice list / echo / type type : std_srvs/Empty.h
//? Used for /reset_positions
#include "std_srvs/Empty.h"
// Add the required header
#include "assigmentRT2/serviceForAssigment.h"

/*
What to do:
- The robot drives forward DONE
- Robot turns DONE
- Robot Reset DONE
    Found service at rosservice list/ echo/ type type: std_srvs/Empty.h
- ROS_INFO on status
- Reaction to any button
- Route algorithm
  !HOW TO:
  nHandle.advertise - cannot publish anything on a topic without this
  nHandle.subscribe - listening on topic for invoking a callback function

*/

// main stuff
ros::Publisher pubVel;
ros::ServiceClient sClientForReset;
// for calling/advertising(?) the service
ros::ServiceServer pubService;
ros::Subscriber sub;

ros::ServiceServer serverReset;       // UI SERVICES
ros::ServiceServer serverVelIncrease; // UI SERVICES
ros::ServiceServer serverVelDecrease; // UI SERVICES

// stuff for nodes and clients

int count = 0;
int userMultiplier = 0;

bool resetRobotPosition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
void robotCallback(const sensor_msgs::LaserScan::ConstPtr &whateverComesOutOfthisStupidThing);
bool velocity_increaseCallback(assigmentRT2::serviceForAssigment::Request &req, assigmentRT2::serviceForAssigment::Response &res);
bool velocity_decreaseCallback(assigmentRT2::serviceForAssigment::Request &req, assigmentRT2::serviceForAssigment::Response &res);

int main(int argc, char **argv)
{
  // Initialize the node, setup the NodeHandle for handling the communication with the ROS //system
  ros::init(argc, argv, "robotLogic");
  ros::NodeHandle nHandler;

  // call the service for reseting the robot's position || need to make a advertiser for publishing
  sClientForReset = nHandler.serviceClient<std_srvs::Empty>("/reset_positions");

  // Define the subscriber to turtle's position
  pubVel = nHandler.advertise<geometry_msgs::Twist>("/cmd_vel", 1); // custom message with random name for the topic
  sub = nHandler.subscribe("/base_scan", 1, robotCallback);         // check the topic name

  serverReset = nHandler.advertiseService("/robotReset", resetRobotPosition);
  serverVelIncrease = nHandler.advertiseService("/velocity_increase", velocity_increaseCallback);
  serverVelDecrease = nHandler.advertiseService("/velocity_decrease", velocity_decreaseCallback);

  ros::spin();
  return 0;
}

bool velocity_increaseCallback(assigmentRT2::serviceForAssigment::Request &req, assigmentRT2::serviceForAssigment::Response &res)
{
  res.message = "123";

  ROS_INFO("W KEY DETECTED!");
  ROS_INFO("W KEY DETECTED!");
  ROS_INFO("W KEY DETECTED!");

  return true;
}

bool velocity_decreaseCallback(assigmentRT2::serviceForAssigment::Request &req, assigmentRT2::serviceForAssigment::Response &res)
{
  res.message = "---123";

  ROS_INFO("S KEY DETECTED!");
  ROS_INFO("S KEY DETECTED!");
  ROS_INFO("S KEY DETECTED!");

  return true;
}

bool resetRobotPosition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std_srvs::Empty request;
  ROS_INFO("R KEY DETECTED!");
  ROS_INFO("R KEY DETECTED!");
  ROS_INFO("R KEY DETECTED!");

  sClientForReset.waitForExistence();
  if (sClientForReset.call(request))
  {
    ROS_INFO("SUCCESS on calling serviceReset!");
  }
  else
  {
    ROS_ERROR("FAILED to calling serviceReset!");
    exit(-1);
  }
  return true;
}

void robotCallback(const sensor_msgs::LaserScan::ConstPtr &whateverComesOutOfthisStupidThing)
{
  // I hope that inventor of pointers stays in hell

  geometry_msgs::Twist customVel;
  if (count == 100)
  {
    ROS_INFO("robotCallback _> RESET");
    // resetRobotPosition();
    count = 0;
  }
  else if (count >= 20)
  {
    ROS_INFO("robotCallback _> moving at 25");
    customVel.linear.x = 50.0;
  }
  else
  {
    ROS_INFO("robotCallback _> moving at -5");
    customVel.linear.x = -5.0;
  }
  ROS_INFO("robotCallback _> publishing customVel");

  pubVel.publish(customVel);
  count++;
}