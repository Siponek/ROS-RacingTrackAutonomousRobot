#include "ros/ros.h"

// Use
//  Found service at rostopic list/ echo/ type type: std_srvs/LaserScan.h
//? Used for topic /base_scan - e.g. coordinates of arena
#include "sensor_msgs/LaserScan.h"
//? Used for cmd_vel - e.g. Moving robot
#include "geometry_msgs/Twist.h"
//? (TODO) Used for incrementing

// Found service at rosservice list / echo / type type : std_srvs/Empty.h
//? Used for /reset_positions
#include "std_srvs/Empty.h"
// The required header for custom service
#include "assigmentRT2/serviceForAssigment.h"
#include <algorithm>
#include <vector>
#include <unistd.h>

// #define LEFTFAR_START 0 * 4
// #define LEFT_START 31 * 4
// #define FRONT_START 86 * 4
// #define RIGHT_START 96 * 4
// #define RIGHTFAR_START 131 * 4

// #define LEFTFAR_END 120  // 30 * 4 = 120
// #define LEFT_END 340     // 85 * 4 = 340
// #define FRONT_END 381    // 95 * 4 = 380
// #define RIGHT_END 601    // 151 * 4 = 604
// #define RIGHTFAR_END 721 // 180 * 4 = 720

#define LEFTFAR_END 30 * 4
#define LEFT_END 85 * 4
#define FRONT_END 95 * 4
#define RIGHT_END 151 * 4
#define RIGHTFAR_END 180 * 4
#define LEFT 1
#define RIGHT 0

/*
What to do:
- The robot moves forward DONE
- Robot turns DONE
- Robot Reset DONE
    Found service at rosservice list/ echo/ type type: std_srvs/Empty.h
- ROS_INFO on status meh
- Reaction to any button DONE
- Route algorithm
    rosmsg show sensormsgs/LaserScan for getting structure of a message
    float32 ranges[] for range to an obstacle
  !HOW TO:
  nHandle.advertise - cannot publish anything on a topic without this
  nHandle.subscribe - listening on topic for invoking a callback function

*/

//? class for containing the distances of scanner readings
//? 180 degrees are divided into 5 sections
//? (front is for trying to eliminate "drunk mover effect"):
//?  frontObst    - 85-95
//?  leftFarObst - 150-180
//?  leftObs    - 95-150
//?  rightFarObst  - 0-30
//?  rightObs     - 30-85
// TODO maxObstacleDistanceAvg not working bcs it does not aquire new values, need to try to make it a method and push back the values?
// TODO Make methods that aquire values of a class!

class ScannerDataClass
{                        // The class
public:                  // Access specifier
  _Float32 frontObst;    // Attribute
  _Float32 leftFarObst;  // Attribute
  _Float32 leftObs;      // Attribute
  _Float32 rightFarObst; // Attribute
  _Float32 rightObs;     // Attribute

  float maxObstacleDistanceAvg()
  {
    std::vector<float> arrayOfAvg{leftFarObst, leftObs, frontObst, rightObs, rightFarObst};
    return *std::max_element(std::begin(arrayOfAvg), std::end(arrayOfAvg));
  }
  float minObstacleDistanceAvg()
  {
    std::vector<float> arrayOfAvg{leftFarObst, leftObs, frontObst, rightObs, rightFarObst};

    return *std::min_element(std::begin(arrayOfAvg), std::end(arrayOfAvg));
  }
  float meanOfLeft()
  {
    std::vector<float> arrayOfAvg{leftFarObst, leftObs};

    return (arrayOfAvg.at(0) + arrayOfAvg.at(1)) / arrayOfAvg.size();
  }
  float meanOfRight()
  {
    std::vector<float> arrayOfAvg{rightFarObst, rightObs};

    return (arrayOfAvg.at(0) + arrayOfAvg.at(1)) / arrayOfAvg.size();
  }
};

ScannerDataClass robotScanner;
ros::Publisher pubVel;
ros::ServiceClient sClientForReset;
ros::ServiceServer pubService;
ros::Subscriber sub;
ros::ServiceServer serverReset;       // UI SERVICES
ros::ServiceServer serverVelIncrease; // UI SERVICES
ros::ServiceServer serverVelDecrease; // UI SERVICES
geometry_msgs::Twist vel;

float primaryVelocity = 15;

bool resetRobotPosition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
void robotSensorFunc(const sensor_msgs::LaserScan::ConstPtr &spaghetti);
bool velocity_increaseCallback(assigmentRT2::serviceForAssigment::Request &req, assigmentRT2::serviceForAssigment::Response &res);
bool velocity_decreaseCallback(assigmentRT2::serviceForAssigment::Request &req, assigmentRT2::serviceForAssigment::Response &res);
void move(float velocity);
void moveTurn(float turnVelocity, float moveVelocity, int leftOrRight);

int main(int argc, char **argv)
{
  // Initialize the node, setup the NodeHandle for handling the communication with the ROS //system
  ros::init(argc, argv, "robotLogic");
  ros::NodeHandle nHandler;
  ros::Rate loopRate(50);
  float maxObstacleDistanceAvg = 0;
  float minObstacleDistanceAvg = 0;
  float diffJump = 0;

  // call the service for reseting the robot's position || need to make a advertiser for publishing
  sClientForReset = nHandler.serviceClient<std_srvs::Empty>("/reset_positions");
  // Define the subscriber to robot's position
  pubVel = nHandler.advertise<geometry_msgs::Twist>("/cmd_vel", 1); // custom message with random name for the topic
  sub = nHandler.subscribe("/base_scan", 1, robotSensorFunc);       // check the topic name
  serverReset = nHandler.advertiseService("/robotReset", resetRobotPosition);
  serverVelIncrease = nHandler.advertiseService("/velocity_increase", velocity_increaseCallback);
  serverVelDecrease = nHandler.advertiseService("/velocity_decrease", velocity_decreaseCallback);
  float difference;

  //! autonomous move algorythm
  while (ros::ok())
  {
    // TODO swap sides since side -> right now
    // system("clear");

    maxObstacleDistanceAvg = robotScanner.maxObstacleDistanceAvg();
    minObstacleDistanceAvg = robotScanner.minObstacleDistanceAvg();

    // ROS_INFO("This is the leftObs avg       %f", robotScanner.leftObs);
    // ROS_INFO("This is the rightObs avg      %f", robotScanner.rightObs);
    // ROS_INFO("This is the leftFarObst avg   %f", robotScanner.leftFarObst);
    // ROS_INFO("This is the rightFarObst avg  %f", robotScanner.rightFarObst);
    // ROS_INFO("This is the front avg         %f", robotScanner.frontObst);
    // ROS_INFO("This is the mean left         %f", robotScanner.meanOfLeft());
    // ROS_INFO("This is the mean right        %f", robotScanner.meanOfRight());
    difference = robotScanner.meanOfLeft() - robotScanner.meanOfRight();
    ROS_INFO("difference        %f", difference);
    if (difference > 0)
    {
      ROS_INFO("TURNING LEFT");
      // 30 is the max distance of a scanner
      // [moveVelocity] needs to rise if [difference] is low
      moveTurn(primaryVelocity * abs(difference), (robotScanner.frontObst / 30) * (primaryVelocity * 1 / (abs(difference) + 1)), LEFT);
    }
    else if (difference <= 0)
    {
      ROS_INFO("TURNING RIGHT");
      moveTurn(primaryVelocity * abs(difference), (robotScanner.frontObst / 30) * (primaryVelocity * 1 / (abs(difference) + 1)), RIGHT);
    }
    ros::spinOnce();
  }
  return 0;
}

bool velocity_increaseCallback(assigmentRT2::serviceForAssigment::Request &req, assigmentRT2::serviceForAssigment::Response &res)
{

  primaryVelocity += 1;
  char *messageToSend = "Current user velocity _>";
  asprintf(&messageToSend, "%f", primaryVelocity);
  res.message = messageToSend;

  ROS_INFO("W KEY DETECTED!");
  ROS_INFO("W KEY DETECTED!");
  ROS_INFO("W KEY DETECTED!");

  return true;
}

bool velocity_decreaseCallback(assigmentRT2::serviceForAssigment::Request &req, assigmentRT2::serviceForAssigment::Response &res)
{
  primaryVelocity -= 1;
  char *messageToSend = "Current user velocity _>";
  asprintf(&messageToSend, "%f", primaryVelocity);
  res.message = messageToSend;

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
    exit(-111);
  }
  return true;
}

void robotSensorFunc(const sensor_msgs::LaserScan::ConstPtr &spaghetti)
{
  //?  frontObst    - 85-95
  //?  leftFarObs - 150-180
  //?  leftObs    - 95-150
  //?  rightFarObs  - 0-30
  //?  rightObs     - 30-85
  // To access members of a class through a pointer,
  // use the arrow operator.
  // ranges is a <array> or an vector data type and thus supports .size()
  // int rangesLen = spaghetti->ranges.size();
  // ROS_INFO("rangesLen is equal to _> %d", rangesLen); //Since .size() give out 721 this means that every degree is divided by 4
  // 360 is 90 degrees
  _Float32 averageFront = 0;
  _Float32 averageRightFar = 0;
  _Float32 averageRight = 0;
  _Float32 averageLeftFar = 0;
  _Float32 averageLeft = 0;
  size_t i = 0;
  int countTemp = 0;

  // #define LEFTFAR_END 170  // 30 * 4 = 120
  // #define LEFT_END 340     // 85 * 4 = 340
  // #define FRONT_END 381    // 95 * 4 = 380
  // #define RIGHT_END 551    // 151 * 4 = 604
  // #define RIGHTFAR_END 721 // 180 * 4 = 720

  for (; i <= LEFTFAR_END; i++)
  {
    averageLeftFar += spaghetti->ranges[i];
    countTemp++;
  }
  averageLeftFar /= countTemp;

  countTemp = 0;
  for (; i <= LEFT_END; i++)
  {
    averageLeft += spaghetti->ranges[i];
    countTemp++;
  }
  averageLeft /= countTemp;

  countTemp = 0;
  for (; i <= FRONT_END; i++)
  {
    averageFront += spaghetti->ranges[i];
    countTemp++;
  }
  averageFront /= countTemp;

  countTemp = 0;
  for (; i <= RIGHT_END; i++)
  {
    averageRight += spaghetti->ranges[i];
    countTemp++;
  }
  averageRight /= countTemp;

  countTemp = 0;
  for (; i <= RIGHTFAR_END; i++)
  {
    averageRightFar += spaghetti->ranges[i];
    countTemp++;
  }
  averageRightFar /= countTemp;

  printf("averageFront %f", averageFront);
  ROS_INFO("averageFront %f", averageFront);
  ROS_INFO("averageRightFar %f", averageRightFar);
  ROS_INFO("averageRight %f", averageRight);
  ROS_INFO("averageLeftFar %f", averageLeftFar);
  ROS_INFO("averageLeft %f", averageLeft);

  robotScanner.frontObst = averageFront;
  robotScanner.leftFarObst = averageRightFar;
  robotScanner.leftObs = averageRight;
  robotScanner.rightFarObst = averageLeftFar;
  robotScanner.rightObs = averageLeft;
}

void move(float velocity)
{
  vel.linear.x = velocity;
  pubVel.publish(vel);
}
void moveTurn(float turnVelocity, float moveVelocity, int leftOrRight)
{
  vel.linear.x = moveVelocity;
  if (leftOrRight)
  {
    // Left turn
    vel.angular.z = turnVelocity;
  }
  else
  {
    // Right turn
    vel.angular.z = -turnVelocity;
  }
  pubVel.publish(vel);
}