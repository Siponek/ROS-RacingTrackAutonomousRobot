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

#define FRONT_START 86 * 4
#define LEFTFAR_START 0 * 4
#define LEFT_START 31 * 4
#define RIGHT_START 96 * 4
#define RIGHTFAR_START 131 * 4

#define FRONT_END 95 * 4
#define LEFTFAR_END 30 * 4
#define LEFT_END 85 * 4
#define RIGHT_END 151 * 4
#define RIGHTFAR_END 180 * 4

/*
What to do:
- The robot drives forward DONE
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
//? (front is for trying to eliminate "drunk driver effect"):
//?  frontObstacle    - 85-95
//?  rightFarObstacle - 150-180
//?  rightObstacle    - 95-150
//?  leftFarObstacle  - 0-30
//?  leftObstacle     - 30-85
class scannerDataClass
{                            // The class
public:                      // Access specifier
  _Float32 frontObstacle;    // Attribute
  _Float32 rightFarObstacle; // Attribute
  _Float32 rightObstacle;    // Attribute
  _Float32 leftFarObstacle;  // Attribute
  _Float32 leftObstacle;     // Attribute
  std::vector<float> arrayOfAvg{rightFarObstacle, rightObstacle, frontObstacle, leftObstacle, leftFarObstacle};

  float maxAvg = *std::max_element(std::begin(arrayOfAvg), std::end(arrayOfAvg));
  float minAvg = *std::min_element(std::begin(arrayOfAvg), std::end(arrayOfAvg));
};

scannerDataClass robotScanner;
ros::Publisher pubVel;
ros::ServiceClient sClientForReset;
ros::ServiceServer pubService;
ros::Subscriber sub;
ros::ServiceServer serverReset;       // UI SERVICES
ros::ServiceServer serverVelIncrease; // UI SERVICES
ros::ServiceServer serverVelDecrease; // UI SERVICES
geometry_msgs::Twist vel;

int userMultiplier = 0;
float maxAvg = robotScanner.maxAvg;

bool resetRobotPosition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
void robotSensorFunc(const sensor_msgs::LaserScan::ConstPtr &spaghetti);
bool velocity_increaseCallback(assigmentRT2::serviceForAssigment::Request &req, assigmentRT2::serviceForAssigment::Response &res);
bool velocity_decreaseCallback(assigmentRT2::serviceForAssigment::Request &req, assigmentRT2::serviceForAssigment::Response &res);
void drive(float velocity);
void driveTurn(float turnVelocity, float driveVelocity, bool side);

int main(int argc, char **argv)
{
  // Initialize the node, setup the NodeHandle for handling the communication with the ROS //system
  ros::init(argc, argv, "robotLogic");
  ros::NodeHandle nHandler;

  // call the service for reseting the robot's position || need to make a advertiser for publishing
  sClientForReset = nHandler.serviceClient<std_srvs::Empty>("/reset_positions");
  // Define the subscriber to robot's position
  pubVel = nHandler.advertise<geometry_msgs::Twist>("/cmd_vel", 1); // custom message with random name for the topic
  sub = nHandler.subscribe("/base_scan", 1, robotSensorFunc);       // check the topic name
  serverReset = nHandler.advertiseService("/robotReset", resetRobotPosition);
  serverVelIncrease = nHandler.advertiseService("/velocity_increase", velocity_increaseCallback);
  serverVelDecrease = nHandler.advertiseService("/velocity_decrease", velocity_decreaseCallback);

  //! autonomous drive algorythm
  while (ros::ok())
  {

    // TODO swap sides since side -> right now
    // system("clear");
    // ROS_INFO("This is the RIGHTFAR AVG: %f", robotScanner.rightFarObstacle);
    // ROS_INFO("This is the RIGHT AVG: %f", robotScanner.rightObstacle);
    // ROS_INFO("This is the FRONT AVG: %f", robotScanner.frontObstacle);
    // ROS_INFO("This is the LEFT AVG: %f", robotScanner.leftObstacle);
    // ROS_INFO("This is the LEFTFAR AVG: %f", robotScanner.leftFarObstacle);
    // ROS_INFO("This is the max avg %f", robotScanner.maxAvg);

    if (maxAvg * 0.9 < robotScanner.rightFarObstacle)
    {
      ROS_INFO("TURNING FAR LEFT");

      driveTurn(5 * robotScanner.minAvg, 5, true);
    }
    else if (maxAvg * 0.9 < robotScanner.rightObstacle)
    {
      ROS_INFO("TURNING LEFT");

      driveTurn(5 * robotScanner.minAvg, 5, true);
    }
    else if (maxAvg * 0.9 < robotScanner.leftObstacle)
    {
      ROS_INFO("TURNING RIGHT");

      driveTurn(5 * robotScanner.minAvg, 5, false);
    }
    else if (maxAvg * 0.9 < robotScanner.leftFarObstacle)
    {
      ROS_INFO("TURNING FAR RIGHT");

      driveTurn(10 * robotScanner.minAvg, 5, false);
    }
    else
    {
      ROS_INFO("FORWARD");

      drive(50 * maxAvg);
    }

    usleep(9000);
    ros::spinOnce();
    // ros::spin();
  }
  return 0;
}

bool velocity_increaseCallback(assigmentRT2::serviceForAssigment::Request &req, assigmentRT2::serviceForAssigment::Response &res)
{
  res.message = "+++123";
  userMultiplier -= 0.5;

  ROS_INFO("W KEY DETECTED!");
  ROS_INFO("W KEY DETECTED!");
  ROS_INFO("W KEY DETECTED!");

  return true;
}

bool velocity_decreaseCallback(assigmentRT2::serviceForAssigment::Request &req, assigmentRT2::serviceForAssigment::Response &res)
{
  res.message = "+++123";
  userMultiplier += 0.5;

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
  //?  frontObstacle    - 85-95
  //?  rightFarObstacle - 150-180
  //?  rightObstacle    - 95-150
  //?  leftFarObstacle  - 0-30
  //?  leftObstacle     - 30-85
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

  for (; i <= LEFTFAR_END; i++)
  {
    averageLeftFar += spaghetti->ranges[i];
  }
  averageLeftFar /= i;
  for (; i <= LEFT_END; i++)
  {
    averageLeft += spaghetti->ranges[i];
  }
  averageLeft /= i;
  for (; i <= FRONT_END; i++)
  {
    averageFront += spaghetti->ranges[i];
  }
  averageFront /= i;
  for (; i <= RIGHT_END; i++)
  {
    averageRight += spaghetti->ranges[i];
  }
  averageRight /= i;
  for (; i <= RIGHTFAR_END; i++)
  {
    averageRightFar += spaghetti->ranges[i];
  }
  averageRightFar /= i;

  robotScanner.frontObstacle = averageFront;
  robotScanner.rightFarObstacle = averageRightFar;
  robotScanner.rightObstacle = averageRight;
  robotScanner.leftFarObstacle = averageLeftFar;
  robotScanner.leftObstacle = averageLeft;
  maxAvg = robotScanner.maxAvg;
  ROS_INFO("This is the RIGHTFAR AVG: %f", robotScanner.rightFarObstacle);
  ROS_INFO("This is the RIGHT AVG: %f", robotScanner.rightObstacle);
  ROS_INFO("This is the FRONT AVG: %f", robotScanner.frontObstacle);
  ROS_INFO("This is the LEFT AVG: %f", robotScanner.leftObstacle);
  ROS_INFO("This is the LEFTFAR AVG: %f", robotScanner.leftFarObstacle);
  ROS_INFO("This is the max avg %f", robotScanner.maxAvg);
}

void drive(float velocity)
{
  vel.linear.x = velocity;
  pubVel.publish(vel);
}
void driveTurn(float turnVelocity, float driveVelocity, bool side)
{
  vel.linear.x = driveVelocity;
  if (side)
  {
    // Turn LEFT
    vel.angular.z = turnVelocity;
  }
  else
  {
    // Turn RIGHT
    vel.angular.z = -turnVelocity;
  }
  pubVel.publish(vel);
}