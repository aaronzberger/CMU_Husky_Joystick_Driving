#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

//Joystick Mappings (Logitech F710)
const int kButtonX = 0;
const int kButtonA = 1;
const int kButtonB = 2;
const int kButtonY = 3;
const int kButtonLeftTrigger = 6;
const int kButtonRightTrigger = 7;

const int kAxisLeftHorizontal = 0;
const int kAxisLeftVertical = 1;
const int kAxisRightHorizontal = 2;
const int kAxisRightVertical = 3;

const double deadzone = 0.2;

const double boostThrottleScalar = 2.0;
const double boostSteerScalar = 1.5;
const double slowThrottleScalar = 0.5;
const double slowSteerScalar = 0.8;

geometry_msgs::Twist currentMessage;

/**
 * @brief Update the velocity message to publish based on joystick input
 * 
 * @param msg the joystick message received from the /joy ros topic
 */
void joyCallBack(const sensor_msgs::Joy::ConstPtr &msg) {

    //Determine linear velocity based on the left vertical axis
    double throttle {-msg->axes.at(kAxisLeftVertical) / 2};

    if(throttle < deadzone && throttle > -deadzone)
        throttle = 0.0;

    //Determine angular velocity based on the right horizontal axis
    double steer {msg->axes.at(kAxisRightHorizontal) / 2};

    if(steer < deadzone && steer > -deadzone)
        steer = 0.0;

    //Apply slow or boost mode based on trigger input
    if(msg->buttons.at(kButtonLeftTrigger)) {
        throttle *= slowThrottleScalar;
        steer *= slowSteerScalar;
    }
    //Use else if for safety: slow mode takes precedence
    else if(msg->buttons.at(kButtonRightTrigger)) {
        throttle *= boostThrottleScalar;
        steer *= boostSteerScalar;
    }

    //Create a velocity message to publish to the husky
    geometry_msgs::Twist twist;

    twist.linear.x = throttle;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = steer;

    currentMessage = twist;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "husky_driving");

    ros::NodeHandle n;

    ros::Subscriber subJoy = n.subscribe<sensor_msgs::Joy>("/joy", 100, joyCallBack);
    ros::Publisher pubVel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    while(ros::ok()) {
        pubVel.publish(currentMessage);
        ros::spinOnce();
    }

    ros::spin();

    return 0;
}