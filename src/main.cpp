#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pubVel;

//Joystick Mappings (Logitech F710)
const int kButtonA = 1;
const int kButtonB = 2;
const int kButtonY = 3;
const int kButtonX = 4;

/**
 * @brief Callback function for joystick input
 * 
 * @param msg the joystick message received from the /joy ros topic
 */
void joyCallBack(const sensor_msgs::Joy::ConstPtr &msg) {

    //Determine linear velocity based on forward(Y) and backwards(A) buttons
    double linVel {0.0};

    if(msg->buttons.at(kButtonY) == 1) {
        linVel = 1.0;
    } else if(msg->buttons.at(kButtonA) == 1) {
        linVel = -1.0;
    }

    //Determine linear velocity based on right(B) and left(X) buttons
    double angVel {0.0};

    if(msg->buttons.at(kButtonB) == 1) {
        angVel = 0.5;
    } else if(msg->buttons.at(kButtonX) == 1) {
        angVel = -0.5;
    }

    //Create a velocity message to publish to the husky
    geometry_msgs::Twist twist;

    twist.linear.x = linVel;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = angVel;

    pubVel.publish(twist);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "husky_driving");

    ros::NodeHandle n;

    ros::Subscriber subJoy = n.subscribe<sensor_msgs::Joy>("/joy", 100, joyCallBack);
    pubVel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    return 0;
}