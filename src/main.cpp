#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

//Joystick Mappings (Logitech F710)
const int kButtonX = 0;
const int kButtonA = 1;
const int kButtonB = 2;
const int kButtonY = 3;

const int kAxisLeftHorizontal = 0;
const int kAxisLeftVertical = 1;
const int kAxisRightHorizontal = 2;
const int kAxisRightVertical = 3;

geometry_msgs::Twist currentMessage;

/**
 * @brief Update the velocity message to publish based on joystick input
 * 
 * @param msg the joystick message received from the /joy ros topic
 */
void joyCallBack(const sensor_msgs::Joy::ConstPtr &msg) {

    //Determine linear velocity based on forward(Y) and backwards(A) buttons
    double linVel {0.0};

    if(msg->buttons.at(kButtonA) == 1) {
        linVel = 1.0;
    } else if(msg->buttons.at(kButtonY) == 1) {
        linVel = -1.0;
    }

    //Determine linear velocity based on right(B) and left(X) buttons
    double angVel {0.0};

    if(msg->buttons.at(kButtonX) == 1) {
        angVel = 0.75;
    } else if(msg->buttons.at(kButtonB) == 1) {
        angVel = -0.75;
    }

    //Create a velocity message to publish to the husky
    geometry_msgs::Twist twist;

    twist.linear.x = linVel;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = angVel;

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