#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <iostream>
#include <algorithm>
#include <sstream>
using namespace std;

const float PI = 3.14159265;
float rate = 100;

geometry_msgs::Twist getMessage(double linear_x, double angular_z)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    return msg;
}

class PoseCallback {
public:
    int turtle_idx;
    ros::Subscriber sub;
    turtlesim::Pose pose;

    void callback(const turtlesim::Pose::ConstPtr& msg)
    {
        cout << "turtle " << turtle_idx+1 << " " << msg->x << " " << msg->y << endl;
        pose = *msg;
    }
};
 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "myturtle_control");
    ros::NodeHandle h;
    ros::Publisher pub[10];
    PoseCallback sub[10];

    int n_turtle = atoi(argv[1]);
    cout << "n_turtle = " << n_turtle << endl;
   //goi rua ngau nhien
    for(int i = 1; i < n_turtle; i++)
      {
        ros::service::waitForService("spawn");
        ros::ServiceClient spawner = h.serviceClient<turtlesim::Spawn>("spawn");
        turtlesim::Spawn turtle;
        turtle.request.x = atof(argv[2 * i]);
        turtle.request.y = atof(argv[2 * i + 1]);
        turtle.request.theta = 0;
        spawner.call(turtle);
      }
    for (int i = 0; i < n_turtle; i++) {
        stringstream s;
        s << "turtle" << i+1;
        string name = s.str();

        pub[i] = h.advertise<geometry_msgs::Twist>(name + "/cmd_vel", 1000);
        sub[i].turtle_idx = i;
        sub[i].sub = h.subscribe(name+"/pose", 1000, &PoseCallback::callback, &sub[i]);
        cout << "subcribe turtle " << i << " to " << name << "/pose" << endl;
    }
    ros::Rate loopRate(rate);
    double goal[10][2];
    int j=2*n_turtle; 
    for(int i=0; i<n_turtle; i++){
       goal[i][0] = sub[i].pose.x;
       goal[i][0] = sub[i].pose.y;
    }
    double distance;
            while (ros::ok()) {
                 
               for(int i=0; i<n_turtle; i++){
                 distance = sqrt(pow(goal[i][0] - sub[i].pose.x, 2) + pow(goal[i][1] - sub[i].pose.y, 2));
                 if(distance < 0.01){
                  if(j < argc-1){
                   goal[i][0]=atof(argv[j++]);
                   goal[i][1]=atof(argv[j++]);
                  }
                 }
                 distance = sqrt(pow(goal[i][0] - sub[i].pose.x, 2) + pow(goal[i][1] - sub[i].pose.y, 2));
                 if(distance < 0.01){
                    distance = 0;
                    pub[i].publish(getMessage(0,0));
                 }
                 else{
                    double dx = goal[i][0]- sub[i].pose.x, dy = goal[i][1] - sub[i].pose.y;
                    double angle = asin((cos(sub[i].pose.theta)*dy - sin(sub[i].pose.theta)*dx)/distance);
                    pub[i].publish(getMessage(1.5*distance,4.5*angle));
                 }
                 
               }
               if(distance < 0.01){
                 break;
               } 
                loopRate.sleep();
                ros::spinOnce();
            }
    
    return 0;
}
