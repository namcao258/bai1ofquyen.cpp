#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include<cmath>
#include <iostream>
using namespace std;

ros::Publisher pub;
const float PI = 3.14159265;
float rate = 100;
turtlesim::Pose current_pose;

geometry_msgs::Twist getMessage(double linear_x, double angular_z)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    return msg;
}

void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    current_pose = *msg;
    if(current_pose.theta > PI) current_pose.theta -=2*PI;
    if(current_pose.theta < -PI) current_pose.theta +=2*PI;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "myturtle_control");
    
    cout<<"argc = "<<argc<<endl;
    for(int i=0; i<argc; i++)
    {
        cout<<"argv["<< i <<"]="<<argv[i]<<endl;
    }
    ros::NodeHandle h;
    pub = h.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    ros::Subscriber sub =
        h.subscribe("/turtle1/pose", 1000, poseCallback);
    ros::Rate loopRate(rate);

    double x0, y0;
    const double tolerance = 1e-2;
    int i=1;
    //while (ros::ok())
    //{
 
        for(int i=1; i<argc-1; i+=2){
              
                 x0 = atof(argv[i]);
                 y0 = atof(argv[i+1]);
                 
                
              
        double dx1 = x0 - current_pose.x;
        double dy1 = y0 - current_pose.y;
        double distance1 = sqrt(dx1*dx1+dy1*dy1);
        double theta1 = current_pose.theta;
        
        double theta_goal = abs(acos(dx1/ distance1));
        while (ros::ok()) 
          {
            
            
            loopRate.sleep();
            ros::spinOnce();
            cout << current_pose.x << " " << current_pose.y << " " << current_pose.theta << endl;

            double distance = sqrt( pow(x0-current_pose.x, 2) + pow(y0-current_pose.y, 2) );
            if (distance < tolerance) { 
                pub.publish(getMessage(0,0));
                break;
            }
          
            double dx = x0 - current_pose.x, dy = y0 - current_pose.y, theta = current_pose.theta;
            double alpha = acos ((cos(theta)*dx+sin(theta)*dy) / distance);
            double angle = asin((cos(theta)*dy-sin(theta)*dx) / distance);
            double dalpha = abs(alpha);

            if(distance > tolerance){
            if(i == 1){
               pub.publish(getMessage(1.0*distance, 4.0*angle));
            }
            else{
            if(theta1 > 0 && theta1 != PI){
               if(dy1 > 0){
                  if(theta_goal <= theta1){
                     pub.publish(getMessage(1.5*distance, -4.5*dalpha));
                  }
                  else{
                     pub.publish(getMessage(1.5*distance, 4.5*dalpha));
                  }
               }
               else if( dy1 < 0){
                  if(PI-theta_goal <= theta1){
                     pub.publish(getMessage(-1.5*distance, -4.5*(PI-dalpha)));
                  }
                  else{
                     pub.publish(getMessage(-1.5*distance, 4.5*(PI-dalpha)));
                  }
               }
               else{
                  if(dx1 > 0){
                     if(theta1 > PI/2){
                       pub.publish(getMessage(-1.5*distance, 4.5*(PI-dalpha)));
                     }
                     else{
                       pub.publish(getMessage(1.5*distance, -4.5*dalpha));
                     }
                  }
                  else{
                     if(theta1 > PI/2){
                       pub.publish(getMessage(1.5*distance, 4.5*dalpha));
                     }
                     else{
                       pub.publish(getMessage(-1.5*distance, -4.5*(PI-dalpha)));
                     }
                  }
               }
            }
            else if(theta1 < 0 && theta1 != -PI){
               if(dy1 > 0){
                  if(-PI+theta_goal <= theta1){
                     pub.publish(getMessage(-1.5*distance, -4.5*(PI-dalpha)));
                  }
                  else{
                     pub.publish(getMessage(-1.5*distance, 4.5*(PI-dalpha)));
                  }
               }
               else if( dy1 < 0){
                  if(-theta_goal <= theta1){
                     pub.publish(getMessage(1.5*distance, -4.5*dalpha));
                  }
                  else{
                     pub.publish(getMessage(1.5*distance, 4.5*dalpha));
                  }
               }
               else{
                  if(dx1 > 0){
                     if(theta1 >= -PI/2){
                       pub.publish(getMessage(1.5*distance, 4.5*dalpha));
                     }
                     else{
                       pub.publish(getMessage(-1.5*distance, -4.5*(PI-dalpha)));
                     }
                  }
                  else{
                     if(theta1 >= -PI/2){
                       pub.publish(getMessage(-1.5*distance, 4.5*(PI-dalpha)));
                     }
                     else{
                       pub.publish(getMessage(1.5*distance, -4.5*dalpha));
                     }
                  }
               }
            }
            else if(theta1 == -PI || theta1 == PI ){
              if(dy1 > 0){
                if(theta_goal <= PI/2){
                   pub.publish(getMessage(-1.5*distance, 4.5*(PI-dalpha)));
                }
                else{
                   pub.publish(getMessage(1.5*distance, -4.5*dalpha));
                }
              }
              else if(dy1 < 0){
                if(theta_goal <= PI/2){
                   pub.publish(getMessage(-1.5*distance, -4.5*(PI-dalpha)));
                }
                else{
                   pub.publish(getMessage(1.5*distance, 4.5*dalpha));
                }
              }
              else{
                if(dx1 > 0){
                   pub.publish(getMessage(-2*distance, 0));
                }
                else if(dx1 < 0){
                   pub.publish(getMessage(2*distance, 0));
                }
              }
            }
            else if(theta1 == 0){
              if(dy1 > 0){
                if(theta_goal <= PI/2){
                   pub.publish(getMessage(-1.5*distance, 4.5*(PI-dalpha)));
                }
                else{
                   pub.publish(getMessage(1.5*distance, -4.5*dalpha));
                }
              }
              else if(dy1 < 0){
                if(theta_goal <= PI/2){
                   pub.publish(getMessage(1.5*distance, -4.5*dalpha));
                }
                else{
                   pub.publish(getMessage(-1.5*distance, 4.5*(PI-dalpha)));
                }
              }
              else{
                if(dx1 > 0){
                   pub.publish(getMessage(2*distance, 0));
                }
                else if(dx1 < 0){
                   pub.publish(getMessage(-2.0*distance, 0));
                }
              }
            }
            }    
          }
          }
    }
    return 0;
}
