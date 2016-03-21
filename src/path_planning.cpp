#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

ros::Publisher twist_Pub;

geometry_msgs::Twist velocidade;
//Pontos de w1 e w2 que definem a trajetória
double w1x = -3;
double w1y = 1.5;
double w2x = 4;
double w2y = 1.5;
double sigma = 0.1;

//As soon as subscriber receive a massage this function will be called
void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    // Instantiate quaternion
    geometry_msgs::Quaternion qt;
    // Copy from received message
    qt = msg->pose.pose.orientation;
    // Instantiate position
    geometry_msgs::Point pt;
    // Copy from received massage
    pt = msg->pose.pose.position;
    // Define double
    double yaw = tf::getYaw(qt);
    //print to screen result
    std::cout << "Yaw: " << yaw*180/M_PI << std::endl;
    
    //Update x and y    
    double x = pt.x;
    double y = pt.y;
    
    //Distance between w1 and P
    double Ru = sqrt(pow((w1x-x),2)+pow((w1y-y),2));
    
    double teta_u = atan((y-w1y)/(x-w1x));
    
    double teta = atan((w2y-w1y)/(w2x-w1x));
    
    double beta = teta - teta_u;
    
    //Calcule d
//    double a = (w1y - w2y);
//    double b = (w2x - w1x);
//    double c = (w2y*w1x-w1y*w2x);
//    double d = fabs(a*x + b*y + c)/sqrt(pow(a,2)+pow(b,2));
    
    double R = sqrt(pow(Ru,2)+pow(sin(beta)*Ru,2));
    
    double xc = (R + sigma)*cos(teta)+w1x;
    double yc = (R + sigma)*sin(teta)+w1y;
    
    
    
    // Update x and y    
//    double x = pt.x;
//    double y = pt.y;
//    double alfa, beta, gama, d, xq, yq, xc, yc;
//    
//
////    if ((w1x < x) && (w1y > y))
////        alfa = alfa + M_PI;
////    if ((w1x < x) && (w1y < y))
////        alfa = alfa - M_PI;
//    //Calcule d
//    double a = (w1y - w2y);
//    double b = (w2x - w1x);
//    double c = (w2y*w1x-w1y*w2x);
//    d = fabs(a*x + b*y + c)/sqrt(pow(a,2)+pow(b,2));
//    
//    //Distance between w1 and P
//    double dw1_P = sqrt(pow((w1x-x),2)+pow((w1y-y),2));
//    //Distance between w1 and q
//    double dw1_Q = sqrt(pow(dw1_P,2)-pow(d,2));
//    
//  ///////////////////
//    alfa = atan((y-w1y)/(x-w1x));
//    if (alfa < 0)
//    {
//        alfa = alfa*(-1);
//    }
//    
//    beta = asin(d/dw1_P);
//    
//    //gama = (M_PI/2) - (alfa + beta);
//    ///////////////////////
//    xq = cos(beta)*dw1_Q;
//    yq = sin(beta)*dw1_P;
//    
//  //  xc = cos(????)*teta + xq;
//   // yc = sin(????)*teta + yq;
//    
    //Segue o ponto Pc
    double tau = atan((yc-y)/(xc-x));
    if ((xc < x) && (yc > y))
        tau = tau + M_PI;
    if ((xc < x) && (yc < y))
        tau = tau - M_PI;
    
    double delta_tau = 0.5*(tau - yaw);
    velocidade.angular.z = delta_tau;
    velocidade.linear.x = 0.1;
 
    twist_Pub.publish(velocidade);
    std::cout << "xc: " << xc << std::endl;
    std::cout << "yc: " << yc << std::endl;
//    std::cout << "dw1_P: " << dw1_P << std::endl;
//    std::cout << "dw1_Q: " << dw1_Q << std::endl;
//    std::cout << "d: " << d << std::endl;
//    std::cout << "alfa: " << alfa << std::endl;
//    std::cout << "beta: " << beta << std::endl;
    
}

int main(int argc, char **argv)
{
    // Start ROS within this node
    ros::init(argc, argv, "quat2yaw");
    // Create node
    ros::NodeHandle nh;
    //Subscribing to odometry topic
    ros::Subscriber odom_sub = nh.subscribe("/vrep/vehicle/odometry",1,odomCallback);
    
    twist_Pub  = nh.advertise<geometry_msgs::Twist>("path/twist",1);
    
    ros::spin();
    
}