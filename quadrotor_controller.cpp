#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include  <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

double  goal [6][3]={{0,0,2},{2,2,2}};
double  tolerance = 0.05;
double  kp = 0.5;
double  ki = 0.0002;
double  kd = 0.00005;
double w;
double  error_x = 0;
double  error_y = 0;
double  error_z = 0;
double  error_w = 0;
double  prev_error_x = 0;
double  prev_error_y = 0;
double  prev_error_z = 0;
double  prev_error_w = 0;
double  rise = 1;
double  nonstop = true;
double  proportional_x = 0;
double  proportional_y = 0;
double  proportional_z = 0;
double  proportional_w = 0;
double  integral_x = 0;
double  integral_y = 0;
double  integral_z = 0;
double  integral_w = 0;
double  derivative_x = 0;
double  derivative_y = 0;
double  derivative_z = 0;
double  derivative_w = 0;
double  action_x = 0;
double  action_y = 0;
double  action_z = 0;
double  action_w = 0;
geometry_msgs ::Point  real;
geometry_msgs ::Twist  twist;
bool  must_exit = false;
int  waypoint_number = 0;

void  odoCallback(const  geometry_msgs :: PoseStamped :: ConstPtr& msg)
{
real.x=msg ->pose.position.x;
real.y=msg ->pose.position.y;

prev_error_x = error_x;
prev_error_y = error_y;

error_x = goal[waypoint_number ][0] - real.x;
error_y = goal[waypoint_number ][1] - real.y;

proportional_x = kp * error_x;
proportional_y = kp * error_y;

integral_x  += ki * error_x;
integral_y  += ki * error_y;

derivative_x = kd * (error_x  - prev_error_x);
derivative_y = kd * (error_y  - prev_error_y);

action_x = proportional_x + integral_x + derivative_x;
action_y = proportional_y + integral_y + derivative_y;

twist.linear.x = action_x;
twist.linear.y = action_y;

ROS_INFO("Error X: %0.2f \n", error_x);
ROS_INFO("Error Y: %0.2f \n", error_y);

ROS_INFO("Action X: %0.2f \n", action_x);
ROS_INFO("Action Y: %0.2f \n", action_y);

ROS_INFO("Voy  hacie  el  objetivo %d \n", waypoint_number +1);
if ((fabs(error_x) < tolerance) && (fabs(error_y) < tolerance)) {
if (must_exit  == true)
{
  twist.linear.x = 0;
  twist.linear.y = 0;

  exit (0);
}
else
{
  waypoint_number  += 1;
  rise += 1;
}
}
if (waypoint_number  == (sizeof(goal)/sizeof(goal [0])))
{
  if (nonstop)
  {
    waypoint_number = 1;
  }
  else
  {
    waypoint_number = 0;
    must_exit = true;
  } 
  }
}
int  main(int argc , char **argv)
{
  double x1,x2,x3,y1,y2,y3,x4,y4,x5,y5,m1,m2,r;
  double xi,yi,d1,d2;
  x1 = 0;
  y1 = 0;
  goal [0][0] = x1;
  goal [0][1] = y1;
  goal [0][2] = 2;
  //cout<<"Enter Ending point\n";
  x2 = 9;
  y2 = 9;
  //cout<<"Going from A("<<x1<<","<<y1<<") to B("<<x2<<","<<y2<<") .\n";
  //cout<<"\nEnter Obstacle coordinates: ";
  x3 = 1.409712; y3 = 1.501614;
  x4 = 4; y4 = 2.44116;
  x5 = 4; y5 = 5;
  //cout<<"Enter Obstacle Radius: ";
  r = 0.5;


//=====Obstacle 1 avoidance route =====
  m1 = (y2-y1)/(double)(x2-x1);
  m2 = (x1-x2)/(double)(y2-y1);
  /* we solve the linear system
      * ax+by=e
      * cx+dy=f
      */
  double a,b,e;
  double c,d,f;
  a = m1;
  b = -1;
  e = (m1*x1) - y1;
  c = m2;
  d = -1;
  f = (m2*x3) - y3;
  double determinant = a*d - b*c;
      if(determinant != 0) 
  {
          xi = (e*d - b*f)/determinant;
          yi = (a*f - e*c)/determinant;
         // printf("Cramer equations system: result, x = %f, y = %f\n", xi, yi);
  } 
  else 
  {
         // printf("Cramer equations system: determinant is zero\n"
               // "there are either no solutions or many solutions exist.\n"); 
  }

  double dist = ((x3-xi)*(x3-xi)) + ((y3-yi)*(y3-yi));
  double xr1,xr2,yr1,yr2;

  if(dist >= r*r)
  {
    //cout<<"Obstacle doesn't interfere, so move directly\n";
    //goal [6][3]={{x1,y1,2},{x2,y2,2}};
    goal [1][0] = xi;
    goal [1][1] = yi;
    goal [1][2] = 2;
    goal [2][0] = x2;
    goal [2][1] = y2;
    goal [2][2] = 2;
  }
  else
  {
    xr1 = x3 + (r)/(double)(sqrt(1 + (m2*m2) ));
    xr2 = x3 - (r)/(double)(sqrt(1 + (m2*m2) ));
    yr1 = (m2*(xr1 - x3)) + y3 ;
    yr2 = (m2*(xr2 - x3)) + y3 ;

    d1 = ((xr1-xi)*(xr1-xi)) + ((yr1-yi)*(yr1-yi));
    d2 = ((xr2-xi)*(xr2-xi)) + ((yr2-yi)*(yr2-yi));
    if(d1 <= d2)
    {
      //cout<<"Coordinate to go to is: "<<xr1<<","<<yr1<<" \n";
      //goal [6][3]={{x1,y1,2},{xr1,yr1,2},{x2,y2,2}};
      goal [2][0] = x2;
      goal [2][1] = y2;
      goal [2][2] = 2;
      goal [1][0] = xr1;
      goal [1][1] = yr1;
      goal [1][2] = 2;
    }
    else
    {
      //cout<<"Coordinate to go to is: "<<xr2<<","<<yr2<<" \n";
      //goal [6][3]={{x1,y1,2},{xr2,yr2,2},{x2,y2,2}};
      goal [2][0] = x2;
      goal [2][1] = y2;
      goal [2][2] = 2;
      goal [1][0] = xr2;
      goal [1][1] = yr2;
      goal [1][2] = 2;
    }
  }
//==========XXXXXX=============

//Obstacle 2 avoidance route =====


  x1 = goal[1][0];
  y1 = goal[1][1];
  x3 = x4;
  y3 = y4;
  m1 = (y2-y1)/(double)(x2-x1);
  m2 = (x1-x2)/(double)(y2-y1);
  /* we solve the linear system
      * ax+by=e
      * cx+dy=f
      */
  a = m1;
  b = -1;
  e = (m1*x1) - y1;
  c = m2;
  d = -1;
  f = (m2*x3) - y3;
  determinant = a*d - b*c;
      if(determinant != 0) 
  {
          xi = (e*d - b*f)/determinant;
          yi = (a*f - e*c)/determinant;
         // printf("Cramer equations system: result, x = %f, y = %f\n", xi, yi);
  } 
  else 
  {
         // printf("Cramer equations system: determinant is zero\n"
               // "there are either no solutions or many solutions exist.\n"); 
  }

  dist = ((x3-xi)*(x3-xi)) + ((y3-yi)*(y3-yi));

  if(dist >= r*r)
  {
    //cout<<"Obstacle doesn't interfere, so move directly\n";
    //goal [6][3]={{x1,y1,2},{x2,y2,2}};
    goal [2][0] = xi;
    goal [2][1] = yi;
    goal [2][2] = 2;
    goal [3][0] = x2;
    goal [3][1] = y2;
    goal [3][2] = 2;
  }
  else
  {
    xr1 = x3 + (r)/(double)(sqrt(1 + (m2*m2) ));
    xr2 = x3 - (r)/(double)(sqrt(1 + (m2*m2) ));
    yr1 = (m2*(xr1 - x3)) + y3 ;
    yr2 = (m2*(xr2 - x3)) + y3 ;
    
    d1 = ((xr1-xi)*(xr1-xi)) + ((yr1-yi)*(yr1-yi));
    d2 = ((xr2-xi)*(xr2-xi)) + ((yr2-yi)*(yr2-yi));
    if(d1 <= d2)
    {
      //cout<<"Coordinate to go to is: "<<xr1<<","<<yr1<<" \n";
      //goal [6][3]={{x1,y1,2},{xr1,yr1,2},{x2,y2,2}};
      goal [3][0] = x2;
      goal [3][1] = y2;
      goal [3][2] = 2;

      goal [2][0] = xr1;
      goal [2][1] = yr1;
      goal [2][2] = 2;
    }
    else
    {
      //cout<<"Coordinate to go to is: "<<xr2<<","<<yr2<<" \n";
      //goal [6][3]={{x1,y1,2},{xr2,yr2,2},{x2,y2,2}};
      goal [3][0] = x2;
      goal [3][1] = y2;
      goal [3][2] = 2;
      goal [2][0] = xr2;
      goal [2][1] = yr2;
      goal [2][2] = 2;
    }
  }

//===================XXXXX

/*
//Obstacle 3 avoidance route =====


  x1 = goal[2][0];
  y1 = goal[2][1];
  x3 = x5;
  y3 = y5;
  m1 = (y2-y1)/(double)(x2-x1);
  m2 = (x1-x2)/(double)(y2-y1);
  /* we solve the linear system
      * ax+by=e
      * cx+dy=f
      */
  a = m1;
  b = -1;
  e = (m1*x1) - y1;
  c = m2;
  d = -1;
  f = (m2*x3) - y3;
  determinant = a*d - b*c;
      if(determinant != 0) 
  {
          xi = (e*d - b*f)/determinant;
          yi = (a*f - e*c)/determinant;
         // printf("Cramer equations system: result, x = %f, y = %f\n", xi, yi);
  } 
  else 
  {
         // printf("Cramer equations system: determinant is zero\n"
               // "there are either no solutions or many solutions exist.\n"); 
  }

  dist = ((x3-xi)*(x3-xi)) + ((y3-yi)*(y3-yi));

  if(dist >= r*r)
  {
    //cout<<"Obstacle doesn't interfere, so move directly\n";
    //goal [6][3]={{x1,y1,2},{x2,y2,2}};
    goal [3][0] = xi;
    goal [3][1] = yi;
    goal [3][2] = 2;
    goal [4][0] = x2;
    goal [4][1] = y2;
    goal [4][2] = 2;
  }
  else
  {
    xr1 = x3 + (r)/(double)(sqrt(1 + (m2*m2) ));
    xr2 = x3 - (r)/(double)(sqrt(1 + (m2*m2) ));
    yr1 = (m2*(xr1 - x3)) + y3 ;
    yr2 = (m2*(xr2 - x3)) + y3 ;
    double d1,d2;
    d1 = ((xr1-xi)*(xr1-xi)) + ((yr1-yi)*(yr1-yi));
    d2 = ((xr2-xi)*(xr2-xi)) + ((yr2-yi)*(yr2-yi));
    if(d1 <= d2)
    {
      //cout<<"Coordinate to go to is: "<<xr1<<","<<yr1<<" \n";
      //goal [6][3]={{x1,y1,2},{xr1,yr1,2},{x2,y2,2}};
      goal [4][0] = x2;
      goal [4][1] = y2;
      goal [4][2] = 2;
      goal [3][0] = xr1;
      goal [3][1] = yr1;
      goal [3][2] = 2;
    }
    else
    {
      //cout<<"Coordinate to go to is: "<<xr2<<","<<yr2<<" \n";
      //goal [6][3]={{x1,y1,2},{xr2,yr2,2},{x2,y2,2}};
      goal [4][0] = x2;
      goal [4][1] = y2;
      goal [4][2] = 2;
      goal [3][0] = xr2;
      goal [3][1] = yr2;
      goal [3][2] = 2;
    }
  }

  ros::init(argc , argv , "talker");
  ros:: NodeHandle  np;
  ros:: NodeHandle  nh;
  ros:: Publisher  pub_vel = nh.advertise <geometry_msgs ::Twist >("cmd_vel", 1);
  ros:: Subscriber  sub = np.subscribe("/ground_truth_to_tf/pose", 1000,  odoCallback);
  ros::Rate  loop_rate (10);
  int  count = 0;
  while (ros::ok())
  {
    pub_vel.publish(twist);
    ros:: spinOnce ();
    loop_rate.sleep();
    ++count;
  }

}

