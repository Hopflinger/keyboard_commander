#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include "std_msgs/Bool.h"

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>

// Map for speed keys
float step = 0.5;
std::map<char, std::vector<float>> speedBindings
{
  {'a', { step, 0, 0}},
  {'d', {-step, 0, 0}},
  {'w', {0,  step, 0}},
  {'s', {0, -step, 0}},
  {'u', {0, 0,  step}},
  {'j', {0, 0, -step}},
};

// Reminder message
const char* msg = R"(

Airbase Remote Control
------------------------------------------
-Increment each joint by angle step per press

g: Set current position as desired start point

Arm:  A    (Check that q_des correct before)
Disarm:  a or A

Joint 1 increment:
   a    d    
Joint 2:
   w    s    

CTRL-C to quit

)";

// Init variables
bool armed = false;
bool error_warn = false;
float q[2] = {0, 0};
float rel_q_des[2] = {0, 0};
float q1_des_0(0.0); // Joint 1 intial des angle 
float q2_des_0(0.0); // Joint 2 
float q1_des(0.0); // Joint 1 desired angle abs
float q2_des(0.0); // Joint 2 
float w1(0.0);  
float w2(0.0);
float p(0.0); // lockable joint 1 relative desired pitch angle 
char key(' ');

// For non-blocking keyboard inputs
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
} 

void jointAngleCallback(const geometry_msgs::Point::ConstPtr& msg){ 
    q[0] = msg->x;
    q[1] = msg->y;
}  

int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "keyboard_commander");
  ros::NodeHandle nh;

  ros::Subscriber jointAngle_sub = nh.subscribe("joint_angles", 100, &jointAngleCallback);

  // Init cmd_vel publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Point>("cmd_pose", 100);
  ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Vector3>("cmd_velocity", 100);
  ros::Publisher arm_pub = nh.advertise<std_msgs::Bool>("arm_cmd", 10);

  // Create Point message
  geometry_msgs::Point point;
  geometry_msgs::Vector3 velocity;
  std_msgs::Bool arm_msg;  


  while(true){

    // Get the pressed key
    key = getch();

    //if it corresponds to a key
    if (key == 'A' && !armed){

      if ( abs(q1_des-q[0]) > 4.0 || abs(q2_des-q[1]) > 4.0  )
      {
        error_warn = true;
      }
      else{
        armed = true;      
        arm_msg.data = armed;
        arm_pub.publish(arm_msg);
        printf("\nArming____________________________________");
      }

      key = ' ';
    }
    
    if ((key == 'a' || key == 'A') && armed){
      armed = false;
      arm_msg.data = armed;
      arm_pub.publish(arm_msg);

      printf("\nDisarming____________________________________");
      key = ' ';
    }

    if (key == 'g' && !armed){
      q1_des_0 = q[0];
      q2_des_0 = q[1];
      printf("\nSet current Point as Desired____________________________________");
      printf("\n ");
      key = ' ';
    }

    //if it corresponds to a key in speedBindings
    if (speedBindings.count(key) == 1)
    {
      // Grab the pos data
      q1_des = q1_des_0 + q1_des + speedBindings[key][0];
      q2_des = q2_des_0 + q2_des + speedBindings[key][1];
      p  = p  + speedBindings[key][2];
    }

    else{
      // If ctrl-C (^C) was pressed, terminate the program
      if (key == '\x03')
      {
        armed = false;
        printf("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
        break;
      }
      printf("\nInvalid command! ");
    }

    q1_des = q1_des_0 + rel_q_des[0];
    q2_des = q2_des_0 + rel_q_des[1];


    printf("%s", msg);

    if (armed){
      printf("\nStatus: ARMED");
    }
    else{
      printf("\nStatus: Disarmed");
    }
    printf("\nCurrent:   q1: %.2f\t q2: %.2f ", q[0], q[1]); 
    printf("\nDesired:   q1: %.2f\t q2: %.2f ", q1_des, q2_des);    
    printf("\nRel.Des.:  q1: %.2f\t q2: %.2f \t| Last command: %c   ", rel_q_des[0], rel_q_des[1], key);

    if (error_warn){
      printf("\nWarning: Large Delta between Set and Current Position: Arming Disabled");
    }

    printf("\n ");


    // Update the Point message
    point.x = q1_des;
    point.y = q2_des;
    point.z = p;

    velocity.x = w1;
    velocity.y = w2;

    // Publish it and resolve any remaining callbacks
    pub.publish(point);
    velocity_pub.publish(velocity);
    ros::spinOnce();
  }

  return 0;
}
