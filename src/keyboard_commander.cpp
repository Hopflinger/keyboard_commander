#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

# define pi           3.14159265358979323846  /* pi */
//#include<conio.h>
#include "keyboard.h"


// Reminder message
const char* msg = R"(

Reading from the keyboard and Publishing to Point!
---------------------------
start circle trajectory:
   u    
Position control

anything else : stop

CTRL-C to quit

)";

// Init variables
float q1_0(0.0); // Joint 1 intial  angle 
float q2_0(0.0); // Joint 2 
float q1(0.0); // Joint 1 intial relative angle 
float q2(0.0); // Joint 2 
char key(' ');
bool trajrun = false;
double alpha = 0.0;
double t_0 = 0.0;
double r = 20; //circle radius
double v_alpha = 2.0 * pi/180;
double alpha_0 = 0.0;

// For non-blocking keyboard inputs
/* int getch(void)
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
} */

int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "keyboard_commander");
  ros::NodeHandle nh;
   init_keyboard();

  // Init cmd_vel publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Point>("cmd_torque", 10);

  // Create Point message
  geometry_msgs::Point point;

  printf("%s", msg);
  printf("\rCurrent: Delta q1 %.2f\t Delta q2 %.2f | Awaiting command...\r", q1, q2);

  //ros::Rate rate(100); // 100 hz
  while(true){

    // Get the pressed key non blcking with kbhit
    if( _kbhit()){
      key = _getch();
    }


    //if it corresponds to a key
    if (key == 'u')
    {
      t_0 = ros::Time::now().toSec();
      //get q0
      trajrun = true;

      printf("\rStarting Circle: Delta q1 %.2f\t Delta q2 %.2f | Last command: %c   ", q1, q2, key);
      key = ' ';
    }

    if (trajrun == true){
      double t = ros::Time::now().toSec();
      double dt = t - t_0;
      alpha = alpha_0 + dt * v_alpha;
      q1 = r * cos(alpha - pi/2);
      q2 = r + r * sin(alpha - pi/2);

      printf("\rDoing Circle: Delta q1 %.3f\t Delta q2 %.3f | Last command: %c   ", q1, q2, key);
    }

    // Otherwise, set the robot to stop
    if (key == 'j')
    {
      trajrun = false;
      alpha_0 = alpha;
      key = ' ';


      // If ctrl-C (^C) was pressed, terminate the program
      if (key == '\x03')
      {
        printf("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
        break;
      }

      printf("\rCurrent: Delta q1 %.2f\t Delta q2 %.2f | Invalid command! %c", q1, q2, key);
    }

    // Update the Point message
    point.x = q1;
    point.y = q2;

    // Publish it and resolve any remaining callbacks
    pub.publish(point);
    ros::spinOnce();
    //rate.sleep();
  }
  close_keyboard();
  return 0;
}
