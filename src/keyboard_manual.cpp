#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>

// Map for speed keys
std::map<char, std::vector<float>> speedBindings
{
  {'w', {0.05, 0.0, 0.0, 0.0}},
  {'s', {-0.05, 0.0, 0.0, 0.0}},
  {'e', {0.0, 0.05, 0.0, 0.0}},
  {'d', {0.0, -0.05, 0.0, 0.0}},
  {'u', {0.0, 0.0, 0.05, 0.0}},
  {'j', {0.0, 0.0, -0.05, 0.0}},
  {'i', {0.0, 0.0, 0.0, 0.05}},
  {'k', {0.0, 0.0, 0.0, -0.05}}
};

// Reminder message
const char* msg = R"(

Manual Airbase Motor Voltage Keyboard Commander
---------------------------
Joint 1 Motors:
   w,s  e,d      
Joint 2 Motors:
   u,j  i,k    


anything else : stop

u/j : increase/decrease both torques by 10%
i/k : increase/decrease only joint 1 torque by 10%
o/l : increase/decrease only joint 2 torque by 10%

CTRL-C to quit

)";

// Init variables
float voltage_1(0.0); // V 1 intial  
float voltage_2(0.0); // V 2
float voltage_3(0.0); 
float voltage_4(0.0);
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

int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "keyboard_manual");
  ros::NodeHandle nh;

  // Init cmd_vel publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Quaternion>("cmd_torque", 1);

  // Create Quaternion message
  geometry_msgs::Quaternion command;

  printf("%s", msg);
  printf("\rCurrent: V1 %.3f  V2 %.3f  V3 %.3f  V4 %.3f | Awaiting command\r", voltage_1, voltage_2, voltage_3, voltage_4);

  while(true){

    // Get the pressed key
    key = getch();

    // Otherwise if it corresponds to a key in speedBindings
    if (speedBindings.count(key) == 1)
    {
      // Grab the speed data
      voltage_1 = voltage_1 + speedBindings[key][0];
      voltage_2 = voltage_2 + speedBindings[key][1];
      voltage_3 = voltage_3 + speedBindings[key][2];
      voltage_4 = voltage_4 + speedBindings[key][3];

      printf("\rCurrent: V1 %.3f  V2 %.3f  V3 %.3f  V4 %.3f | Last command: %c   ", voltage_1, voltage_2, voltage_3, voltage_4, key);
    }

    // Otherwise, set the robot to stop
    else
    {
      voltage_1 = 0;
      voltage_2 = 0;
      voltage_3 = 0;
      voltage_4 = 0;

      // If ctrl-C (^C) was pressed, terminate the program
      if (key == '\x03')
      {
        printf("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
        break;
      }

      printf("\rCurrent: V1 %.3f  V2 %.3f  V3 %.3f  V4 %.3f | Invalid command! %c", voltage_1, voltage_2, voltage_3, voltage_4, key);
    }

    // Update the Quaternion message
    command.x = voltage_1;
    command.y = voltage_2;
    command.z = voltage_3;
    command.w = voltage_4;

    // Publish it and resolve any remaining callbacks
    pub.publish(command);
    ros::spinOnce();
  }

  return 0;
}
