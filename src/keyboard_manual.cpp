#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>

// Map for movement keys
std::map<char, std::vector<float>> moveBindings
{
  {'w', {0, 1}},
  {'s', {0, -1}},
  {'a', {1, 0}},
  {'d', {-1, 0}},
};

// Map for speed keys
std::map<char, std::vector<float>> speedBindings
{
  {'u', {1.1, 1.1}},
  {'j', {0.9, 0.9}},
  {'i', {1.1, 1}},
  {'k', {0.9, 1}},
  {'o', {1, 1.1}},
  {'l', {1, 0.9}}
};

// Reminder message
const char* msg = R"(

Manual Airbase Motor Voltage Keyboard Commander
---------------------------
Joint 1:
   a    d    
Joint 2:
   w    s    


anything else : stop

u/j : increase/decrease both torques by 10%
i/k : increase/decrease only joint 1 torque by 10%
o/l : increase/decrease only joint 2 torque by 10%

CTRL-C to quit

)";

// Init variables
float voltage_a(0.01); // Joint 1 intial tension 
float voltage_b(0.01); // Joint 2 
float x(0), y(0) ; // pitch/ yaw vars
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
  ros::Publisher pub = nh.advertise<geometry_msgs::Point>("cmd_torque", 1);

  // Create Point message
  geometry_msgs::Point point;

  printf("%s", msg);
  printf("\rCurrent: Yaw Voltage  %f\tPitch Voltage  %f | Awaiting command...\r", voltage_a, voltage_b);

  while(true){

    // Get the pressed key
    key = getch();

    // If the key corresponds to a key in moveBindings
    if (moveBindings.count(key) == 1)
    {
      // Grab the direction data
      x = moveBindings[key][0];
      y = moveBindings[key][1];

      //todo sign is not displayed
      printf("\rCurrent: Yaw Voltage  %f\tPitch Voltage  %f | Last command: %c   ", voltage_a, voltage_b, key);
    }

    // Otherwise if it corresponds to a key in speedBindings
    else if (speedBindings.count(key) == 1)
    {
      // Grab the speed data
      voltage_a = voltage_a * speedBindings[key][0];
      voltage_b = voltage_b * speedBindings[key][1];

      printf("\rCurrent: Yaw Voltage  %f\tPitch Voltage  %f | Last command: %c   ", voltage_a, voltage_b, key);
    }

    // Otherwise, set the robot to stop
    else
    {
      x = 0;
      y = 0;

      // If ctrl-C (^C) was pressed, terminate the program
      if (key == '\x03')
      {
        printf("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
        break;
      }

      printf("\rCurrent: Yaw Voltage  %f\tPitch Voltage  %f | Invalid command! %c", voltage_a, voltage_b, key);
    }

    // Update the Point message
    point.x = x * voltage_a;
    point.y = y * voltage_b;

    // Publish it and resolve any remaining callbacks
    pub.publish(point);
    ros::spinOnce();
  }

  return 0;
}
