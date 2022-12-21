#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>

// Map for speed keys
std::map<char, std::vector<float>> speedBindings
{
  {'w', {0, 2.5}},
  {'s', {0, -2.5}},
  {'a', {2.5, 0}},
  {'d', {-2.5, 0}},
};

// Reminder message
const char* msg = R"(

Reading from the keyboard and Publishing to Point!
---------------------------
Joint 1:
   a    d    
Joint 2:
   w    s    
Increase relative joint angle position, linear curve.
Position control

anything else : stop

CTRL-C to quit

)";

// Init variables
float q1(0.0); // Joint 1 intial relative angle 
float q2(0.0); // Joint 2 
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
  ros::init(argc, argv, "keyboard_commander");
  ros::NodeHandle nh;

  // Init cmd_vel publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Point>("cmd_torque", 1);

  // Create Point message
  geometry_msgs::Point point;

  printf("%s", msg);
  printf("\rCurrent: Delta q1 %.2f\t Delta q2 %.2f | Awaiting command...\r", q1, q2);

  while(true){

    // Get the pressed key
    key = getch();

    //if it corresponds to a key in speedBindings
    if (speedBindings.count(key) == 1)
    {
      // Grab the speed data
      q1 = q1 + speedBindings[key][0];
      q2 = q2 + speedBindings[key][1];

      printf("\rCurrent: Delta q1 %.2f\t Delta q2 %.2f | Last command: %c   ", q1, q2, key);
    }

    // Otherwise, set the robot to stop
    else
    {
      //q1 = 0;
      //q2 = 0;

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
  }

  return 0;
}
