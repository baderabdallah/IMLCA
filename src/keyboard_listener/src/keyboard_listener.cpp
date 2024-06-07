#include <ros/ros.h>
#include <std_msgs/String.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sstream>
#include <unordered_map>

std::string getch() {
    char buf[3] = {0};
    struct termios old = {0};

    if (tcgetattr(0, &old) < 0)
        perror("tcgetattr()");
    
    old.c_lflag &= ~ICANON; // 
    old.c_lflag &= ~ECHO;   // The input is not displayed as normal terminal input
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    
    int num_read = read(0, buf, 1);
    if (buf[0] == '\x1b') {
        if (read(0, buf+1, 2) < 0)
            perror("read()");
        num_read = 3;
    }
    
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");

    return std::string(buf, num_read);
}

std::string convertSpecialKeys(const std::string& input) {
    static const std::unordered_map<std::string, std::string> key_map = {
        {"\x1b[A", "Up Arrow"},
        {"\x1b[B", "Down Arrow"},
        {"\x1b[C", "Right Arrow"},
        {"\x1b[D", "Left Arrow"},
    };

    auto it = key_map.find(input);
    if (it != key_map.end()) {
        return it->second;
    }
    return input;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard_listener");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("keyboard_input", 10);
    
    ros::Rate loop_rate(10); // 10 Hz

    while (ros::ok()) {
        std::string input = getch();

        std::string human_str = convertSpecialKeys(input);

        std_msgs::String msg;
        msg.data = human_str;
        pub.publish(msg);
        ROS_INFO("Key pressed (ASCII): %s", human_str.c_str());
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}