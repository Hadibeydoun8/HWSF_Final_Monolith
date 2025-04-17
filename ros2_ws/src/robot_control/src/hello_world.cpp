/*
 * analogReadcpp.cpp
 *
 *  Created on: April 9, 2024
 *      Author: utayba
 * This program subscribes to turtle1/pose and shows its
 * messages on the screen.
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <stdlib.h>
#include <string>


#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <inttypes.h>

#include "robot_interfaces/msg/wisker_data.hpp"



//using std::placeholders::_1;


class analogRead : public rclcpp::Node {
public:
	analogRead():Node("analog_publisher"){
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("my_topic", 100);

		tcgetattr(STDOUT_FILENO,&old_stdio);

		memset(&stdio,0,sizeof(stdio));

		stdio.c_lflag |= ECHO;
		stdio.c_iflag |= ICRNL;
		stdio.c_oflag |= (OPOST | ONLCR);

		tcsetattr(STDOUT_FILENO,TCSAFLUSH,&stdio);
		fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);       // make the reads non-blocking

		tty_fd=open("/dev/ttyACM0", O_RDWR | O_NONBLOCK);
		tcgetattr(tty_fd,&tio);


		tio.c_cflag &= ~CSIZE;
		tio.c_cflag|= CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
		tio.c_cflag &= ~PARENB; //No Parity
		tio.c_cflag &= ~CSTOPB; // 1 stop bit
		tio.c_lflag=0;

		cfsetospeed(&tio,B9600);
		cfsetispeed(&tio,B9600);
		tcsetattr(tty_fd,TCSAFLUSH,&tio);
		capture_data();

	}

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  	struct termios tio;
	struct termios stdio;
	struct termios old_stdio;
	int tty_fd;
	unsigned char c='D';
    char rxString[40];
	int ind,atdX, atdY;
	float voltageX, voltageY;
	double normalX, normalY;
	char inchar='b';
	geometry_msgs::msg::Twist msg;



//  void capture_data(const turtlesim::msg::Pose & msg) const {
//    RCLCPP_INFO(this->get_logger(),"position=(%.2f , %.2f) ,direction= %.2f",msg.x, msg.y,msg.theta);
void capture_data(){
	while (c!='q')
	{
		if (read(tty_fd,&inchar,1)>0){
			if (inchar == 'd') {
				rxString[0]=inchar;
				ind=1;
				while(inchar != '\n'){
					if(read(tty_fd,&inchar,1)>0){
						if (ind<40) {
							rxString[ind]=inchar;
							ind++;

						} else {
							inchar = '\n';
						}
					}
				 }
				if (ind<40) {
					sscanf(rxString,"d:%d\t%f\t%d\t%f",&atdX, &voltageX, &atdY, &voltageY);
					normalX = double(voltageX-2.5)/2.5;
					normalY = double(voltageY-2.5)/2.5;
					printf("VoltageX: %f\t VoltageY: %.2f\n",normalX, normalY);
					msg.linear.x=normalX;
					msg.linear.y=normalY;
					publisher_->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "Publishing: Linear X = %.2f, Linear Y = %.2f", msg.linear.x, msg.linear.y);

				}
			}
		}
		read(STDIN_FILENO,&c,1);
	}
	close(tty_fd);
	tcsetattr(STDOUT_FILENO,TCSANOW,&old_stdio);
  }


};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node=std::make_shared<analogRead>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}