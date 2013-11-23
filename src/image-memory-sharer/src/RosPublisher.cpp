/*
 * RosReceiver.cpp
    Copyright (C) 2013  Timothy Sweet

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "SharedMessageReader.h"
#include <cv_bridge/cv_bridge.h>

class Receiver {
public:
	Receiver() {
		out_stream_ = handle_.advertise<sensor_msgs::Image>("image",1);
	}
	void callback(const ros::TimerEvent& event);
private:
	SharedMessageReader message_reader_;
	ros::Publisher out_stream_;
	ros::NodeHandle handle_;
};

sensor_msgs::ImageConstPtr ConvertMatToMsg(const cv::Mat& frame);


int main(int argc,char* argv[]) {

	ros::init(argc,argv,"RosPublisher");
	ros::NodeHandle handle;
	//create a timer to  get the frame at 30 Hz
	Receiver receiver;
	ros::Timer timer = handle.createTimer(ros::Duration(1/30.),&Receiver::callback,&receiver);

	ros::spin();
}

void Receiver::callback(const ros::TimerEvent& event) {
	cv::Mat frame = message_reader_.retrieve();
	out_stream_.publish(ConvertMatToMsg(frame));
}

sensor_msgs::ImageConstPtr ConvertMatToMsg(const cv::Mat& frame){
	std_msgs::Header header;
	static int sequence_number=0;
	header.seq = sequence_number++;
	std::string encoding;

	assert(frame.channels()==1 || frame.channels()==3);
	if(frame.channels()==1) encoding="mono8";
	else if(frame.channels()==3) encoding="bgr8";

	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage(header,encoding,frame));
	return cv_ptr->toImageMsg();
}
