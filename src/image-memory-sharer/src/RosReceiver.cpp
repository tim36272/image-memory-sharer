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
#include "SharedMessageSaver.h"
#include <cv_bridge/cv_bridge.h>

class Sender {
public:
	Sender() : message_saver_(NULL) {}
	void callback(const sensor_msgs::ImageConstPtr& msg);
private:
	SharedMessageSaver* message_saver_;
};

cv::Mat convertMsgToCvImage(const sensor_msgs::ImageConstPtr& msg);

int main(int argc,char* argv[]) {
    //Remove shared memory on construction and destruction
    struct shm_remove
    {
       shm_remove() { boost::interprocess::shared_memory_object::remove("image_transport"); }
       ~shm_remove(){ boost::interprocess::shared_memory_object::remove("image_transport"); }
    } remover;

	ros::init(argc,argv,"RosReceiver");
	ros::NodeHandle handle;

	//register a subscriber, in the callback put the frame into memory
	Sender sender;
	ros::Subscriber sub = handle.subscribe("image",1,&Sender::callback,&sender);

	ros::spin();
}

void Sender::callback(const sensor_msgs::ImageConstPtr& msg) {

	cv::Mat frame = convertMsgToCvImage(msg);
	if(message_saver_==NULL) message_saver_ = new SharedMessageSaver(frame);
	message_saver_->store(frame);
}

cv::Mat convertMsgToCvImage(const sensor_msgs::ImageConstPtr& msg){
	//try to get the message
	try
	{
		return cv_bridge::toCvCopy(msg)->image;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	return cv::Mat();
}
