/*
 * SharedMessageSaver.h
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

#include <boost/interprocess/managed_shared_memory.hpp>

#include <opencv2/opencv.hpp>
class SharedMessageSaver {
public:
	SharedMessageSaver(const cv::Mat& frame) : memory_handle_(boost::interprocess::create_only,
															  "image_transport",
															  frame.cols*frame.rows*frame.channels())
	{
		rows = memory_handle_.construct<int>("rows")(frame.rows);
		cols = memory_handle_.construct<int>("cols")(frame.cols);
		channels = memory_handle_.construct<int>("channels")(frame.channels());
		try {
			data = memory_handle_.construct<uchar>("image")[frame.rows*frame.cols*frame.channels()]();

		} catch (boost::interprocess::bad_alloc& e) {
			std::cout<<"Couldn't allocate at full size, trying black and white"<<std::endl;
			try {
				data = memory_handle_.construct<uchar>("image")[frame.rows*frame.cols]();
				*channels = 1;
			} catch (boost::interprocess::bad_alloc& e) {
				std::cout<<"Couldn't allocate enough space for the image, you must not have much RAM. Aborting."<<std::endl;
				assert(false);
			}
		}
		std::cout<<"Memory allocation successful"<<std::endl;
	}

	void store(const cv::Mat& frame) {
		//we need to check if the frames are being broadcast in color or grayscale
		cv::Mat temp_frame;
		if(*channels == 1) cv::cvtColor(frame,temp_frame,CV_BGR2GRAY,1);
		else temp_frame = frame;

		//now copy the image to shared memory
		uchar* ros_image_ptr = temp_frame.data;
		uchar* memory_image_ptr = data;
		for(int i=0;i<*rows * *cols;i++) {
			*memory_image_ptr = *ros_image_ptr;
			memory_image_ptr++;
			ros_image_ptr++;
		}

	}
private:
	boost::interprocess::managed_shared_memory memory_handle_;
	uchar *data;
	int *rows,*cols,*channels;
};
