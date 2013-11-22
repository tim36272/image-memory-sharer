/*
 * SharedMessageReader.h
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
class SharedMessageReader {
public:
	SharedMessageReader() : memory_handle_(boost::interprocess::open_only,"image_transport")
	{
		data = memory_handle_.find<uchar>("image");
		rows = memory_handle_.find<int>("rows");
		cols = memory_handle_.find<int>("cols");
		channels = memory_handle_.find<int>("channels");
		assert(data.first!=0);
		assert(rows.first!=0);
		assert(cols.first!=0);
		assert(channels.first!=0);

	}

	void retrieve(cv::Mat* frame) {
		frame->create(*(rows.first),*(cols.first),CV_8UC1);

		//these pointers will iterate through the entire image
		uchar* ros_image_ptr = frame->data;
		uchar* memory_image_ptr = data.first;

		for(int i=0;i<*rows.first * *cols.first * *channels.first;i++) {
			*ros_image_ptr = *memory_image_ptr;
			memory_image_ptr++;
			ros_image_ptr++;
		}
	}
private:
	boost::interprocess::managed_shared_memory memory_handle_;
	std::pair<uchar*,unsigned long int> data;
	std::pair<int*,unsigned long int> rows,cols,channels;
};
