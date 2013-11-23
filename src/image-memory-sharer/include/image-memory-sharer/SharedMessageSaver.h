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
	//needs to be initialized with a frame so the system knows how much memory to allocate
	SharedMessageSaver(const cv::Mat& frame);

	//input: the frame to be stored into shared memory
	void store(const cv::Mat& frame);
private:
	boost::interprocess::managed_shared_memory memory_handle_;
	uchar *data;
	int *rows,*cols,*channels;
};
