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
	SharedMessageReader();

	//output: a cv::Mat with the shared memory data
	//this is a shallow copy since Mat's have to be forced to deep copy
	cv::Mat retrieve();
private:
	boost::interprocess::managed_shared_memory memory_handle_;
	std::pair<uchar*,unsigned long int> data;
	std::pair<int*,unsigned long int> rows,cols,channels;
};
