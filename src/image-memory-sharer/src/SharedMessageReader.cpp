/*
 * SharedMessageReader.cpp
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


#include "image-memory-sharer/SharedMessageReader.h"

SharedMessageReader::SharedMessageReader() : memory_handle_(boost::interprocess::open_only,"image_transport")
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

cv::Mat SharedMessageReader::retrieve() {
	//create a new CV Mat object with the user-allocated data and assign that to *frame.
	//This doesn't allocate new memory because cv::Mat operator= is a shallow copy
	cv::Mat shared_frame(*(rows.first),*(cols.first),CV_8UC1,data.first);
	return shared_frame;

}
