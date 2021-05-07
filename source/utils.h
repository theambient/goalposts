#pragma once

#include <opencv2/opencv.hpp>

#define MAX_PATH_SIZE 1024

template<typename Except = std::runtime_error, typename... Args>
void enforce(bool condition, Args... args)
{
	if(!condition)
	{
		throw Except(args...);
	}
}

#define ENFORCE(cond, ...) \
	enforce<EnforceEx>(cond, #cond, __FILE__, __LINE__, ## __VA_ARGS__)

std::vector<cv::Point2f> read_points(std::string filepath);
