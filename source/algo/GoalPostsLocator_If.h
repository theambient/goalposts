#pragma once

#include <opencv2/opencv.hpp>

class GoalPostsLocator_If
{
public:
	virtual std::vector<cv::Point2f> locate() = 0;
	virtual ~GoalPostsLocator_If(){};
};
