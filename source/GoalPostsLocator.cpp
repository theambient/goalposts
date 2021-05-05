
#include <opencv2/opencv.hpp>
#include "GoalPostsLocator.h"
#include "utils.h"

GoalPostsLocator::GoalPostsLocator(std::string img_path, std::string approx_points_path)
{
	_img = cv::imread(img_path);
	_approx_points = read_points(approx_points_path);
}

std::vector<cv::Point2f> GoalPostsLocator::get_approx_points() const
{
	return _approx_points;
}

std::vector<cv::Point2f> GoalPostsLocator::locate()
{
	return _approx_points;
}
