
#include <opencv2/opencv.hpp>

class GoalPostsLocator
{
public:
	GoalPostsLocator(std::string img_path, std::string approx_points_path);
	std::vector<cv::Point2f> locate();
	std::vector<cv::Point2f> get_approx_points();
};
