
#include <opencv2/opencv.hpp>

class GoalPostsLocator
{
	cv::Mat _img;
	std::vector<cv::Point2f> _approx_points;
public:
	GoalPostsLocator(std::string img_path, std::string approx_points_path);
	std::vector<cv::Point2f> locate();
	std::vector<cv::Point2f> get_approx_points() const;
};
