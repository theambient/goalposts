
#include <opencv2/opencv.hpp>

class GoalPostsLocator
{
	cv::Mat _img;
	cv::Mat _img_ext;
	std::vector<cv::Point2f> _approx_points;
	int _pattern_size;
	int _search_wnd_size;

	cv::Point2i _match_pattern(cv::Mat pattern, cv::Point2i sp);
	float _calc_metric(int x, int y, cv::Mat pattern, cv::Point2i sp);
	cv::Mat _calc_pattern();
public:
	GoalPostsLocator(std::string img_path, std::string approx_points_path);
	GoalPostsLocator(cv::Mat img, std::string approx_points_path);

	std::vector<cv::Point2f> locate();
	std::vector<cv::Point2f> get_approx_points() const;
};
