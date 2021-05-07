
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>
#include "GoalPostsLocator_If.h"
#include "../config.h"
#include "pattern_metrics.h"

class GoalPostsLocator_FullMatch : public GoalPostsLocator_If
{
	cv::Mat _img;
	cv::Mat _img_ext;
	std::vector<cv::Point2f> _approx_points;
	AlgorithmParams _params;
	int _margin_size;
	std::unique_ptr<PatternMetric> _pattern_metric;

	cv::Point2i _match_pattern_fast(cv::Mat pattern, const std::vector<cv::Point2i> & start_points);
	cv::Point2i _match_pattern(cv::Mat pattern, const std::vector<cv::Point2i> & start_points);
	float _calc_metric(int x, int y, cv::Mat pattern, cv::Point2i sp);
public:
	GoalPostsLocator_FullMatch(std::string img_path, std::string approx_points_path);
	GoalPostsLocator_FullMatch(cv::Mat img, std::string approx_points_path);

	std::vector<cv::Point2f> locate() override;
	void set_algorithm_params(const AlgorithmParams & params);
	void set_pattern_metric(std::unique_ptr<PatternMetric> pm);
};

