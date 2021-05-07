#pragma once

#include <opencv2/opencv.hpp>
#include "algo/GoalPostsLocator_If.h"
#include "config.h"
#include "pattern_metrics.h"

class GoalPostsLocator : public GoalPostsLocator_If
{
	cv::Mat _img;
	cv::Mat _img_ext;
	std::vector<cv::Point2f> _approx_points;
	AlgorithmParams _params;
	int _margin_size;
	std::unique_ptr<PatternMetric> _pattern_metric;

	cv::Point2i _match_pattern(cv::Mat pattern, cv::Point2i sp);
public:
	GoalPostsLocator(std::string img_path, std::string approx_points_path);
	GoalPostsLocator(cv::Mat img, std::string approx_points_path);

	std::vector<cv::Point2f> locate();
	std::vector<cv::Point2f> get_approx_points() const;
	void set_algorithm_params(const AlgorithmParams & params);
	void set_pattern_metric(std::unique_ptr<PatternMetric> pm);
};
