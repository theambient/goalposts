
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "algo/helpers.h"
#include "../utils.h"
#include "GoalPostsLocator_FullMatch.h"

GoalPostsLocator_FullMatch::GoalPostsLocator_FullMatch(std::string img_path, std::string approx_points_path)
{
	_img = cv::imread(img_path);
	_approx_points = read_points(approx_points_path);
}

GoalPostsLocator_FullMatch::GoalPostsLocator_FullMatch(cv::Mat img, std::string approx_points_path)
{
	_img = img;
	_approx_points = read_points(approx_points_path);
}

void GoalPostsLocator_FullMatch::set_algorithm_params(const AlgorithmParams & params)
{
	_params = params;
}

std::vector<cv::Point2f> GoalPostsLocator_FullMatch::locate()
{
	_margin_size = _params.pattern_size + _params.search_wnd;
	cv::copyMakeBorder(_img, _img_ext, _margin_size, _margin_size, _margin_size, _margin_size, cv::BORDER_REFLECT_101);
	for(size_t i=0; i<_approx_points.size(); ++i)
	{
		_approx_points[i].x += _margin_size;
		_approx_points[i].y += _margin_size;
	}

	auto pattern = build_pattern(_img_ext, _params, _approx_points);
	std::vector<cv::Point2f> located_pts;
	std::vector<cv::Point2i> start_points;

	for(auto& ap: _approx_points)
	{
		auto sp = cv::Point2i(ap.x - _params.pattern_size / 2, ap.y - _params.pattern_size / 2);
		start_points.push_back(sp);
	}

	auto outline_diff = _match_pattern(pattern, start_points);

	for(size_t i=0; i<start_points.size(); ++i)
	{
		located_pts.push_back(outline_diff);
	}

	return std::move(located_pts);
}

cv::Point2i GoalPostsLocator_FullMatch::_match_pattern(cv::Mat pattern, const std::vector<cv::Point2i> & start_points)
{
	float max_metric = 0;
	cv::Point2i max_displacement;
	const size_t num_points = start_points.size();

	for(int dy=-_params.search_wnd; dy<_params.search_wnd+1; ++dy)
	{
		for(int dx=-_params.search_wnd; dx<_params.search_wnd+1; ++dx)
		{
			// _img_ext.rows - _params.pattern_size
			float metric_val = 0;
			size_t cnt = 0;
			for(int pidx=0; pidx<num_points; ++pidx)
			{
				auto sp = start_points[pidx];
				int x = sp.x + dx;
				int y = sp.y + dy;

				assert(x >= 0);
				assert(y >= 0);
				assert(x < _img_ext.cols - _params.pattern_size);
				assert(y < _img_ext.rows - _params.pattern_size);

				metric_val += _calc_metric(x, y, pattern, sp);
				cnt += 1;
			}

			// printf("    _match_pattern xy: (%3d, %3d) -> %7.2f\n", dx, dy, metric_val);
			if(cnt == num_points && metric_val > max_metric)
			{
				max_metric = metric_val;
				max_displacement = cv::Point2i(dx, dy);
				// printf("    _match_pattern update xy: (%3d, %3d) -> %7.2f\n", dx, dy, metric_val);
			}
		}
	}

	return max_displacement;
}

float GoalPostsLocator_FullMatch::_calc_metric(int x, int y, cv::Mat pattern, cv::Point2i sp)
{
	auto pptr = pattern.ptr<unsigned char>(sp.y, sp.x);
	auto iptr = _img_ext.ptr<unsigned char>(y, x);

	unsigned acc[2] = {0};
	unsigned cnt[2] = {0};

	int width = pattern.cols;
	int stride = width - _params.pattern_size;

	for(int i=0; i<_params.pattern_size; ++i)
	{
		for(int j=0; j<_params.pattern_size; ++j)
		{
			acc[*pptr == 0] += *iptr;
			cnt[*pptr == 0] += 1;
			++pptr;
			++iptr;
		}

		pptr += stride;
		iptr += stride;
	}

	assert(cnt[0] > 0);
	assert(cnt[1] > 0);

	return abs(acc[0] / float(cnt[0]) - acc[1] / float(cnt[1]));
}
