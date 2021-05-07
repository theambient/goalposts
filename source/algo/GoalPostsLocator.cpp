
#include <opencv2/opencv.hpp>
#include "GoalPostsLocator.h"
#include "algo/helpers.h"
#include "utils.h"

GoalPostsLocator::GoalPostsLocator(std::string img_path, std::string approx_points_path)
{
	_img = cv::imread(img_path);
	_approx_points = read_points(approx_points_path);
}

GoalPostsLocator::GoalPostsLocator(cv::Mat img, std::string approx_points_path)
{
	_img = img;
	_approx_points = read_points(approx_points_path);
}

std::vector<cv::Point2f> GoalPostsLocator::get_approx_points() const
{
	return _approx_points;
}

void GoalPostsLocator::set_algorithm_params(const AlgorithmParams & params)
{
	_params = params;
}

std::vector<cv::Point2f> GoalPostsLocator::locate()
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

	for(auto& ap: _approx_points)
	{
		auto start_point = cv::Point2i(ap.x - _params.pattern_size / 2, ap.y - _params.pattern_size / 2);
		auto pt_matched = _match_pattern(pattern, start_point);
		auto pt_located = cv::Point2i(pt_matched.x + ap.x - start_point.x, pt_matched.y + ap.y - start_point.y);
		located_pts.push_back(pt_located);
	}

	for(size_t i=0; i<located_pts.size(); ++i)
	{
		located_pts[i].x -= _approx_points[i].x;
		located_pts[i].y -= _approx_points[i].y;
	}

	return std::move(located_pts);
}

cv::Point2i GoalPostsLocator::_match_pattern(cv::Mat pattern, cv::Point2i sp)
{
	float val_max = 0;
	cv::Point2i pt_max = sp;

	for(int y=std::max(0,sp.y-_params.search_wnd); y<std::min(sp.y+_params.search_wnd+1, _img_ext.rows - _params.pattern_size); ++y)
	{
		for(int x=std::max(0, sp.x-_params.search_wnd); x<std::min(sp.x+_params.search_wnd+1,_img_ext.cols - _params.pattern_size); ++x)
		{
			float val = _calc_metric(x, y, pattern, sp);
			if(val > val_max)
			{
				val_max = val;
				pt_max = cv::Point2i(x, y);
			}
		}
	}

	return pt_max;
}

float GoalPostsLocator::_calc_metric(int x, int y, cv::Mat pattern, cv::Point2i sp)
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
