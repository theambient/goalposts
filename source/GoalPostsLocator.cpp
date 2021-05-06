
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "GoalPostsLocator.h"
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
	size_t margin_size = _params.pattern_size + _params.search_wnd;
	cv::copyMakeBorder(_img, _img_ext, margin_size, margin_size, margin_size, margin_size, cv::BORDER_REFLECT_101);

	auto pattern = _calc_pattern();
	std::vector<cv::Point2f> located_pts;

	for(auto& ap: _approx_points)
	{
		auto start_point = cv::Point2i(ap.x - _params.pattern_size / 2, ap.y - _params.pattern_size / 2);
		auto pt_matched = _match_pattern(pattern, start_point);
		auto pt_located = cv::Point2i(pt_matched.x + ap.x - start_point.x, pt_matched.x + ap.y - start_point.y);
		located_pts.push_back(pt_located);
	}

	return std::move(located_pts);
}

cv::Point2i GoalPostsLocator::_match_pattern(cv::Mat pattern, cv::Point2i sp)
{
	float val_max = 0;
	cv::Point2i pt_max = sp;

	for(int y=sp.y-_params.search_wnd; y<sp.y+_params.search_wnd+1; ++y)
	{
		for(int x=sp.x-_params.search_wnd; x<sp.x+_params.search_wnd+1; ++x)
		{
			float val = _calc_metric(x, y, pattern, sp);
			if(val > val_max)
			{
				val_max = val;
				pt_max = sp;
			}
		}
	}

	return pt_max;
}

float GoalPostsLocator::_calc_metric(int x, int y, cv::Mat pattern, cv::Point2i sp)
{
	auto pptr = pattern.ptr<unsigned char>(sp.x, sp.y);
	auto iptr = _img_ext.ptr<unsigned char>(x, y);

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

	return abs(acc[0] / float(cnt[0]) - acc[1] / float(cnt[1]));
}

cv::Mat GoalPostsLocator::_calc_pattern()
{
    int w = _img_ext.cols;
    int h = _img_ext.rows;

    cv::Mat pattern = cv::Mat::zeros(h,w, CV_8U);

	int GH  = _params.gate_height;
	int GW  = _params.gate_width;
    int BS  = _params.bar_width;
    int BS2 = _params.bar_width / 2;
    int BW  = _params.bar_width;
    int BW2 = _params.bar_width / 2;

    cv::Point2i pts[1][8] = {{
    	cv::Point2i(0, GH),
    	cv::Point2i(0, 0),
    	cv::Point2i(GW, 0),
    	cv::Point2i(GW, GH),
    	cv::Point2i(GW - BW, GH),
    	cv::Point2i(GW - BW, BS),
    	cv::Point2i(BW, BS),
    	cv::Point2i(BW, GH),
    }};

    const cv::Point* ppt[1] = { pts[0] };
    const int npt[] = {8};

    cv::fillPoly(pattern, ppt, npt, 1, cv::Scalar(255,255,255), cv::LINE_8);

    cv::Point2f srcTri[] = {
        cv::Point2f(BW2, GH),
        cv::Point2f(BW2, BS2),
        cv::Point2f(GW - BW2, BS2),
        cv::Point2f(GW - BW2, GH),
    };

    cv::Point2f dstTri[] = {
        _approx_points[0],
        _approx_points[1],
        _approx_points[3],
        _approx_points[4],
    };

    cv::Mat warp_mat = cv::getPerspectiveTransform(srcTri, dstTri);
    cv::Mat pattern_w;
    cv::warpPerspective(pattern, pattern_w, warp_mat, cv::Size(w, h));

    return pattern_w;
}
