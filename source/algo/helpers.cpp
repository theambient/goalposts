
#include "helpers.h"

cv::Mat build_pattern(cv::Mat img, AlgorithmParams params, std::vector<cv::Point2f> points)
{
    int w = img.cols;
    int h = img.rows;

    cv::Mat pattern = cv::Mat::zeros(h,w, CV_8U);

	int GH  = params.gate_height;
	int GW  = params.gate_width;
    int BS  = params.bar_width;
    int BS2 = params.bar_width / 2;
    int BW  = params.bar_width;
    int BW2 = params.bar_width / 2;

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
        points[0],
        points[1],
        points[3],
        points[4],
    };

    cv::Mat warp_mat = cv::getPerspectiveTransform(srcTri, dstTri);
    cv::Mat pattern_w;
    cv::warpPerspective(pattern, pattern_w, warp_mat, cv::Size(w, h));

    return pattern_w;
}
