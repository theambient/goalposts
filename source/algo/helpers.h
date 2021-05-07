#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "config.h"

cv::Mat build_pattern(cv::Mat img, AlgorithmParams params, std::vector<cv::Point2f> points);
