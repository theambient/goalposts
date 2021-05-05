
#include <fstream>
#include "utils.h"

std::vector<cv::Point2f> read_points(std::string filepath)
{
	std::ifstream fin(filepath);

	enforce(fin.is_open(), "failed to open file for reading: " + filepath);

	float x, y;
	std::vector<cv::Point2f> pts;

	while(fin >> x >> y)
	{
		pts.emplace_back(x, y);
	}

	return std::move(pts);
}
