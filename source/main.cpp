
#include <stdio.h>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include "GoalPostsLocator.h"
#include "utils.h"

class App
{
public:
	int run(int argc, const char* argv[])
	{
		if(argc != 6)
		{
			printf("usage: %s <images-dir> <approximate-points-dir> <exact-points-dir> <num-images> <output-dir>\n", argv[0]);
			return 1;
		}

		_img_dir = argv[1];
		_approx_dir = argv[2];
		_exact_dir = argv[3];
		int num_images = atoi(argv[4]);
		_output_dir = argv[5];

		for(int i=0; i<num_images; ++i)
		{
			_locate_gate(i);
		}

		return 0;
	}

private:

	void _locate_gate(int image_idx)
	{
		char img_path[MAX_PATH_SIZE], ap_path[MAX_PATH_SIZE], ep_path[MAX_PATH_SIZE], out_path[MAX_PATH_SIZE];
		sprintf(img_path, "%s/%04d.png", _img_dir.c_str(), image_idx);
		sprintf(ap_path, "%s/approximateGoalPosition%04d.txt", _approx_dir.c_str(), image_idx);
		sprintf(ep_path, "%s/exactGoalPosition%04d.txt", _exact_dir.c_str(), image_idx);
		sprintf(out_path, "%s/algorithmGoalPosition%04d.txt", _output_dir.c_str(), image_idx);

		auto gl = GoalPostsLocator(img_path, ap_path);
		auto located_pts = gl.locate();
		auto approx_pts = gl.get_approx_points();
		auto exact_pts = read_points(ep_path);

		std::ofstream file(out_path);
		cv::Vec2f total_diff(0,0);
		for(size_t i=0; i<located_pts.size(); ++i)
		{
			auto lp = located_pts[i];
			auto ap = approx_pts[i];
			auto ep = exact_pts[i];

			total_diff += cv::Vec2f(lp) - cv::Vec2f(ep);

			file << lp.x << " " << lp.y;
		}

		char buf[1024];
		sprintf(buf, "%04d (%.1f, %.1f) %.1f", image_idx, total_diff[0], total_diff[1], cv::norm(total_diff));
		_results_file << buf << std::endl;
	}

	std::string _img_dir;
	std::string _approx_dir;
	std::string _exact_dir;
	std::string _output_dir;
	std::ofstream _results_file;
};

int main(int argc, const char* argv[])
{
	auto app = App();
	return app.run(argc, argv);
}
