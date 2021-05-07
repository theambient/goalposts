
#include <memory>
#include <exception>
#include <stdio.h>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include "algo/GoalPostsLocator.h"
#include "algo/GoalPostsLocator_FullMatch.h"
#include "utils.h"

class App
{
public:
	int run(int argc, const char* argv[])
	{
		if(argc != 7)
		{
			printf("usage: %s <images-dir> <approximate-points-dir> <exact-points-dir> <num-images> <output-dir> <algorithm-name>\n", argv[0]);
			printf("    <algorithm-name>: v1|full-match\n");
			return 1;
		}

		_img_dir = argv[1];
		_approx_dir = argv[2];
		_exact_dir = argv[3];
		int num_images = atoi(argv[4]);
		_output_dir = argv[5];
		_algo_name = argv[6];

		auto results_filepath = _output_dir + "/" + "GoalPositionAlgorithmResults.txt";
		_results_file.open(results_filepath);
		enforce(_results_file.is_open(), "failed to open file for writing: " + results_filepath);

		for(int i=1; i<=num_images; ++i)
		{
			_locate_gate(i);
		}

		_results_file.close();

		return 0;
	}

private:

	// oversimplified factory
	std::unique_ptr<GoalPostsLocator_If> _create_locator(cv::Mat img, std::string ap_path)
	{
		std::unique_ptr<GoalPostsLocator_If> gl;
		if(_algo_name == "v1")
		{
			gl = std::make_unique<GoalPostsLocator>(img, ap_path);
		}
		else if(_algo_name == "full-match")
		{
			gl = std::make_unique<GoalPostsLocator_FullMatch>(img, ap_path);
		}
		else
		{
			throw std::runtime_error("unknown algoithm " + _algo_name);
		}

		return gl;
	}

	void _locate_gate(int image_idx)
	{
		char img_path[MAX_PATH_SIZE], ap_path[MAX_PATH_SIZE], ep_path[MAX_PATH_SIZE], out_path[MAX_PATH_SIZE];
		sprintf(img_path, "%s/im%04d.png", _img_dir.c_str(), image_idx);
		sprintf(ap_path, "%s/approximateGoalPosition%04d.txt", _approx_dir.c_str(), image_idx);
		sprintf(ep_path, "%s/exactGoalPosition%04d.txt", _exact_dir.c_str(), image_idx);
		sprintf(out_path, "%s/algorithmGoalPosition%04d.txt", _output_dir.c_str(), image_idx);

		auto img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);
		enforce(!img.empty(), "failed to load image " + std::string(img_path));

		// could be a factory
		auto gl = _create_locator(img, ap_path);
		auto located_diffs = gl->locate();
		auto approx_pts = read_points(ap_path);
		auto exact_pts = read_points(ep_path);

		std::ofstream file(out_path);
		cv::Vec2f possible_outline_diff;
		bool outline_preserved = true;
		std::vector<cv::Point2f> located_pts;
		for(size_t i=0; i<located_diffs.size(); ++i)
		{
			auto ld = located_diffs[i];
			auto ap = approx_pts[i];
			auto ep = exact_pts[i];
			auto lp = ap + ld;

			auto diff = cv::Vec2f(lp) - cv::Vec2f(ep);
			if(i == 0)
			{
				possible_outline_diff = diff;
			}
			else
			{
				if(cv::norm(diff - possible_outline_diff) > 1e-03)
				{
					outline_preserved = false;
				}
			}

			file << lp.x << " " << lp.y << std::endl;

			located_pts.push_back(lp);
		}

		char buf[1024];
		if(outline_preserved)
		{
			sprintf(buf, "%04d (%.1f, %.1f) %.1f", image_idx, possible_outline_diff[0], possible_outline_diff[1], cv::norm(possible_outline_diff));
		}
		else
		{
			sprintf(buf, "%04d outline was not preserved", image_idx);
		}
		_results_file << buf << std::endl;

		_draw_goalpost(located_pts, img, image_idx);
	}

	void _draw_goalpost(const std::vector<cv::Point2f> & points, cv::Mat img, int image_idx)
	{
		cv::Mat img_out;
		cv::cvtColor(img, img_out, cv::COLOR_GRAY2RGB);

		cv::Scalar color(0, 0, 255);

		for(auto &p: points)
		{
			cv::circle(img_out, cv::Point(int(p.x),int(p.y)), 5, color, -1);
		}

		char out_path[MAX_PATH_SIZE];
		sprintf(out_path, "%s/imageWithGoal%04d.png", _output_dir.c_str(), image_idx);

		cv::imwrite(out_path, img_out);
	}

	std::string _img_dir;
	std::string _approx_dir;
	std::string _exact_dir;
	std::string _output_dir;
	std::string _algo_name;
	std::ofstream _results_file;
};

int main(int argc, const char* argv[])
{
	auto app = App();
	return app.run(argc, argv);
}
