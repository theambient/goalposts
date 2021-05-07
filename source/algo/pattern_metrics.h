#pragma once

class PatternMetric
{
public:
	virtual float calculate(int x, int y, cv::Mat img, cv::Mat pattern, cv::Point2i sp, int pattern_size) = 0;
	virtual ~PatternMetric(){}
};

class PatternMetric_DiffOfMeans : public PatternMetric
{
public:
	virtual float calculate(int x, int y, cv::Mat img, cv::Mat pattern, cv::Point2i sp, int pattern_size) override
	{
		auto pptr = pattern.ptr<unsigned char>(sp.y, sp.x);
		auto iptr = img.ptr<unsigned char>(y, x);

		unsigned acc[2] = {0};
		unsigned cnt[2] = {0};

		int width = img.cols;
		int stride = width - pattern_size;

		for(int i=0; i<pattern_size; ++i)
		{
			for(int j=0; j<pattern_size; ++j)
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
};
