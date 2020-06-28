#pragma once
#include <set>
#include <sstream>
#include <vector>
#include "Hungarian.h"
#include "KalmanTracker.h"
#include "opencv2/video/tracking.hpp"

typedef struct TrackingBox {
	int frame;
	int id;
	Rect_<float> box;
	TrackingBox() {}
	//TrackingBox(int f, int i, float x, float y, float w, float h):frame(f), id(i), box(x, y, w, h) {}
} TrackingBox;

class OnlineSortTracker {
public:
	OnlineSortTracker();

	void process(std::vector<TrackingBox>& frame);
	void print(string& result);

protected:
	void processFirstFrame(std::vector<TrackingBox>& frame);

	int total_frames;
	int frame_count;
	int max_age;
	int min_hits;
	double iouThreshold;
	std::vector<KalmanTracker> trackers;

	std::vector<Rect_<float>> predictedBoxes;
	std::vector<std::vector<double>> iouMatrix;
	std::vector<int> assignment;
	std::set<int> unmatchedDetections;
	std::set<int> unmatchedTrajectories;
	std::set<int> allItems;
	std::set<int> matchedItems;
	std::vector<cv::Point> matchedPairs;
	unsigned int trkNum;
	unsigned int detNum;
	//std::stringstream results;
};
