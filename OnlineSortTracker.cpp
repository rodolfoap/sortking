#include <vector>
#include <set>
#include <sstream>
#include <vector>
#include "opencv2/video/tracking.hpp"
#include "OnlineSortTracker.h"
#include "Hungarian.h"
#include "KalmanTracker.h"

// Computes IOU between two bounding boxes
double GetIOU(Rect_<float> bb_test, Rect_<float> bb_gt) {
	float in = (bb_test & bb_gt).area();
	float un = bb_test.area() + bb_gt.area() - in;
	if(un < DBL_EPSILON) return 0;
	return (double)(in/un);
}

OnlineSortTracker::OnlineSortTracker() :
	total_frames(0),
	frame_count(0),
	max_age(1),
	min_hits(3),
	iouThreshold(0.3),
	trkNum(0),
	detNum(0) {
	KalmanTracker::kf_count = 0;
}

void OnlineSortTracker::processFirstFrame(std::vector<TrackingBox>& frame) {
	//std::vector<TrackingBox> vresults;
	// Initialize Kalman trackers using detections in first frame.
	for(unsigned int i=0; i<frame.size(); i++) {
		KalmanTracker trk=KalmanTracker(frame[i].box);
		trackers.push_back(trk);
		frame[i].id=trk.m_id+1;
	}
}

// Entry point
void OnlineSortTracker::process(std::vector<TrackingBox>& frame) {
	total_frames++;
	frame_count++;
	// If this is the first frame, treatment is not the same.
	if (trackers.size() == 0) { processFirstFrame(frame); return; }
	// Get predicted locations from existing trackers.
	predictedBoxes.clear();
	for (auto it = trackers.begin(); it != trackers.end();) {
		Rect_<float> pBox = (*it).predict();
		if (pBox.x >= 0 && pBox.y >= 0) {
			predictedBoxes.push_back(pBox);
			it++;
		} else {
			it = trackers.erase(it);
			//cerr << "Box invalid at frame: " << frame_count << endl;
		}
	}
	// Associate detections to tracked object (both represented as bounding boxes)
	trkNum = predictedBoxes.size();
	detNum = frame.size();
	iouMatrix.clear();
	iouMatrix.resize(trkNum, vector<double>(detNum, 0));
	// compute iou matrix as a distance matrix
	for (unsigned int i = 0; i < trkNum; i++) {
		for (unsigned int j = 0; j < detNum; j++) {
			// use 1-iou because the hungarian algorithm computes a minimum-cost assignment.
			iouMatrix[i][j] = 1 - GetIOU(predictedBoxes[i], frame[j].box);
		}
	}
	// solve the assignment problem using hungarian algorithm.
	// the resulting assignment is [track(prediction) : detection], with len=preNum
	HungarianAlgorithm HungAlgo;
	assignment.clear();
	HungAlgo.Solve(iouMatrix, assignment);
	// find matches, unmatched_detections and unmatched_predictions
	unmatchedTrajectories.clear();
	unmatchedDetections.clear();
	allItems.clear();
	matchedItems.clear();
	// unmatched detections
	if (detNum > trkNum) {
		for (unsigned int n = 0; n < detNum; n++)
			allItems.insert(n);
		for (unsigned int i = 0; i < trkNum; ++i)
			matchedItems.insert(assignment[i]);
		set_difference(allItems.begin(), allItems.end(),
					   matchedItems.begin(), matchedItems.end(),
					   insert_iterator<set<int>>(unmatchedDetections, unmatchedDetections.begin()));
	}
	// unmatched trajectory/predictions
	else if (detNum < trkNum) {
		for (unsigned int i = 0; i < trkNum; ++i)
			// unassigned label will be set as -1 in the assignment algorithm
			if (assignment[i] == -1) unmatchedTrajectories.insert(i);
	} else
		;
	// filter out matched with low IOU
	matchedPairs.clear();
	for (unsigned int i = 0; i < trkNum; ++i) {
		// pass over invalid values
		if (assignment[i] == -1) continue;
		if (1 - iouMatrix[i][assignment[i]] < iouThreshold) {
			unmatchedTrajectories.insert(i);
			unmatchedDetections.insert(assignment[i]);
		} else {
			matchedPairs.push_back(cv::Point(i, assignment[i]));
		}
	}
	// update trackers
	// update matched trackers with assigned detections.
	// each prediction is corresponding to a tracker
	int detIdx, trkIdx;
	for (unsigned int i = 0; i < matchedPairs.size(); i++) {
		trkIdx = matchedPairs[i].x;
		detIdx = matchedPairs[i].y;
		trackers[trkIdx].update(frame[detIdx].box);
		frame[detIdx].id=trackers[trkIdx].m_id+1;
	}
	// create and initialise new trackers for unmatched detections
	for (auto umd : unmatchedDetections) {
		KalmanTracker tracker = KalmanTracker(frame[umd].box);
		trackers.push_back(tracker);
		frame[umd].id=tracker.m_id+1;
	}
	// get trackers' output
	for (auto it = trackers.begin(); it != trackers.end();) {
		it++;
		// remove dead tracklet
		if (it != trackers.end() && (*it).m_time_since_update > max_age) { it = trackers.erase(it); }
	}
}
