#include <iostream>
#include <fstream>
#include <iomanip>
#include <unistd.h>
#include <set>
#include "OnlineSortTracker.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

void getData(string seqName, vector<vector<TrackingBox>>& frames) {
	// 1. read detection file
	ifstream detectionFile;
	detectionFile.open(seqName);
	if (!detectionFile.is_open()) { cerr << "Error: can not find file " << seqName << endl; return; }
	string detLine;
	istringstream ss;
	vector<TrackingBox> detData;
	char ch;
	float tpx, tpy, tpw, tph;
	while ( getline(detectionFile, detLine) ) {
		TrackingBox tb;
		ss.str(detLine);
		// Format is: n-frame, id, x, y, w, h, probability, class, -1
		ss >> tb.frame >> ch >> tb.id >> ch >> tpx >> ch >> tpy >> ch >> tpw >> ch >> tph >> ch >> tb.p >> ch >> tb.cls;
		ss.str("");
		// std::cout << "f:" << tb.frame << "\tid:" << tb.id << "\txy:(" << tpx << ", " << tpy << ")\twh:(" << tpw << ", " << tph << ")\n";
		// Build boxes, put them in detData.
		tb.box = Rect_<float>(Point_<float>(tpx, tpy), Point_<float>(tpx + tpw, tpy + tph));
		detData.push_back(tb);
	}
	detectionFile.close();
	// 2. group detData by frame
	size_t maxFrame = 0;
	// find max frame number
	for (auto tb: detData) if (maxFrame < tb.frame) maxFrame = tb.frame;
	// std::cout << "maxFrame: " << maxFrame << "\n";
	vector<TrackingBox> tempVec;
	for (int fi = 0; fi < maxFrame; fi++) {
		// frame num starts from 1
		for (auto tb: detData) if (tb.frame == fi + 1) tempVec.push_back(tb);
		// frames is a vector of vectors
		frames.push_back(tempVec);
		tempVec.clear();
	}
}

void doSORT(string seqName) {
	// Data is a vector of frames, each frame containing a vector of boxes
	vector<vector<TrackingBox>> frames;
	// It is read from a file
	getData(seqName, frames);

	// Main loop
	OnlineSortTracker sort;
	string result;
	for (size_t fi = 0; fi < frames.size(); fi++) {
		sort.process(frames[fi]);
		// Printing entries
		int n=0;
		for(TrackingBox tb: frames[fi])
			std::cout<<tb.frame<<","<<tb.id<<"," <<tb.box.x<<"," <<tb.box.y<<"," <<tb.box.width<<"," <<tb.box.height<<"," <<tb.p<<',' <<tb.cls<<",-1\n";
		//std::cout<<std::endl;
	}
}

int main(int argc, char** argv) {
	doSORT(argv[1]);
	return 0;
}
