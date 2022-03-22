#include <math.h>
#include <complex>
#include <iostream>
#include <iomanip>
#include <complex>
#include <stdio.h>
#include <vector>
#include <cmath>
#include <fstream>
#include <string>
#include <windows.h>
#include <chrono>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include <opencv2/core/utils/logger.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp> 
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>

#include "imageReg.h"

void coordinate_calculater(int target_x, int target_y, float& new_latitude, float& new_longitude);

using namespace cv;
using namespace std;

#define imSizeRow 2048
#define imSizeCol 2048

#define frame1 304
#define frame2 171

int main() {
	Mat dst = imread("C:/Users/ahmet/Desktop/Matching/2048lik.jpg", IMREAD_GRAYSCALE);
	//Size size(2048, 2048);
	//Mat dst;
	//resize(map, dst, size);
	
	
	Mat real_part_map, im_part_map;
	sobelCalc(dst, real_part_map, im_part_map, false);
	Mat dftMap = dft_img(real_part_map, im_part_map, imSizeRow, imSizeCol, false);
	
	Mat realMap(imSizeRow, imSizeCol, CV_64FC1);
	Mat imMap(imSizeRow, imSizeCol, CV_64FC1);
	Mat planesMap[] = { Mat_<float>(realMap), Mat_<float>(imMap) };
	split(dftMap, planesMap);
	
	
    VideoCapture cap("C:/Users/ahmet/Desktop/Matching/googleStudio5fps.mp4");
    if (!cap.isOpened())
        std::cout << "Video is not opened!" << std::endl;	
	
	// defining the matrices for frame operations
	Mat real_part_frame, im_part_frame;
	Mat realFrame(imSizeRow, imSizeCol, CV_64FC1);
	Mat imFrame(imSizeRow, imSizeCol, CV_64FC1);
	Mat dftFrame, idftResult;
	Mat mulReal, mulIm;
	
	//defining the values for maxMinLoc
	double minVal;
	double maxVal;
	Point minLoc;
	Point maxLoc;
	
	
	// Videodan frame çekme iþlemleri baþlýyor
	Mat frame, frameGray;
	cap >> frame;
	while (!frame.empty()) {
		auto start = chrono::high_resolution_clock::now();
		cvtColor(frame, frameGray, COLOR_BGR2GRAY);

		Size frameScale(frame1, frame2);
		Mat frameScl;
		resize(frameGray, frameScl, frameScale);

		imshow("Frame taken from video", frameScl);
		waitKey(1);


		sobelCalc(frameScl, real_part_frame, im_part_frame, true);
		dftFrame = dft_img(real_part_frame, im_part_frame, imSizeRow, imSizeCol, true);

		Mat planesFrame[] = { Mat_<float>(realFrame), Mat_<float>(imFrame) };
		split(dftFrame, planesFrame);

		mulReal = planesMap[0].mul(planesFrame[0]) - planesMap[1].mul(planesFrame[1]);
		mulIm = planesMap[0].mul(planesFrame[1]) + planesFrame[0].mul(planesMap[1]);

		idftResult = idft_img(mulReal, mulIm);

		minMaxLoc(idftResult, &minVal, &maxVal, &minLoc, &maxLoc);
		
		float lat, longitude;
		coordinate_calculater(maxLoc.x - frame1 / 2, maxLoc.y - frame2 / 2, lat, longitude);

		cout << "latitude: " << lat << " longitude: " << longitude << endl;
		cout << "loc: " << maxLoc << "" << endl;
		auto stop = chrono::high_resolution_clock::now();
		auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
		cout << "duration: "  << duration.count() << endl;
		
		Mat found = dst;
		Mat dstc;
		cvtColor(found, dstc, COLOR_GRAY2BGR);
		Point other(maxLoc.x-frame1, maxLoc.y-frame2);
		cv::rectangle(dstc, other, maxLoc, cv::Scalar(0, 255, 0), 4);
		
		
		Size size(1024, 1024);
		Mat foundScl;
		resize(dstc, foundScl, size);
		imshow("Image Location", foundScl(Range(0, 800), Range(0, 500)));
		waitKey(1);
		
		
		cap >> frame;
	}
	
	cap.release();
	destroyAllWindows();
	
    return 0;
}


void coordinate_calculater(int target_x, int target_y, float& new_latitude, float& new_longitude) {
	float dx = (target_x -207) / 500.0;
	float dy = (target_y - 1170) / 500.0;

	new_latitude = 39.8668174 - (dy / 111);
	new_longitude = 32.7486015 + (dx / 111) / cos(39.8668174 * CV_PI / 180);
}