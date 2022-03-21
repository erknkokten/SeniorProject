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


using namespace cv;
using namespace std;

#define imSize 4096

int main() {
	Mat map = imread("C:/Users/ahmet/Desktop/Matching/test_new.jpg", IMREAD_GRAYSCALE);
	Size size(2048, 2048);
	Mat dst;
	resize(map, dst, size);
	
	Mat real_part_map, im_part_map;
	sobelCalc(dst, real_part_map, im_part_map, false);
	Mat dftMap = dft_img(real_part_map, im_part_map, imSize, imSize, false);
	
	Mat realMap(imSize, imSize, CV_64FC1);
	Mat imMap(imSize, imSize, CV_64FC1);
	Mat planesMap[] = { Mat_<float>(realMap), Mat_<float>(imMap) };
	split(dftMap, planesMap);
	
	
    VideoCapture cap("C:/Users/ahmet/Desktop/Matching/googleStudio5fps.mp4");
    if (!cap.isOpened())
        std::cout << "Video is not opened!" << std::endl;	
	
	// defining the matrices for frame operations
	Mat real_part_frame, im_part_frame;
	Mat realFrame(imSize, imSize, CV_64FC1);
	Mat imFrame(imSize, imSize, CV_64FC1);
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
		cvtColor(frame, frameGray, COLOR_BGR2GRAY);
		
		Size frameScale(365, 205);
		Mat frameScl;
		resize(frameGray, frameScl, frameScale);
		
		//imshow("Frame taken from video", frameScl);
		char c = (char)waitKey(1);
		if (c == 27) 
			break;
		
		
		sobelCalc(frameScl, real_part_frame, im_part_frame, true);
		dftFrame = dft_img(real_part_frame, im_part_frame, imSize, imSize, true);
		
		Mat planesFrame[] = { Mat_<float>(realFrame), Mat_<float>(imFrame) };
		split(dftFrame, planesFrame);
		
		mulReal = planesMap[0].mul(planesFrame[0]) - planesMap[1].mul(planesFrame[1]);
		mulIm = planesMap[0].mul(planesFrame[1]) + planesFrame[0].mul(planesMap[1]);

		idftResult = idft_img(mulReal, mulIm);
		
		minMaxLoc(idftResult, &minVal, &maxVal, &minLoc, &maxLoc);
		
		cout << "loc: " << maxLoc << endl;
		
		cap >> frame;
	}
	
	cap.release();
	destroyAllWindows();
	
    return 0;
}


