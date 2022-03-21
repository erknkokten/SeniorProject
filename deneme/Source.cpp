#include <iostream>
#include <math.h>
#include <complex>      
#include <iostream>
#include <iomanip>
#include <complex>
#include <stdio.h>
#include <vector>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <string>
//#include <windows.h>

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

int main31()
{
	Mat map = imread("C:/Users/ahmet/Desktop/Matching/test_new.jpg", IMREAD_GRAYSCALE);
	Size size(2048, 2048);
	Mat dst;
	resize(map, dst, size);

	cout << "burdayým" << endl;
	Mat dst2 = imread("C:/Users/ahmet/Desktop/Matching/framefortest.jpg", IMREAD_GRAYSCALE); //dst(Range(800, 900), Range(371, 769));


	//imshow("frame original", frame);
	Size size2(365, 205);
	Mat frame;
	resize(dst2, frame, size2);


	Mat real_part_map, im_part_map;
	Mat real_part_frame, im_part_frame;

	imshow("bu da test", dst);
	
	sobelCalc(dst, real_part_map, im_part_map, false);
	sobelCalc(frame, real_part_frame, im_part_frame, true);


	int imSize = 4096;

	Mat dftMap = dft_img(real_part_map, im_part_map, imSize, imSize, false);
	Mat dftFrame = dft_img(real_part_frame, im_part_frame, imSize, imSize, true);

	Mat realMap(imSize, imSize, CV_64FC1);
	Mat imMap(imSize, imSize, CV_64FC1);
	Mat planesMap[] = { Mat_<float>(realMap), Mat_<float>(imMap) };
	split(dftMap, planesMap);


	Mat realFrame(imSize, imSize, CV_64FC1);
	Mat imFrame(imSize, imSize, CV_64FC1);
	Mat planesFrame[] = { Mat_<float>(realFrame), Mat_<float>(imFrame) };
	split(dftFrame, planesFrame);

	Mat mulReal = planesMap[0].mul(planesFrame[0]) - planesMap[1].mul(planesFrame[1]);
	Mat mulIm = planesMap[0].mul(planesFrame[1]) + planesFrame[0].mul(planesMap[1]);


	Mat idftResult = idft_img(mulReal, mulIm);

	double minVal;
	double maxVal;
	Point minLoc;
	Point maxLoc;

	minMaxLoc(idftResult, &minVal, &maxVal, &minLoc, &maxLoc);

	cout << "loc: " << maxLoc << endl;

	waitKey();
	return 0;
}
