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
#include "visualOdometry.h"


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


	VideoCapture cap("C:/Users/ahmet/Desktop/Matching/framegg.mp4");
	if (!cap.isOpened())
		std::cout << "Video is not opened!" << std::endl;

	// defining the matrices for frame operations
	Point maxLoc;
	double speedX, speedY;

	// Videodan frame çekme iþlemleri baþlýyor
	Mat frame, frameGray, framePrior, framePriorGray;
	cap >> framePrior;
	cvtColor(framePrior, framePriorGray, COLOR_BGR2GRAY);
	cap >> frame;

	int pixel1 = 0;
	int pixel2 = 0;

	pixel_calculater(32.7486015, 39.8668174, pixel1, pixel2);

	Mat X_0 = (Mat_<float>(4, 1) << pixel1 / 500, pixel2 / 500, 4.846 / 2, 8.747 / 2);
	Mat P_0 = Mat::eye(4, 4, CV_32F) * 200;

	while (!frame.empty()) {
		auto start = chrono::high_resolution_clock::now();
		cvtColor(frame, frameGray, COLOR_BGR2GRAY);

		float lat, longitude;
		// Image Registration part
		imReg(planesMap, &frameGray, imSizeRow, imSizeCol, frame1, frame2, lat, longitude, maxLoc);

		// Visual Odometry part
		visOdo(&framePriorGray, &frameGray, speedX, speedY);
		Mat Z = (Mat_<float>(4, 1) << (float)maxLoc.x, (float)maxLoc.y, speedX, speedY);
		int a = 0;
		Kalman(Z, X_0, P_0, 0.2);
		cout << "X_0 = " << endl << " " << X_0 << endl << endl;
		cout << "SpeedX: " << speedX << "SpeedY: " << speedY << "\n" << endl;
		cout << "Latitude: " << lat << ", Longitude: " << longitude << endl;
		cout << "Pixel Loc: " << maxLoc << "" << endl;
		auto stop = chrono::high_resolution_clock::now();
		auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
		cout << "Duration: " << duration.count() << " ms" << endl;

		Mat found = dst;
		Mat dstc;
		cvtColor(found, dstc, COLOR_GRAY2BGR);
		Point other(maxLoc.x - frame1, maxLoc.y - frame2);
		cv::rectangle(dstc, other, maxLoc, cv::Scalar(0, 255, 0), 4);


		Size size(1024, 1024);
		Mat foundScl;
		resize(dstc, foundScl, size);
		imshow("Image Location", foundScl(Range(0, 800), Range(0, 500)));
		waitKey(1);




		cout << "\n--------------------------------------\n" << endl;
		frameGray.copyTo(framePriorGray);
		cap >> frame;
	}

	cap.release();
	destroyAllWindows();

	return 0;
}