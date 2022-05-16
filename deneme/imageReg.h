#ifndef IMAGE_REG
#define IMAGE_REG
#include <complex>
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/cuda.hpp>


#define M_PI 3.141592653589793238462643383279502884
#define earthRadiusKm 6371.0
#define W 2048
#define H 2048
#define SCALE_VALUE 500.0

#define rowSize 180
#define colSize 322

using namespace cv;

Mat dft_img(Mat& img_1, Mat& imPart, int m, int n, bool logFlag);
Mat idft_img(Mat& real_part, Mat& im_part);
void sobelCalc(Mat& img, Mat& real_part, Mat& im_part, bool frameFlag);
void imReg(Mat* planesMap, Mat* frameGray, int imSizeRow, int imSizeCol, int frame1, int frame2, float& lat, float& longitude, Point& maxLoc, float rotation);



double deg2rad(double deg);
double rad2deg(double deg);
void MyLine(Mat img, Point start, Point end, int x, int y, int z);
double distanceEarth(double lat1d, double lon1d, double lat2d, double lon2d);
void coordinate_calculater(int pointx, int pointy, int target_x, int target_y, float& new_latitude, float& new_longitude, double angle);
void pixel_calculater(float target_lattitude, float target_longitude, int& new_x, int& new_y);
#endif