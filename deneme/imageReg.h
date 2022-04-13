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

using namespace cv;

Mat dft_img(Mat& img_1, Mat& imPart, int m, int n, bool logFlag);
Mat idft_img(Mat& real_part, Mat& im_part);
void sobelCalc(Mat& img, Mat& real_part, Mat& im_part, bool frameFlag);
void coordinate_calculater(int target_x, int target_y, float& new_latitude, float& new_longitude);
void pixel_calculater(float target_lattitude, float target_longitude, int& new_x, int& new_y);
void imReg(Mat* planesMap, Mat* frameGray, int imSizeRow, int imSizeCol, int frame1, int frame2, float& lat, float& longitude, Point& maxLoc);



Mat dft_img2(Mat& input, int m, int n, bool logFlag);
Mat idft_img2(Mat& mat);
void sobelCalc2(Mat& img, Mat& sobel, bool frameFlag);
