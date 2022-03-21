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
