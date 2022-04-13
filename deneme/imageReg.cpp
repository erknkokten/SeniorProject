#include "imageReg.h"

using namespace cv;
using namespace std;


void coordinate_calculater(int target_x, int target_y, float& new_latitude, float& new_longitude) {
	float dx = (target_x - 207) / 500.0;
	float dy = (target_y - 1170) / 500.0;

	new_latitude = 39.8668174 - (dy / 111);
	new_longitude = 32.7486015 + (dx / 111) / cos(39.8668174 * CV_PI / 180);
}

void pixel_calculater(float target_lattitude, float target_longitude, int& new_x, int& new_y) {
	int x = (int)((target_longitude - 32.7486015) * 111 * cos(39.8668174 * CV_PI / 180));
	int y = (int)((39.8668174 - target_lattitude) * 111);
	new_x = 500*x + 207;
	new_y = 500*y + 1170;
}


/*
	map is grayscale and have size (imSizeRow,imSizeCol)
	frame is grayscale
	frame1 frame width
	frame2 frame height
*/
void imReg(Mat* planesMap, Mat* frameGray, int imSizeRow, int imSizeCol, int frame1, int frame2, float& lat, float& longitude, Point& maxLoc)
{
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

	Size frameScale(frame1, frame2);
	Mat frameScl;
	resize(*frameGray, frameScl, frameScale);

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

	coordinate_calculater(maxLoc.x - frame1 / 2, maxLoc.y - frame2 / 2, lat, longitude);

	return;
}


Mat dft_img(Mat& real_part, Mat& im_part, int m, int n, bool logFlag = false) {
	Mat padded;
	Mat padded2;
	copyMakeBorder(real_part, padded, 0, m - real_part.rows, 0, n - real_part.cols, BORDER_CONSTANT, Scalar::all(0));
	copyMakeBorder(im_part, padded2, 0, m - im_part.rows, 0, n - im_part.cols, BORDER_CONSTANT, Scalar::all(0));
	Mat planes[] = { Mat_<float>(padded), Mat_<float>(padded2) };
	Mat complexI;
	
	merge(planes, 2, complexI);
	dft(complexI, complexI);
	return complexI;

}

Mat idft_img(Mat& real_part, Mat& im_part) {

	Mat planes[] = { Mat_<float>(real_part), Mat_<float>(im_part) };
	Mat complexI;
	merge(planes, 2, complexI);

	cv::Mat inverseTransform;
	cv::dft(complexI, inverseTransform, cv::DFT_INVERSE | DFT_REAL_OUTPUT);

	return inverseTransform;
}


void sobelCalc(Mat& img, Mat& real_part, Mat& im_part, bool frameFlag) {

	cv::Mat image_X;
	cv::Sobel(img, image_X, CV_64FC1, 1, 0, 7);
	cv::Mat image_Y;
	cv::Sobel(img, image_Y, CV_64FC1, 0, 1, 7);

	image_X.convertTo(image_X, CV_64FC1, 1.0 / 255.0);
	image_Y.convertTo(image_Y, CV_64FC1, 1.0 / 255.0);

	cv::Mat orientation;
	cv::phase(image_X, image_Y, orientation, true);
	cv::subtract(orientation, 2 * CV_PI, orientation, (orientation > CV_PI));

	normalize(orientation, orientation, 0, 1, NORM_MINMAX);

	cv::Mat cos(orientation.rows, orientation.cols, CV_64FC1, CV_PI / 2);
	cv::Mat orientation_two = 2 * orientation;
	cv::Mat sq_oc = orientation_two.mul(orientation_two);
	//busin
	cv::Mat sin_fin = orientation_two - ((sq_oc.mul(orientation_two)) / 6);


	cv::Mat cos_oc = cos - orientation_two;
	cv::Mat cos_sq = cos_oc.mul(cos_oc);

	//bucos
	cv::Mat cos_fin = cos_oc - ((cos_sq.mul(cos_oc)) / 6);


	cv::Mat Magn = image_X + image_Y;

	real_part = Magn.mul(cos_fin);
	im_part = Magn.mul(sin_fin);

	if (frameFlag) {

		flip(real_part, real_part, 0);
		flip(real_part, real_part, 1);
		flip(im_part, im_part, 0);
		flip(im_part, im_part, 1);

		im_part = -1 * im_part;

	}
}


void imRegGPU(Mat* planesMap, Mat* frameGray, int imSizeRow, int imSizeCol, int frame1, int frame2, float& lat, float& longitude, Point& maxLoc)
{
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

	Size frameScale(frame1, frame2);
	Mat frameScl;
	resize(*frameGray, frameScl, frameScale);

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

	coordinate_calculater(maxLoc.x - frame1 / 2, maxLoc.y - frame2 / 2, lat, longitude);

	return;
}