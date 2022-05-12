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
void imReg(Mat* planesMap, Mat* frameGray, int imSizeRow, int imSizeCol, int frame1, int frame2, float& lat, float& longitude, Point& maxLoc, float rotation)
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

	if (rotation != 0) {
		
		sobelCalcRotate(*frameGray, frame1, frame2, real_part_frame, im_part_frame, true, rotation);
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
	
	else {

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
	
}


Mat dft_img(Mat& real_part, Mat& im_part, int m, int n, bool logFlag = false) {
	Mat padded;
	Mat padded2;
	
	copyMakeBorder(real_part, padded, 0, m - real_part.rows, 0, n - real_part.cols, BORDER_CONSTANT, Scalar::all(0));
	copyMakeBorder(im_part, padded2, 0, m - im_part.rows, 0, n - im_part.cols, BORDER_CONSTANT, Scalar::all(0));
	//copyMakeBorder(real_part, padded, (m - real_part.rows) / 2, (m - real_part.rows)/2, (n - real_part.cols) / 2, (n - real_part.cols)/2, BORDER_CONSTANT, Scalar::all(0));
	//copyMakeBorder(im_part, padded2, (m - im_part.rows) / 2, (m - im_part.rows)/2, (n - im_part.cols) / 2, (n - im_part.cols) / 2, BORDER_CONSTANT, Scalar::all(0));
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
	cv::Sobel(img, image_X, CV_64FC1, 1, 0, 5);
	cv::Mat image_Y;
	cv::Sobel(img, image_Y, CV_64FC1, 0, 1, 5);

	image_X.convertTo(image_X, CV_64FC1, 1.0 / 255.0);
	image_Y.convertTo(image_Y, CV_64FC1, 1.0 / 255.0);

	cv::Mat orientation;
	cv::phase(image_X, image_Y, orientation, true);
	cv::subtract(orientation, 2 * CV_PI, orientation, (orientation > CV_PI));

	//normalize(orientation, orientation, 0, 1, NORM_MINMAX);

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
	cout << "burdayým" << endl;
	
}


void sobelCalcRotate(Mat& img, int frame1, int frame2, Mat& real_part, Mat& im_part, bool frameFlag, float rotation) {

	double minVal;
	double maxVal;
	Point minLoc;
	Point maxLoc;


	cv::Mat image_X;
	cv::Sobel(img, image_X, CV_64FC1, 1, 0, 5);
	cv::Mat image_Y;
	cv::Sobel(img, image_Y, CV_64FC1, 0, 1, 5);

	image_X.convertTo(image_X, CV_64FC1, 1.0 / 255.0);
	image_Y.convertTo(image_Y, CV_64FC1, 1.0 / 255.0);

	cv::Mat orientation;
	cv::phase(image_X, image_Y, orientation, true);

	float rotation_rad = rotation * CV_PI / 180;
	
	cv::subtract(orientation, 2 * CV_PI, orientation, (orientation > CV_PI));
	
	cv::subtract(orientation, -rotation_rad, orientation);

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

	Mat real_a, im_a;

	real_a = Magn.mul(cos_fin);
	im_a = Magn.mul(sin_fin);

	if (frameFlag) {

		flip(real_a, real_a, 0);
		flip(real_a, real_a, 1);
		flip(im_a, im_a, 0);
		flip(im_a, im_a, 1);

		im_a = -1 * im_a;

	}

	Size frameScale(frame1, frame2);
	Mat real_partScl, im_partScl;
	resize(real_a, real_partScl, frameScale);
	resize(im_a, im_partScl, frameScale);
	


	int Height = im_partScl.rows / 2;//getting middle point of rows//
	int Width = im_partScl.cols / 2;//getting middle point of height//

	Mat im_partSquare = im_partScl(Range(0, im_partScl.rows), Range(Width - Height, Width + Height)).clone();//getting the middle part of the frame//
	Mat real_partSquare = real_partScl(Range(0, real_partScl.rows), Range(Width - Height, Width + Height)).clone();//getting the middle part of the frame//

	Mat rotMat = getRotationMatrix2D(Point(Height, Height), rotation, 1);
	warpAffine(im_partSquare, im_partSquare, rotMat, im_partSquare.size());

	rotMat = getRotationMatrix2D(Point(Height, Height), rotation, 1);
	warpAffine(real_partSquare, real_partSquare, rotMat, real_partSquare.size());
	


	
	// REAL PART ROTATION SONRASI KIRPIÞ
	Mat real_partCopy = real_partSquare.clone();
	flip(real_partCopy, real_partCopy, 1);

	Mat diag_real = real_partSquare.diag(0);
	Mat nonZero_real;
	findNonZero(diag_real, nonZero_real);
	//cout << "diag: " << diag << endl;
	cout << endl;


	Mat a_real, b_real;
	Mat nonZeroSplit_real[] = { Mat_<float>(a_real), Mat_<float>(b_real) };

	split(nonZero_real, nonZeroSplit_real);
	//cout << "\nnonZero: " << nonZeroSplit[1] << endl;

	minMaxLoc(nonZeroSplit_real[1], &minVal, &maxVal, &minLoc, &maxLoc);


	Mat diag2_real = real_partCopy.diag(0);
	Mat nonZero2_real;
	findNonZero(diag2_real, nonZero2_real);
	//cout << "diag: " << diag2 << endl;
	cout << endl;

	double minVal2;
	double maxVal2;
	Point minLoc2;
	Point maxLoc2;
	Mat a2_real, b2_real;
	Mat nonZeroSplit2_real[] = { Mat_<float>(a2_real), Mat_<float>(b2_real) };

	split(nonZero2_real, nonZeroSplit2_real);
	//cout << "\nnonZero: " << nonZeroSplit2[1] << endl;

	minMaxLoc(nonZeroSplit2_real[1], &minVal2, &maxVal2, &minLoc2, &maxLoc2);


	int maxMin = (int)minVal;
	int minMax = (int)maxVal;
	if (maxMin < minVal2)
		maxMin = minVal2;

	if (minMax > maxVal2)
		minMax = maxVal2;

	int size = (int)(minMax - maxMin);
	cv::Rect crop_region((int)maxMin, (int)maxMin, size, size);

	Mat cropped_real = real_partSquare(crop_region);


	// BURDAN SONRA IM PARTI YAP KNK

	Mat im_partCopy = im_partSquare.clone();
	flip(im_partCopy, im_partCopy, 1);

	Mat diag_im = im_partSquare.diag(0);
	Mat nonZero_im;
	findNonZero(diag_im, nonZero_im);
	//cout << "diag: " << diag << endl;
	cout << endl;


	Mat a_im, b_im;
	Mat nonZeroSplit_im[] = { Mat_<float>(a_im), Mat_<float>(b_im) };

	split(nonZero_im, nonZeroSplit_im);
	//cout << "\nnonZero: " << nonZeroSplit[1] << endl;

	minMaxLoc(nonZeroSplit_im[1], &minVal, &maxVal, &minLoc, &maxLoc);


	Mat diag2_im = im_partCopy.diag(0);
	Mat nonZero2_im;
	findNonZero(diag2_im, nonZero2_im);
	//cout << "diag: " << diag2 << endl;
	cout << endl;

	Mat a2, b2;
	Mat nonZeroSplit2[] = { Mat_<float>(a2), Mat_<float>(b2) };

	split(nonZero2_im, nonZeroSplit2);
	//cout << "\nnonZero: " << nonZeroSplit2[1] << endl;

	minMaxLoc(nonZeroSplit2[1], &minVal2, &maxVal2, &minLoc2, &maxLoc2);


	maxMin = (int)minVal;
	minMax = (int)maxVal;
	if (maxMin < minVal2)
		maxMin = minVal2;

	if (minMax > maxVal2)
		minMax = maxVal2;

	size = (int)(minMax - maxMin);
	cv::Rect crop_region2((int)maxMin, (int)maxMin, size, size);

	Mat cropped_im = im_partSquare(crop_region2);

	real_part = cropped_real.clone();
	im_part = cropped_im.clone();
	
}
/*

void imRegGPU(Mat* planesMap, Mat* frameGpu, int imSizeRow, int imSizeCol, int frame1, int frame2, float& lat, float& longitude, Point& maxLoc)
{
	// defining the matrices for frame operations
	cv::cuda::GpuMat real_part_frame, im_part_frame;
	cv::cuda::GpuMat realFrame(imSizeRow, imSizeCol, CV_64FC1);
	cv::cuda::GpuMat imFrame(imSizeRow, imSizeCol, CV_64FC1);
	cv::cuda::GpuMat dftFrame, idftResult;
	cv::cuda::GpuMat mulReal, mulIm;

	//defining the values for maxMinLoc
	double minVal;
	double maxVal;
	Point minLoc;

	Size frameScale(frame1, frame2);
	cv::cuda::GpuMat frameScl;
	cv::cuda::resize(*frameGpu, frameScl, frameScale);

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

*/