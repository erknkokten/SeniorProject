// opencv-test.cpp : Bu dosya 'main' iþlevi içeriyor. Program yürütme orada baþlayýp biter.
//

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>
#include <cmath>
#include <windows.h>
//#include <graphics.h>
#include <stdio.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include "imageReg.h"
#include "visualOdometry.h"
#include "ins.h"
using namespace std::chrono;


#define SCALE_VALUE 500.0
using namespace cv;
using namespace std;

# define M_PI 3.141592653589793238462643383279502884
#define earthRadiusKm 6371.0
#define W 2048
#define H 2048


// BURASI ÖNEMLÝ HARD CODED YAPIOZ
//#define croppedImSize 512

#define rowSize 180
#define colSize 322

//#define rowSize 124
//#define colSize 221

#define frame1 colSize
#define frame2 rowSize



float reflon = 0.0;
float reflat = 0.0;

// This function converts decimal degrees to radians
double deg2rad(double deg) {
	return (deg * M_PI / 180);
};

//  This function converts radians to decimal degrees
double rad2deg(double rad) {
	return (rad * 180 / M_PI);
};


void MyLine(Mat img, Point start, Point end, int x, int y, int z)
{
	int thickness = 1;
	int lineType = LINE_AA;
	line(img,
		start,
		end,
		Scalar(255, 255, 255),
		thickness,
		lineType);
}


double distanceEarth(double lat1d, double lon1d, double lat2d, double lon2d) {
	double lat1r, lon1r, lat2r, lon2r, u, v;
	lat1r = deg2rad(lat1d);
	lon1r = deg2rad(lon1d);
	lat2r = deg2rad(lat2d);
	lon2r = deg2rad(lon2d);
	u = sin((lat2r - lat1r) / 2);
	v = sin((lon2r - lon1r) / 2);
	return 2.0 * earthRadiusKm * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));
}



void coordinate_calculater(int pointx, int pointy, int target_x, int target_y, float& new_latitude, float& new_longitude, double angle) {

	float new_target_x = target_x * cos(-angle) - target_y * sin(-angle);
	float new_target_y = target_x * sin(-angle) + target_y * cos(-angle);

	float new_pointx = pointx * cos(-angle) - pointy * sin(-angle);
	float new_pointy = pointx * sin(-angle) + pointy * cos(-angle);



	float dx = (new_target_x - new_pointx) / 500.0;
	float dy = (new_target_y - new_pointy) / 500.0;

	new_latitude = 39.8668174 - (dy / 111);
	new_longitude = 32.7486015 + (dx / 111) / cos(39.8668174 * CV_PI / 180);
}


int main() {
	// INS variables
	float roll, pitch, yaw, altitude, latGT, longGT, timeStamp;
	
	// For initializing the INS system
	//insCall(roll, pitch, yaw, altitude, latGT, longGT, timeStamp);
	
	
	double speedX, speedY;

	// Döndürmeli imreg için deðiþkenler dft alýnacak size ve haritadan croplanacak küçük alan için
	int dftSize = 512;
	int croppedImSize = 256;

	int previousPixelRow = 1300;
	int previousPixelCol = 250;
	int rightCut = (croppedImSize / 2) - (colSize / 2);
	int leftCut = (croppedImSize / 2) + (colSize / 2);
	int topCut = (croppedImSize / 2) + (rowSize / 2);
	int bottomCut = (croppedImSize / 2) - (rowSize / 2);


	// Düz haritada bastýrmak için haritayý okuma
	Mat dst = imread("C:/Users/ahmet/Desktop/Matching/2048lik.jpg");

	// BURALAR ARALIN KODU
	float Xmax;
	float Xmin;

	float Ymax;
	float Ymin;

	char rook_window[] = "Drawing 2: Rook";
	Mat rook_image = Mat::zeros(H, W, CV_8UC1);

	ifstream ifs("C:/Users/ahmet/Desktop/Matching/temp.txt");
	//Two ways:

	//Assign it at initialization
	string content((istreambuf_iterator<char>(ifs)),
		(istreambuf_iterator<char>()));

	int count = 0;
	int count_eleman_size = 0;



	//Pointleri ikili float olarak kaydedip her bir way warasýný da bi vector eleman olarak alýp sonunda vütün wayleri da vector olarak almak istedik 
	vector <vector<vector<float>>> all_points;
	//vector <vector<float>> all_points_x;



	while (content.find("WAY") != -1) {




		int pos_way = content.find("WAY");
		content = content.substr(pos_way + 7);
		vector <float> lat_set;
		vector <float> lon_set;

		vector <float> max_x;
		vector <float> min_x;

		vector <float> max_y;
		vector <float> min_y;
		while (content.find("POINT") < content.find("WAY")) {


			int pos = content.find("POINT");
			content = content.substr(pos + 6);
			int pos2 = content.find(",");
			string lat = content.substr(0, pos2);
			float lat1 = stof(lat);
			content = content.substr(pos2 + 1);
			int pos3 = content.find(",");
			string lon = content.substr(0, pos3);
			float lon1 = stof(lon);
			lon_set.push_back(lon1);
			lat_set.push_back(lat1);


		}
		if (count == 0) {
			reflon = lon_set[0];
			reflat = lat_set[0];
		}


		vector <vector<float>> dist_list;
		vector<float> temp_points;
		//vector <float> dist_y_list;

		if (count < 1) {
			temp_points.push_back(0);
			temp_points.push_back(0);
			dist_list.push_back(temp_points);
		}

		for (int i = 0; i < lat_set.size(); i++) {
			vector<float> temp_points;
			float dist_x = distanceEarth(reflat, reflon, reflat, lon_set[i]);
			if ((reflon > lon_set[i])) {
				dist_x = -dist_x;
			}
			float dist_y = (reflat - lat_set[i]) * 111;
			temp_points.push_back(dist_x);
			temp_points.push_back(dist_y);
			dist_list.push_back(temp_points);
		}

		all_points.push_back(dist_list);

		count_eleman_size += dist_list.size();

		count = count + 1;

	}


	//hepsine mat a koy



	Mat pointx_mat(count_eleman_size, 1, CV_32FC1);
	Mat pointy_mat(count_eleman_size, 1, CV_32FC1);
	int matcnt = 0;

	for (int i = 0; i < all_points.size(); i++) {
		for (int i2 = 0; i2 < all_points[i].size(); i2++) {

			pointx_mat.at<float>(matcnt, 0) = all_points[i][i2][0];
			pointy_mat.at<float>(matcnt, 0) = all_points[i][i2][1];
			matcnt += 1;
		}
	}
	// ARALIN KODU BÝTTÝ


	// dönemelison.mp4 -> drivedaki Adsýz5fps.mp4
	VideoCapture cap("C:/Users/ahmet/Desktop/Matching/donmelison.mp4");
	if (!cap.isOpened())
		std::cout << "Video is not opened!" << std::endl;

	// Kalman filter için deðiþkenler
	int pixel1 = 0;
	int pixel2 = 0;
	pixel_calculater(32.7486015, 39.8668174, pixel1, pixel2);

	Mat X_0 = (Mat_<float>(4, 1) << pixel1 / 500, pixel2 / 500, 4.846 / 2, 8.747 / 2);
	Mat P_0 = Mat::eye(4, 4, CV_32F) * 200;
	float delta_t = 0.2;
	Mat Q = (Mat_<float>(4, 4) <<
		pow(delta_t, 4) / 4, 0, pow(delta_t, 3) / 2, 0,
		0, pow(delta_t, 4) / 4, 0, pow(delta_t, 3) / 2,
		pow(delta_t, 3) / 2, 0, pow(delta_t, 2), 0,
		0, pow(delta_t, 3) / 2, 0, pow(delta_t, 2));


	// Rotation için deðiþkenler
	float rotation = 0;
	int countFrame = 1;
	bool rotFlag = true;

	// Last true location for prohibiting image registration jumps
	Point lastTrueLoc;
	bool init = false;

	// Videodan frame çekme iþlemleri baþlýyor
	Mat frame, frameGray, framePrior, framePriorGray;
	cap >> framePrior;
	cv::cvtColor(framePrior, framePriorGray, COLOR_BGR2GRAY);
	copyMakeBorder(framePriorGray, framePriorGray, (2048 - framePriorGray.rows) / 2, (2048 - framePriorGray.rows) / 2, (2048 - framePriorGray.cols) / 2, (2048 - framePriorGray.cols) / 2, BORDER_CONSTANT, Scalar::all(0));


	int Height = framePriorGray.rows / 2;   //getting middle point of rows//
	int Width = framePriorGray.cols / 2;    //getting middle point of height//


	// rotating the first frame
	int Rotation1 = 0;
	Mat for_Rotation = getRotationMatrix2D(Point(Width, Height), (Rotation1), 1);   //affine transformation matrix for 2D rotation//
	Mat for_Rotated;//declaring a matrix for rotated image
	warpAffine(framePriorGray, framePriorGray, for_Rotation, framePriorGray.size());    //applying affine transformation//

	
	cap >> frame;

	float rotationOld = 0;
	while (!frame.empty()) {
		
		// Calling the INS 10 times and getting the average
		/*
		roll = 0, pitch = 0, yaw = 0, altitude=0, latGT=0, longGT=0;

		for (int i = 0; i < 10; i++) {
			float rollTemp, pitchTemp, yawTemp, altitudeTemp, latGTTemp, longGTTemp;
			insCall(rollTemp, pitchTemp, yawTemp, altitudeTemp, latGTTemp, longGTTemp, timeStamp);
			roll += rollTemp;
			pitch += pitchTemp;
			yaw += yawTemp;
			altitude += altitudeTemp;
			latGT += latGTTemp;
			longGT += longGTTemp;
		}
		roll = roll / 10;
		pitch = pitch / 10;
		yaw = yaw / 10;
		altitude = altitude / 10;
		latGT = latGT / 10;
		longGT = longGT / 10;
		*/
		
		// Eðer yaw batýyý gösteriyosa, bu dereceye göre döndürme yap
		float insRotation = 90 - yaw;
		// Eðer kuzeyi gösteriyosa
		insRotation = -yaw;

		auto start = chrono::high_resolution_clock::now();
		cv::cvtColor(frame, frameGray, COLOR_BGR2GRAY);

		// Clearing the rook image
		rook_image = Mat::zeros(H, W, CV_8UC1);


		// Rook image'ý dönürme iþlemleri (ARAL ÝYÝ BÝLÝR)
		// TODO: INS'ten direkt radyan olarak alýnabilir
		float angle = deg2rad(rotation);
		Mat rotated_mat_x = cos(angle) * pointx_mat - sin(angle) * pointy_mat;
		Mat rotated_mat_y = sin(angle) * pointx_mat + cos(angle) * pointy_mat;


		double min_val_matx;
		double max_val_matx;

		Point min_matlocx;
		Point max_matlocx;


		double min_val_maty;
		double max_val_maty;

		Point min_matlocy;
		Point max_matlocy;


		minMaxLoc(rotated_mat_x, &min_val_matx, &max_val_matx, &min_matlocx, &max_matlocx);
		minMaxLoc(rotated_mat_y, &min_val_maty, &max_val_maty, &min_matlocy, &max_matlocy);

		rotated_mat_x = SCALE_VALUE * (rotated_mat_x + abs(min_val_matx));

		rotated_mat_y = SCALE_VALUE * (rotated_mat_y + abs(min_val_maty));



		int mtcnt2 = 0;
		for (vector<vector<float>> i : all_points) {

			for (int j = 0; j < i.size() - 1; j++) {

				MyLine(rook_image, Point(rotated_mat_x.at<float>(mtcnt2, 0), rotated_mat_y.at<float>(mtcnt2, 0)), Point(rotated_mat_x.at<float>(mtcnt2 + 1, 0), rotated_mat_y.at<float>(mtcnt2 + 1, 0)), 0, 100, 220);
				mtcnt2 += 1;

			}
			mtcnt2 += 1;
		}

		// Pixel locations of reference point
		int refPixelRow = rotated_mat_x.at<float>(0, 0);
		int refPixelCol = rotated_mat_y.at<float>(0, 0);

		// BURDAN ÝTÝBAREN ELÝMÝZDE DÖNMÜÞ ROOK_ÝMAGE VAR BUNU CROPLAYIP MATCH ETCEZ

		// Croppin parts in order to stay þin the rook image, 512x512 square is cutted depending on the previously known pixel location
		int up = ((previousPixelRow - topCut) > 0) ? (previousPixelRow - topCut) : 0;
		int bottom = (previousPixelRow + bottomCut) < H - 1 ? (previousPixelRow + bottomCut) : H - 1;
		int left = (previousPixelCol - leftCut) > 0 ? previousPixelCol - leftCut : 0;
		int right = (previousPixelCol + rightCut) < W - 1 ? previousPixelCol + rightCut : W - 1;

		cout << "up: " << up << " bottom: " << bottom << " left: " << left << " right: " << right << endl;

		// Croplanmýþ rook_image'ý bastýrmak
		Mat rook_cropped = rook_image(Range(up, bottom), Range(left, right));
		imshow("rook_cropped", rook_cropped);
		waitKey(1);

		// croppedImSize = 512, 512x512 kestiðimiz için dftyi 512lik almasý gerek
		Mat real_part_map, im_part_map;
		sobelCalc(rook_cropped, real_part_map, im_part_map, false);
		Mat dftMap = dft_img(real_part_map, im_part_map, dftSize, dftSize, false);

		// Ýmage reg için gerekli matrislerin hazýrlanmasý
		Mat realMap(dftSize, dftSize, CV_64FC1);
		Mat imMap(dftSize, dftSize, CV_64FC1);
		Mat planesMap[] = { Mat_<float>(realMap), Mat_<float>(imMap) };
		split(dftMap, planesMap);


		float lat, longitude;
		Point maxLoc;
		// Image Registration part
		imReg(planesMap, &frameGray, dftSize, dftSize, colSize, rowSize, lat, longitude, maxLoc, 0);
		// Passing to the real pixel locations from the cropped square pixel locations
		maxLoc.x = maxLoc.x + left;
		maxLoc.y = maxLoc.y + up;

		// bi sonraki iteration için eski deðerleri kaydetmek
		previousPixelRow = maxLoc.y;
		previousPixelCol = maxLoc.x;

		// atlama engeli için maxLoc u imRegouta kaydettik
		Point imRegOut = maxLoc;

		// eþlenen frame'in orta noktasýný bulmak
		Point imRegOutCenter(imRegOut.x - 161, imRegOut.y-90);
		
		//offset çýkar sonra döndür
		float offsetx = maxLoc.x - refPixelRow-(colSize/2);
		float offsety = maxLoc.y - refPixelCol-(rowSize/2);
		
		//offsetx ve offsety deðerleri geri döndür
		float offsetx_rotateback = offsetx*cos(-angle) - offsety* sin(-angle);
		float offsety_rotateback = offsetx*sin(-angle) + offsety*cos(-angle);
		
		// döndürülmemiþ offseti ekle
		int offsetx_new = offsetx_rotateback + 207;
		int offsety_new = offsety_rotateback +1170;
		
		// Showing the frame
		Size frameScale(frame1, frame2);
		Mat frameScl;
		resize(frameGray, frameScl, frameScale);

		imshow("Frame taken from video", frameScl);
		waitKey(1);
		


		// Visual Odometry part
		visOdo(&framePriorGray, &frameGray, speedX, speedY, rotation);

		// visodo speed iyi sonuç vermediði için elle speed besleme
		speedX = 2.5 / 5;
		speedY = -4.375 / 5;


		// Eðer zýplama olursa önceki lokasyona odometriden gelen en son hýzlarý ekle

		/*
		if (distance(lastTrueLoc.x, lastTrueLoc.y, maxLoc.x, maxLoc.y) > 10 && init) {
			imRegOut.x = lastTrueLoc.x;
			imRegOut.y = lastTrueLoc.y;
		}
		*/



		// Pixel locationlarý dönmüþ halden nortun yukarýyý gösterdiði hale çevirme
		//float new_target_x = (imRegOut.x - 161) * cos(-angle) - (imRegOut.y - 90) * sin(-angle);
		//float new_target_y = (imRegOut.x - 161) * sin(-angle) + (imRegOut.y - 90) * cos(-angle);

		Mat Z = (Mat_<float>(4, 1) << offsetx_new, offsety_new, speedX, speedY);
		int a = 0;

		Kalman(Z, X_0, P_0, delta_t, Q);
		//cout << "X_0 = " << endl << " " << X_0 << endl << endl;
		//cout << "Pixel Loc: " << maxLoc << "" << endl;


		//cout << "SpeedX: " << speedX << ", SpeedY: " << speedY << "\n" << "Overall speed: " << sqrt(pow(speedX, 2) + pow(speedY, 2)) << endl;

		/*cout << "SpeedX: " << speedX << ", SpeedY: " << speedY << "\n" << endl;
		cout << "Latitude: " w
		
		<< lat << ", Longitude: " << longitude << endl;
		cout << "Pixel Loc: " << maxLoc << "" << endl;*/


		Mat dstc = dst.clone();

		Point kalmanEstimationPoint(X_0.at<float>(0, 0), X_0.at<float>(1, 0));
		Point imRegEstimationPoint(offsetx_new, offsety_new);

		//lastTrueLoc.x = kalmanEstimationPoint.x * cos(angle) - kalmanEstimationPoint.y * sin(angle);
		//lastTrueLoc.y = kalmanEstimationPoint.x * sin(angle) + kalmanEstimationPoint.y * cos(angle);

		cout << "Kalman Estimation Point: " << kalmanEstimationPoint << endl;
		cout << "ImReg Estimation Point: " << imRegEstimationPoint << endl;

		circle(dstc, kalmanEstimationPoint, 7, Scalar(0, 255, 0), 7);
		circle(dstc, imRegEstimationPoint, 7, Scalar(0, 0, 255), 7);

		Size size(1024, 1024);
		Mat foundScl;
		cv::resize(dstc, foundScl, size);
		cv::imshow("Image Location", foundScl(Range(0, 800), Range(0, 500)));
		cv::waitKey(1);


		// BURANIN ALTI BÝ SONRAKÝ COMMENTA KADAR DEBUG ICIN
		Mat found = rook_image.clone();
		Mat dstcx;

		cv::cvtColor(found, dstcx, COLOR_GRAY2BGR);
		//Point rook_center(maxLoc.x - 160, maxLoc.y - 89);
		//Point rook_center2(maxLoc.x - 162, maxLoc.y - 91);

		Point imRegx(maxLoc.x - colSize, maxLoc.y - rowSize);
		cv::rectangle(dstcx, maxLoc, imRegx, cv::Scalar(0, 0, 255), 4);
		//cv::rectangle(dstcx, rook_center, rook_center2, cv::Scalar(0, 255, 0), 4);
		Size size31(1024, 1024);
		Mat foundScl31;
		cv::resize(dstcx, foundScl31, size31);
		cv::imshow("Image LocationXD", foundScl31(Range(0, 1024), Range(0, 1024)));
		cv::waitKey(0);
		// DEBUG SONU


		cout << "\n--------------------------------------\n" << endl;
		frameGray.copyTo(framePriorGray);
		cap >> frame;

		if ((rotation > -360) && rotFlag) {
			rotation = rotation - 1;
		}
		else if (rotation == -360) {
			rotFlag = false;
			rotation = 0;
		}

		rotationOld = rotation;
		//cout << "count: " << count << endl;
		countFrame++;
		init = true;

	}

	cap.release();
	cv::destroyAllWindows();

	return 0;
}



