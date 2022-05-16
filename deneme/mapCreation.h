#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>
#include <cmath>
#include <windows.h>
#include <stdio.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>

using namespace cv;
using namespace std;

extern float reflon = 0.0;
extern float reflat = 0.0;

double deg2rad(double deg);
double rad2deg(double rad);
void MyLine(Mat img, Point start, Point end, int x, int y, int z);
double distanceEarth(double lat1d, double lon1d, double lat2d, double lon2d);
void coordinate_calculater(Mat pointx, Mat pointy, int target_x, int target_y, float& new_latitude, float& new_longitude, double angle);