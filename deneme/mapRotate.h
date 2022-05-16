#pragma once

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
using namespace std::chrono;


#define SCALE_VALUE 500.0
using namespace cv;


using namespace std;

# define M_PI 3.141592653589793238462643383279502884
#define earthRadiusKm 6371.0
#define W 2048
#define H 2048

float reflon = 0.0;
float reflat = 0.0;
namespace {
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



    void coordinate_calculater(Mat pointx, Mat pointy, int target_x, int target_y, float& new_latitude, float& new_longitude, double angle) {
        float dx = (target_x - pointx.at<float>(0, 0)) / 500.0;
        float dy = (target_y - pointy.at<float>(0, 0)) / 500.0;


        if (angle != M_PI / 2 && angle != 3 * M_PI / 2) {
            new_latitude = 39.8668174 - (dy / cos(angle) / 111);
            new_longitude = 32.7486015 + (dx / 111) / cos(39.8668174 * CV_PI / 180);
        }
        else {
            new_latitude = 39.8668174 - (dx / cos(angle) / 111);
            new_longitude = 32.7486015 + (dy / 111) / cos(39.8668174 * CV_PI / 180);
        }
    }
}