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
using namespace cv;


using namespace std;

# define M_PI 3.141592653589793238462643383279502884
#define earthRadiusKm 6371.0
#define w 1700

void MyLine(Mat img, Point start, Point end);


// This function converts decimal degrees to radians
double deg2rad(double deg) {
    return (deg * M_PI / 180);
};

//  This function converts radians to decimal degrees
double rad2deg(double rad) {
    return (rad * 180 / M_PI);
};

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

void MyLine(Mat img, Point start, Point end, int x, int y, int z)
{
    int thickness = 1;
    int lineType = LINE_4;
    line(img,
        start,
        end,
        Scalar(x, y, z),
        thickness,
        lineType);
}


int main1()
{
    char rook_window[] = "Drawing 2: Rook";
    Mat rook_image = Mat::zeros(w, w, CV_8UC3);
    std::ifstream file("C:/Users/ahmet/Desktop/Matching/TEMP.txt");

    ifstream infile;
    infile.open("C:/Users/ahmet/Desktop/Matching/TEMP.txt");

    ifstream ifs("C:/Users/ahmet/Desktop/Matching/TEMP.txt");
    //Two ways:

    //Assign it at initialization
    string content((istreambuf_iterator<char>(ifs)),
        (istreambuf_iterator<char>()));

    int count = 0;
    float reflon = 0;
    float reflat = 0;
    int pos_way;
    while (1+(pos_way = content.find("WAY"))) {

        
        content = content.substr(pos_way + 7);
        vector <float> lat_set;
        vector <float> lon_set;
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


        for (int i = 0; i < lon_set.size(); i++) {
            std::cout << lon_set[i] << ",  " << lat_set[i] << endl;
        }


        vector <float> dist_x_list;
        vector <float> dist_y_list;

        if (count < 1) {
            dist_x_list.push_back(0);
            dist_y_list.push_back(0);
        }

        for (int i = 0; i < lat_set.size(); i++) {
            float dist_x = distanceEarth(reflat, reflon, reflat, lon_set[i]);
            if ((reflon > lon_set[i])) {
                dist_x = -dist_x;
            }
            float dist_y = (reflat - lat_set[i]) * 111;
            dist_x_list.push_back(dist_x);
            dist_y_list.push_back(dist_y);
        }


        cout << "***********************" << endl;

        for (int i = 0; i < lat_set.size() - 1; i++) {
            MyLine(rook_image, Point(500 * dist_x_list[i] + 400, 500 * dist_y_list[i] + 300), Point(500 * dist_x_list[i + 1] + 400, 500 * dist_y_list[i + 1] + 300), 0, 100, 220);
        }

        count = count + 1;
        cout << "*******************" << endl;
    }

    imwrite("C:/Users/ahmet/Desktop/Matching/test_new.jpg", rook_image);
    cout << count << endl;
    //imshow(rook_window, rook_image);
    //moveWindow(rook_window, w, 200);


    /*
    GaussianBlur(lena, lena, Size(3, 3), 0, 0, BORDER_DEFAULT);


    Mat grad_x;
    Mat grad_y;
    Sobel(lena, grad_x, CV_64F, 1, 0, 1, BORDER_DEFAULT);
    Sobel(lena, grad_y, CV_64F, 0, 1, 1, BORDER_DEFAULT);


    pow(grad_x, 2, grad_x);
    pow(grad_y, 2, grad_y);
    Mat sum = grad_x + grad_y;

    int rows = lena.rows;
    int cols = lena.cols;

    sqrt(sum, sum);

    Mat mag;
    magnitude(grad_x, grad_y, mag);


    //to get column size


    //int rows = rook_image.rows;

    Mat orientation;
    phase(grad_x, grad_y, orientation, true);
    subtract(orientation, 2 * M_PI, orientation, (orientation > M_PI));
    */
    /*

    cv::Mat image_X;
    // this is how we can create a horizontal edge detector
    // Sobel(src_gray, dst, depth, x_order, y_order)
    // src_gray: The input image.
    // dst: The output image.
    // depth: The depth of the output image.
    // x_order: The order of the derivative in x direction.
    // y_order: The order of the derivative in y direction.
    // To calculate the gradient in x direction we use: x_order= 1 and y_order= 0.
    // To calculate the gradient in x direction we use: x_order= 0 and y_order= 1.
    cv::Sobel(lena, image_X, CV_8UC1, 1, 0);
    //cv::imshow("Sobel image", image_X);
    //cv::waitKey();

    cv::Mat image_Y;
    // this is how we can create a vertical edge detector.
    cv::Sobel(lena, image_Y, CV_8UC1, 0, 1);
    //cv::imshow("Sobel image", image_Y);
    //cv::waitKey();

    cv::Mat sobel = image_X + image_Y;
    imwrite("sobel.jpg", sobel);
    cv::imshow("Sobel - L1 norm", sobel);
    cv::waitKey();

    image_X.convertTo(image_X, CV_64F, 1.0 / 255.0);
    image_Y.convertTo(image_Y, CV_64F, 1.0 / 255.0);

    cv::Mat orientation;
    cv::phase(image_X, image_Y, orientation, true);
    cv::subtract(orientation, 2 * M_PI, orientation, (orientation > M_PI));
    imwrite("orientation.jpg", orientation);

    cv::imshow("Sobel - L1 norm, Orientation", orientation);
    cv::waitKey();

    double sobmin, sobmax;
    cv::minMaxLoc(sobel, &sobmin, &sobmax);

    cv::Mat sobelImage;
    sobel.convertTo(sobelImage, CV_8UC1, -255. / sobmax, 255);
    //cv::imshow("Edges with a sobel detector", sobelImage);
    //cv::waitKey();


    cv::Mat image_Sobel_thresholded;
    double max_value, min_value;
    cv::minMaxLoc(sobelImage, &min_value, &max_value);
    //image_Laplacian = image_Laplacian / max_value * 255;


    cv::threshold(sobelImage, image_Sobel_thresholded, 20, 255, cv::THRESH_BINARY);
    //cv::imshow("Thresholded Sobel", image_Sobel_thresholded);
    //cv::waitKey();

    /*

    String name = "Magnitude";
    String name2 = "Grad direction";
    String name3 = "Lena";
    namedWindow(name, 1);
    imshow(name, mag);
    //moveWindow(rook_window, w, 200);

    namedWindow(name2, 1);
    imshow(name2, orientation);

    namedWindow(name3, 1);
    imshow(name3, lena);


    waitKey(0);
    */

    return 0;
}