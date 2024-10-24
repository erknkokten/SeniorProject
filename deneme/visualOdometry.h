#include <iostream>
#include <math.h>
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
#include <opencv2/highgui.hpp>      //for imshow
#include <vector>
#include <iostream>
#include <iomanip>
#include <opencv2/features2d.hpp>

// #include <opencv2/xfeatures2d.hpp>
using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;
using std::cout;
using std::endl;
using namespace std::chrono;


#define height 200
#define cx 240
#define cy 135
#define sx 0.015 / 480
#define sy 0.012 / 270
#define focallength 0.08189
#define sampleperiod 0.2

void visOdo(Mat* img_1, Mat* img_2, double& speedX, double& speedY, float rotation);
void Kalman(Mat Z, Mat& X_nn_1, Mat& P_nn_1, float delta_t, Mat& Q);