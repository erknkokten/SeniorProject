#include "visualOdometry.h"

void visOdo(Mat* img_1, Mat* img_2, double& speedX, double& speedY)
{
    double xc1 = 0;
    double xc2 = 0;
    double yc1 = 0;
    double yc2 = 0;
    double velocity = 0;
    double degrees = 0;
    //double speedX=0, speedY=0;

    double x1index = 0, x2index = 0;
    double y1index = 0, y2index = 0;

    int count = 0;
    double resultX = 0;
    double resultY = 0;
    double result_degrees = 0;
    double direction;

    vector<KeyPoint> img_1_keypoints, img_2_keypoints;
    Mat img_1_descriptors, img_2_descriptors;
    Ptr<ORB> detector = ORB::create();     //Feature points are extracted by ORB algorithm 
    //Detect FAST corner position
    detector->detect(*img_1, img_1_keypoints);
    detector->detect(*img_2, img_2_keypoints);
    //Calculate the BRIEF descriptor according to the corner position
    detector->compute(*img_1, img_1_keypoints, img_1_descriptors);
    detector->compute(*img_2, img_2_keypoints, img_2_descriptors);
    //Hamming distance as similarity measure
    BFMatcher matcher(NORM_HAMMING, true);
    vector<DMatch> matches; //Note that the type here is DMatch



    matcher.match(img_1_descriptors, img_2_descriptors, matches);

    //Optimized by RANSAC algorithm
    //Save matching sequence number  
    vector<int> queryIdxs(matches.size()), trainIdxs(matches.size());
    for (size_t i = 0; i < matches.size(); i++)
    {
        queryIdxs[i] = matches[i].queryIdx;
        trainIdxs[i] = matches[i].trainIdx;
    }

    Mat homography_matrix;   //Transformation matrix  

    vector<Point2f> points1; KeyPoint::convert(img_1_keypoints, points1, queryIdxs);
    vector<Point2f> points2; KeyPoint::convert(img_2_keypoints, points2, trainIdxs);
    int ransacReprojThreshold = 5;  //Reject threshold  


    homography_matrix = findHomography(Mat(points1), Mat(points2), RANSAC, ransacReprojThreshold);
    vector<char> matchesMask(matches.size(), 0);
    Mat points1t;
    perspectiveTransform(Mat(points1), points1t, homography_matrix);
    for (size_t i = 0; i < points1.size(); i++)  //Save 'local point'  
    {
        if (norm(points2[i] - points1t.at<Point2f>((int)i, 0)) <= ransacReprojThreshold) //Mark internal points  
        {
            x1index = points1[i].x;
            x2index = points2[i].x;
            y1index = points1[i].y;
            y2index = points2[i].y;

            double shiftX = x2index - x1index;
            double shiftY = y2index - y1index;

            resultX = resultX + shiftX;
            resultY = resultY + shiftY;

            /*xc1 = (x1index - cx) * height * sx / focallength;
            xc2 = (x2index - cx) * height * sx / focallength;
            yc1 = (y1index - cy) * height * sy / focallength;
            yc2 = (y2index - cy) * height * sy / focallength;*/

            //velocity = pow((pow((xc2 - xc1), 2) + pow((yc2 - yc1), 2)), 0.5) / sampleperiod;
            //result = result + velocity;

            matchesMask[i] = 1;
            count = count + 1;

            /*speedX = speedX + xc2 - xc1;
            speedY = speedY + yc2 - yc1;*/
        }
    }
    speedX = resultX / count;
    speedY = resultY / count;

    /*Mat match_img2;
    cv::drawMatches(*img_1, img_1_keypoints, *img_2, img_2_keypoints, matches, match_img2, Scalar(0, 0, 255), Scalar::all(-1), matchesMask);
    cv::imshow("Optimized matching", match_img2);
    cv:: waitKey(1);*/



}
void Kalman(Mat Z, Mat& X_nn_1, Mat& P_nn_1, float delta_t, Mat& Q) {
    int size_state = 4;
    float sigma_x = 0.5;
    float sigma_y = 0.5;
    float sigma_vx = 0.1;
    float sigma_vy = 0.1;

    Z.at<float>(2, 0) = Z.at<float>(2, 0) / delta_t;
    Z.at<float>(3, 0) = Z.at<float>(3, 0) / delta_t;

    Mat F = Mat::eye(size_state, size_state, CV_32F);
    F.at<float>(0, 2) = delta_t;
    F.at<float>(1, 3) = delta_t;

    Mat R = (Mat_<float>(4, 4) <<
        pow(sigma_x, 2), 0, 0, 0,
        0, pow(sigma_y, 2), 0, 0,
        0, 0, pow(sigma_vx, 2), 0,
        0, 0, 0, pow(sigma_vy, 2));



    //state update estimation
    Mat X_nn = F * X_nn_1;

    //covariance estimate
    Mat P_nn = F * P_nn_1 * F.t() + Q;


    //kalman gain
    Mat S = P_nn + R;
    Mat K = P_nn * S.inv();

    //
    X_nn = X_nn + K * (Z - X_nn_1);

    P_nn = (Mat::eye(K.rows, K.rows, CV_32F) - K) * P_nn_1 * ((Mat::eye(K.rows, K.rows, CV_32F) - K).t()) + K * R * K.t();

    P_nn_1 = P_nn;
    X_nn_1 = X_nn;

}