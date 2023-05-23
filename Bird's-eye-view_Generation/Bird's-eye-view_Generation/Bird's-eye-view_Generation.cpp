#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
using namespace std;
using namespace cv;

int main()
{
    // If there are images for calibration, use the codes from Line 15 to Line 105 rather than those from Line 109 to Line 124.
    /*
    // Open the text file that records the images needed for calibration.
    ifstream fin("Calibration.txt");
    if (!fin) {
        cerr << "File Not Found Error." << endl;
        return -1;
    }
    ofstream fout("CalibrationLog.txt");

    int image_nums = 0;
    int points_per_row = 9, points_per_col = 6;
    Size image_size;
    Size corner_size(points_per_row, points_per_col);
    vector<Point2f> points_per_image;
    vector<vector<Point2f>> points_all_images;
    string image_file_name;
    Mat image_raw, image_grey;

    // Read the image files and find the calibration board and the pixel coordinates of its key points in these pictures.
    while (getline(fin, image_file_name)) {
        image_raw = imread(image_file_name);
        cvtColor(image_raw, image_grey, COLOR_BGR2GRAY);
        bool success = findChessboardCorners(image_grey, corner_size, points_per_image);
        if (!success) {
            cout << "Corner Detection for " << image_file_name << " Failed." << endl;
            return 1;
        }
        else {
            cout << "Corner Detection for " << image_file_name << " Succeeded." << endl;
            find4QuadCornerSubpix(image_grey, points_per_image, Size(5, 5));
            points_all_images.push_back(points_per_image);
        }
        if (image_nums == 0) {
            image_size.width = image_raw.cols;
            image_size.height = image_raw.rows;
        }
        image_nums++;
    }
    fin.close();

    Size block_size(Point2f(25.4, 25.4));
    vector<Point3f> points3D_per_image;
    Point3f point3D;

    // Set the world coordinates of the key points on the calibration board.
    for (int i = 0; i < corner_size.height; i++) {
        for (int j = 0; j < corner_size.width; j++) {
            point3D = Point3f(block_size.width * j, block_size.height * i, 0);
            points3D_per_image.push_back(point3D);
        }
    }
    vector<vector<Point3f>> points3D_all_images(image_nums, points3D_per_image);
    int point_counts = corner_size.area();

    Mat cameraMat(3, 3, CV_32FC1, Scalar::all(0));
    Mat distCoeffs(1, 5, CV_32FC1, Scalar::all(0));
    vector<Mat> rotationVec;
    vector<Mat> translationMat;

    // Calibrate the camera.
    calibrateCamera(points3D_all_images, points_all_images, image_size, cameraMat, distCoeffs, rotationVec, translationMat, 0);

    double total_err = 0.0;
    double err = 0.0;
    vector<Point2f> points_reproject;
    fout << "Calculating calibration errors of every image . . ." << endl;
    for (int i = 0; i < image_nums; i++) {
        points_per_image = points_all_images[i];
        points3D_per_image = points3D_all_images[i];
        projectPoints(points3D_per_image, rotationVec[i], translationMat[i], cameraMat, distCoeffs, points_reproject);
        Mat detect_points_Mat(1, points_per_image.size(), CV_32FC2);
        Mat points_reproj_Mat(1, points_reproject.size(), CV_32FC2);
        for (int j = 0; j < points_per_image.size(); j++) {
            detect_points_Mat.at<Vec2f>(0, j) = Vec2f(points_per_image[j].x, points_per_image[j].y);
            points_reproj_Mat.at<Vec2f>(0, j) = Vec2f(points_reproject[j].x, points_reproject[j].y);
        }
        err = norm(points_reproj_Mat, detect_points_Mat);
        total_err += err /= point_counts;
        fout << "Calibration error of Image No." << i + 1 << " : " << err << endl;
    }
    fout << "Mean calibration error of all images : " << total_err / image_nums << endl << endl;

    fout << "Intrinsic parameters :" << endl << cameraMat << endl << endl;
    fout << "Distortion parameters :" << endl << distCoeffs << endl << endl;
    Mat rotateMat = Mat(3, 3, CV_32FC1, Scalar::all(0));
    for (int i = 0; i < image_nums; i++) {
        Rodrigues(rotationVec[i], rotateMat);
        fout << "Extrinsic parameters of Image No." << i + 1 << " :" << endl << endl;
        fout << "Rotation matrix :" << endl << rotateMat << endl << endl;
        fout << "Translation vector :" << endl << translationMat[i] << endl << endl;
    }
    fout.close();
    */

    // If there's no image for calibration, use the codes from Line 109 to Line 124 rather than those from Line 15 to Line 105.
    Mat image_grey;
    Mat cameraMat = (Mat_<double>(3, 3) << 2858.67327582012, 0, 1842.422793753554, 0, 2885.015011675339, 1382.559613891173, 0, 0, 1);
    Mat distCoeffs = (Mat_<double>(1, 5) << 0.2942999624117501, -3.840871146301559, -0.006278249865101324, 0.003666120134607291, 16.25106189045292);
    Size image_size = Size(3648, 2736);
    int points_per_row = 9, points_per_col = 6;
    Size corner_size(points_per_row, points_per_col);
    vector<Point2f> points_per_image;
    Size block_size(Point2f(25.4, 25.4));
    vector<Point3f> points3D_per_image;
    Point3f point3D;
    for (int i = 0; i < corner_size.height; i++) {
        for (int j = 0; j < corner_size.width; j++) {
            point3D = Point3f(block_size.width * j, block_size.height * i, 0);
            points3D_per_image.push_back(point3D);
        }
    }

    Mat map1(image_size, CV_32FC1);
    Mat map2(image_size, CV_32FC1);
    Mat R = Mat::eye(3, 3, CV_32F);
    Mat image_dist, image_undist;

    // Generate the undistorted image.
    initUndistortRectifyMap(cameraMat, distCoeffs, R, cameraMat, image_size, CV_32FC1, map1, map2);
    image_dist = imread("Target.jpg");
    image_undist = image_dist.clone();
    remap(image_dist, image_undist, map1, map2, INTER_LINEAR);
    imwrite("UndistortedTarget.jpg", image_undist);

    Mat H(3, 3, CV_32F, Scalar::all(0));
    vector<Point2f> objPts(4);
    vector<Point2f> imgPts(4);
    int indexArray[4] = {
        0,
        points_per_row - 1,
        (points_per_col - 1) * points_per_row,
        (points_per_col - 1) * points_per_row + points_per_row - 1
    };
    cvtColor(image_undist, image_grey, COLOR_BGR2GRAY);

    // Find the origin pixel coordinates of the key points on the calibration board in the undistorted image.
    bool success = findChessboardCorners(image_grey, corner_size, points_per_image);
    if (!success) {
        cout << "Corner Detection for Target.jpg Failed." << endl;
        return 1;
    }
    else {
        cout << "Corner Detection for Target.jpg Succeeded." << endl;
        find4QuadCornerSubpix(image_grey, points_per_image, Size(5, 5));
    }

    // Set the ideal pixel coordinates of the key points on the calibration board in the undistorted image.
    for (int i = 0; i < 4; i++) {
        objPts[i].x = 4 * points3D_per_image[indexArray[i]].x + 150;
        objPts[i].y = 4 * points3D_per_image[indexArray[i]].y + 850;
        imgPts[i] = points_per_image[indexArray[i]];
    }

    // Estimate the homography transformation and generate the bird's-eye-view image.
    H = getPerspectiveTransform(objPts, imgPts);
    Mat birdsEyeView = image_undist.clone();
    warpPerspective(image_undist, birdsEyeView, H, image_size, CV_INTER_LINEAR + CV_WARP_INVERSE_MAP + CV_WARP_FILL_OUTLIERS);
    imwrite("BirdsEyeView.jpg", birdsEyeView);

    return 0;
}
