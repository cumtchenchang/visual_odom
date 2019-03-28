#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <string.h>
#include <ctype.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
// NonLinearOptimization is not used
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <opencv2/core/eigen.hpp>



#include "feature.h"
#include "utils.h"
#include "evaluate_odometry.h"
#include "visualOdometry.h"
#include "Frame.h"
#include "MapPoint.h"

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;



int main(int argc, char **argv)
{

    // -----------------------------------------
    // Load images and calibration parameters
    // -----------------------------------------
//     bool display_ground_truth = false;
    
    std::vector<Matrix> pose_matrix_gt;

    string filepath = string("/home/cc/data/mav0/cam0/data");
    string right_filepath=string("/home/cc/data/mav0/cam1/data");
    string timestampes=string("/home/cc/data/mav0/MH01.txt");
    string strSettingPath = string("/home/cc/code/visual_odom/calibration/kitti00.yaml");
    vector<string> vTimeStamp;
    bool display_ground_truth = false;
    
    
//     pose_matrix_gt = loadPoses("/media/cc/LENOVO_USB_HDD/data/kitti/dataset/poses/00.txt");
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    float bf = fSettings["Camera.bf"];

    cv::Mat projMatrl = (cv::Mat_<float>(3, 4) << fx, 0., cx, 0., 0., fy, cy, 0., 0,  0., 1., 0.);
    cv::Mat projMatrr = (cv::Mat_<float>(3, 4) << fx, 0., cx, bf, 0., fy, cy, 0., 0,  0., 1., 0.);
    cout << "P_left: " << endl << projMatrl << endl;
    cout << "P_right: " << endl << projMatrr << endl;

    // -----------------------------------------
    // Initialize variables
    // -----------------------------------------
    cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat translation_stereo = cv::Mat::zeros(3, 1, CV_64F);

    cv::Mat pose = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat Rpose = cv::Mat::eye(3, 3, CV_64F);
    
    cv::Mat frame_pose = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat frame_pose32 = cv::Mat::eye(4, 4, CV_32F);
    // cv::hconcat(cv::Mat::eye(4, 4, CV_64F), cv::Mat::zeros(3, 1, CV_64F), frame_pose);
    // cv::vconcat(frame_pose, cv::Mat::zeros(1, 4, CV_64F), frame_pose);

    std::cout << "frame_pose " << frame_pose << std::endl;


    cv::Mat trajectory = cv::Mat::zeros(600, 1200, CV_8UC3);
    // show point cloud in another view
//     pcl::visualization::PCLVisualizer *visualizer;

    FeatureSet currentVOFeatures;

//     PointCloud::Ptr features_cloud_ptr (new PointCloud);

    cv::Mat points4D, points3D;


// for euroc datasets
    
   
    ifstream fTimes;
    fTimes.open(timestampes.c_str());
    vTimeStamp.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vTimeStamp.push_back(ss.str());

        }
    }
    
    cout<<"vTimeStamp.size= "<<vTimeStamp.capacity()<<endl;
    
    
//      vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
//      vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
    string imageLeft_t0_path=filepath+"/"+*(vTimeStamp.begin()) + ".png";
    string imageRight_t0_path=right_filepath+"/" + *(vTimeStamp.begin()) + ".png";
    
    int frame_id = 0;
    cv::Mat imageLeft_t0,imageRight_t0;
    imageLeft_t0=cv::imread(imageLeft_t0_path,cv::IMREAD_GRAYSCALE);
    imageRight_t0=cv::imread(imageRight_t0_path,cv::IMREAD_GRAYSCALE);   
    
    // -----------------------------------------
    // Run visual odometry
    // -----------------------------------------

    clock_t tic = clock();

    std::vector<MapPoint> mapPoints;

    std::vector<FeaturePoint> oldFeaturePointsLeft;
    std::vector<FeaturePoint> currentFeaturePointsLeft;
   
    
    for(vector<string>::iterator iter=vTimeStamp.begin()+1;iter!=vTimeStamp.end();++iter)
    {
       clock_t tic = clock();
       frame_id++;
       string mid_name=*iter;
       string imageLeft_t1_path=filepath+"/" +mid_name+ ".png";
       cout<<"imageLeft_t1_path"<<imageLeft_t1_path<<endl;
       string imageRight_t1_path=right_filepath+"/" +mid_name+ ".png";
       cv::Mat imageLeft_t1=cv::imread(imageLeft_t1_path,cv::IMREAD_GRAYSCALE);
       cv::Mat imageRight_t1=cv::imread(imageRight_t1_path,cv::IMREAD_GRAYSCALE);
  
      
       std::vector<cv::Point2f> oldPointsLeft_t0 = currentVOFeatures.points;


        std::vector<cv::Point2f> pointsLeft_t0, pointsRight_t0, pointsLeft_t1, pointsRight_t1;  

        matchingFeatures( imageLeft_t0, imageRight_t0,
                          imageLeft_t1, imageRight_t1, 
                          currentVOFeatures,
                          mapPoints,
                          pointsLeft_t0, 
                          pointsRight_t0, 
                          pointsLeft_t1, 
                          pointsRight_t1);  

        imageLeft_t0 = imageLeft_t1;
        imageRight_t0 = imageRight_t1;

        std::vector<cv::Point2f>& currentPointsLeft_t0 = pointsLeft_t0;
        std::vector<cv::Point2f>& currentPointsLeft_t1 = pointsLeft_t1;

        std::cout << "oldPointsLeft_t0 size : " << oldPointsLeft_t0.size() << std::endl;
        std::cout << "currentFramePointsLeft size : " << currentPointsLeft_t0.size() << std::endl;
        

        std::vector<cv::Point2f> newPoints;
        std::vector<bool> valid; // valid new points are ture



        // ---------------------
        // Triangulate 3D Points
        // ---------------------
        cv::Mat points3D_t0, points4D_t0;
        cv::triangulatePoints( projMatrl,  projMatrr,  pointsLeft_t0,  pointsRight_t0,  points4D_t0);
        cv::convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);
        // std::cout << "points4D_t0 size : " << points4D_t0.size() << std::endl;

        cv::Mat points3D_t1, points4D_t1;
        // std::cout << "pointsLeft_t1 size : " << pointsLeft_t1.size() << std::endl;
        // std::cout << "pointsRight_t1 size : " << pointsRight_t1.size() << std::endl;

        cv::triangulatePoints( projMatrl,  projMatrr,  pointsLeft_t1,  pointsRight_t1,  points4D_t1);
        cv::convertPointsFromHomogeneous(points4D_t1.t(), points3D_t1);
        // std::cout << "points4D_t1 size : " << points4D_t1.size() << std::endl;
        // ---------------------
        // Tracking transfomation
        // ---------------------
        trackingFrame2Frame(projMatrl, projMatrr, pointsLeft_t0, pointsLeft_t1, points3D_t0, rotation, translation_stereo);
        displayTracking(imageLeft_t1, pointsLeft_t0, pointsLeft_t1);


        points4D = points4D_t0;
        frame_pose.convertTo(frame_pose32, CV_32F);
        points4D = frame_pose32 * points4D;
        cv::convertPointsFromHomogeneous(points4D.t(), points3D);
	
        cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);
        std::cout << "rotation: " << rotation_euler << std::endl;
        std::cout << "translation: " << translation_stereo.t() << std::endl;

        cv::Mat rigid_body_transformation;

        if(abs(rotation_euler[1])<0.1 && abs(rotation_euler[0])<0.1 && abs(rotation_euler[2])<0.1)
        {
            integrateOdometryStereo(frame_id, rigid_body_transformation, frame_pose, rotation, translation_stereo);

        } else {

            std::cout << "Too large rotation"  << std::endl;
        }

//         std::cout << "rigid_body_transformation" << rigid_body_transformation << std::endl;

//         std::cout << "frame_pose" << frame_pose << std::endl;


        Rpose =  frame_pose(cv::Range(0, 3), cv::Range(0, 3));
        cv::Vec3f Rpose_euler = rotationMatrixToEulerAngles(Rpose);
//         std::cout << "Rpose_euler" << Rpose_euler << std::endl;

        cv::Mat pose = frame_pose.col(3).clone();



        // pose = -pose;
//         std::cout << "Pose" << pose.t() << std::endl;
//         std::cout << "FPS: " << fps << std::endl;
	int fps=1;
	clock_t toc = clock();
        float frame_time =(toc-tic)*1000/(double)CLOCKS_PER_SEC;
	std::cout<<"the time of every frame(ms) "<<frame_time<<endl;

        display(frame_id, trajectory, pose, pose_matrix_gt, fps, display_ground_truth);

        // break;

    }

    return 0;
}


