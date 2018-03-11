#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include <iostream>
#include <string>
#include <cstdlib>
#include <fstream>
#include <stdlib.h>     //for using the function sleep
#include <unistd.h>

using namespace cv;
using namespace cv::ximgproc;
using namespace std;

Mat left_disp,right_disp;
Mat left_for_matcher,right_for_matcher;
const int slider_max = 100;
int minDisparity=0;
int numDisparities=1; // divisible with 16
int blockSize=9; // must be odd 
int P1=0;
int P2=0;
int disp12MaxDiff=0;
int preFilterCap=0;
int uniquenessRatio=0;
int speckleWindowSize=0;
int speckleRange=0;
int mode=StereoSGBM::MODE_SGBM;
int wls_lambda=8000;
int wls_sigma=15; //must divide with 10 , must be float 
double alpha,beta;

void on_trackbar( int , void* )
{

    //cout<<' '<<minDisparity<<' '<<numDisparities<<' '<<blockSize<<' '<<P1<<' '<<P2<<' '<<disp12MaxDiff<<' '<<preFilterCap<<' '<<uniquenessRatio<<' '<<speckleWindowSize<<' '<<speckleRange<<' '<<mode<<' '<<endl;
    Ptr<StereoSGBM> left_matcher =  StereoSGBM::create(minDisparity,numDisparities*16,blockSize,P1,P2,disp12MaxDiff,preFilterCap,uniquenessRatio,speckleWindowSize,speckleRange,mode);
    Ptr<DisparityWLSFilter>  wls_filter = createDisparityWLSFilter(left_matcher);
    Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
    
    
    left_matcher->compute(left_for_matcher, right_for_matcher, left_disp);
    right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
    //usleep(2000); 

    Mat filtered_disp;
    Mat conf_map = Mat(left_for_matcher.rows,left_for_matcher.cols,CV_8U);
    conf_map = Scalar(255);
    Rect ROI;
    wls_filter->setLambda(wls_lambda);
    wls_filter->setSigmaColor(wls_sigma/10);
    wls_filter->filter(left_disp,left_for_matcher,filtered_disp,right_disp);

    conf_map = wls_filter->getConfidenceMap();

    // Get the ROI that was used in the last filter call:
    ROI = wls_filter->getROI();

    Mat raw_disp_vis;
    getDisparityVis(left_disp,raw_disp_vis,20);
    imshow("SGBM with WLS", raw_disp_vis);

    Mat filtered_disp_vis;
    getDisparityVis(filtered_disp,filtered_disp_vis,20);
    imshow("WLS RESULT", filtered_disp_vis);

    imshow("Original", left_for_matcher);

    //addWeighted( src1, alpha, src2, beta, 0.0, dst);
}

int getValue(){
    cout<<"Give the value:\n";
    int val;
    cin>>val;
    return val;
}

double getfloatValue(){
    cout<<"Give the value:\n";
    double val;
    cin>>val;
    return val;
}

int main(int argc, char const *argv[])
{
    printf("%d\n", argc);
    if( argc < 2)
    {
     cout <<" Usage: display_image left right images" << endl;
     return -1;
    }
   
    Mat left,right;
    left = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file
    right = imread(argv[2], CV_LOAD_IMAGE_COLOR);   // Read the file
    
    if(!right.data  || !left.data)                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }    

    left_for_matcher  = left.clone();
    right_for_matcher = right.clone();
    
    Ptr<StereoSGBM> left_matcher =  StereoSGBM::create(minDisparity,numDisparities,blockSize,P1,P2,disp12MaxDiff,preFilterCap,uniquenessRatio,speckleWindowSize,speckleRange,mode);
       
/*    bool stopit=true;
    while(!stopit){
        cout<<"Give value for, \n"
        "minDisparity:1\n"
        "numDisparities:2\n"
        "blockSize:3\n"
        "p1:4\n"
        "p2:5\n"
        "disp12MaxDiff:6\n"
        "preFilterCap:7\n"
        "uniquenessRatio:8\n"
        "speckleWindowSize:9\n"
        "speckleRange:10\n"
        "mode:11\n->";
        
        int choice;
        cin>>choice;
        if(choice==0){
            stopit=true;
            continue;
        }else if(choice==1)
            minDisparity=getValue();
        else if(choice==2)
            numDisparities=getValue();
        else if(choice==3)
            blockSize=getValue();
        else if(choice==4)
            P1=getValue();
        else if(choice==5)
            P2=getValue();
        else if(choice==6)
            disp12MaxDiff=getValue();
        else if(choice==7)
            preFilterCap=getValue();
        else if(choice==8)
            uniquenessRatio=getValue();
        else if(choice==9)
            speckleWindowSize=getValue();
        else if(choice==10)
            speckleRange=getValue();
        else if(choice==11)
            mode=getValue();
        else if(choice==12)
            wls_lambda=getValue();
        else if(choice==13)
            wls_sigma=getValue();

        Ptr<StereoSGBM> left_matcher =  StereoSGBM::create(minDisparity,numDisparities,blockSize,P1,P2,disp12MaxDiff,preFilterCap,uniquenessRatio,speckleWindowSize,speckleRange,mode);
        Ptr<DisparityWLSFilter>  wls_filter = createDisparityWLSFilter(left_matcher);
        Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

        left_matcher->compute(left_for_matcher, right_for_matcher, left_disp);
        right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);

        Mat filtered_disp;
        Mat conf_map = Mat(left.rows,left.cols,CV_8U);
        conf_map = Scalar(255);
        Rect ROI;
        wls_filter->setLambda(wls_lambda);
        wls_filter->setSigmaColor(wls_sigma);
        wls_filter->filter(left_disp,left,filtered_disp,right_disp);

        conf_map = wls_filter->getConfidenceMap();

        // Get the ROI that was used in the last filter call:
        ROI = wls_filter->getROI();

        Mat filtered_disp_vis;
        getDisparityVis(filtered_disp,filtered_disp_vis,20);
        namedWindow("filtered disparity", WINDOW_AUTOSIZE);
        imshow("filtered disparity", filtered_disp_vis);
        imwrite("disparity.png",filtered_disp_vis);
        Mat raw_disp_vis;
        getDisparityVis(left_disp,raw_disp_vis,20);
        namedWindow("raw disparity", WINDOW_AUTOSIZE);
        imshow("raw disparity", raw_disp_vis);

        waitKey(0);

        system("clear");

    }*/

    namedWindow("SGBM with WLS", WINDOW_AUTOSIZE);
    
    createTrackbar( "minDisparity", "SGBM with WLS", &minDisparity, slider_max, on_trackbar );
    createTrackbar( "numDisparities", "SGBM with WLS", &numDisparities, 10, on_trackbar );
    createTrackbar( "P1", "SGBM with WLS", &P1, 2000, on_trackbar );
    createTrackbar( "P2", "SGBM with WLS", &P2, 2000, on_trackbar );
    createTrackbar( "disp12MaxDiff", "SGBM with WLS", &disp12MaxDiff, slider_max, on_trackbar );
    createTrackbar( "preFilterCap", "SGBM with WLS", &preFilterCap, slider_max, on_trackbar );
    createTrackbar( "uniquenessRatio", "SGBM with WLS", &uniquenessRatio, slider_max, on_trackbar );
    createTrackbar( "speckleWindowSize", "SGBM with WLS", &speckleWindowSize, slider_max, on_trackbar );
    createTrackbar( "speckleRange", "SGBM with WLS", &speckleRange, slider_max, on_trackbar );
    createTrackbar( "mode", "SGBM with WLS", &mode, 3, on_trackbar );
    createTrackbar( "wls_lambda", "SGBM with WLS", &wls_lambda, 20000, on_trackbar );    
    createTrackbar( "wls_sigma", "SGBM with WLS", &wls_sigma, slider_max, on_trackbar );

    on_trackbar( P1, 0 );
    waitKey(0);


/*    left_disp.convertTo(left_disp8, CV_8U, 255/(numDisparities*16.));
    right_disp.convertTo(right_disp8, CV_8U, 255/(numDisparities*16.));*/

/*    namedWindow("left", 1);
    imshow("left", left);
    namedWindow("right", 1);
    imshow("right", right);
    namedWindow("disparity", 0);
    imshow("disparity", left_disp8);
    printf("press any key to continue...");
    fflush(stdout);
    waitKey();
    printf("\n");*/

	return 0;
}