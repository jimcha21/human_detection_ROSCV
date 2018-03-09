#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include <iostream>
#include <string>

using namespace cv;
using namespace cv::ximgproc;
using namespace std;

int main(int argc, char const *argv[])
{
    printf("%d\n", argc);
    if( argc < 2)
    {
     cout <<" Usage: display_image left right ijamges" << endl;
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

/*    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", right );                   // Show our image inside it.
    namedWindow( "Display window2", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window2", left );                   // Show our image inside it.
    waitKey(0);*/


    int window_size=3;

    Mat left_for_matcher  = left.clone();
    Mat right_for_matcher = right.clone();


    Ptr<StereoSGBM> left_matcher =  StereoSGBM::create(0,64,9);
   
/*    
    left_matcher->setBlockSize(23);
*/
    left_matcher->setDisp12MaxDiff(-1);

    left_matcher->setMinDisparity(0);

    left_matcher->setMode(0);
    left_matcher->setNumDisparities(32);
    left_matcher->setP1(0);
    left_matcher->setP2(760);
    left_matcher->setPreFilterCap(1);
    left_matcher->setSpeckleRange(0);
    left_matcher->setSpeckleWindowSize(0);
    left_matcher->setUniquenessRatio(0);

    Ptr<DisparityWLSFilter>  wls_filter = createDisparityWLSFilter(left_matcher);
    Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

    Mat left_disp,right_disp,left_disp8,right_disp8;
    left_matcher->compute(left_for_matcher, right_for_matcher, left_disp);
    right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
    
    left_disp.convertTo(left_disp8, CV_8U, 255/(32*16.));
    right_disp.convertTo(right_disp8, CV_8U, 255/(32*16.));

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

    Mat filtered_disp;
    Mat conf_map = Mat(left.rows,left.cols,CV_8U);
    conf_map = Scalar(255);
    Rect ROI;
    wls_filter->setLambda(8000);
    wls_filter->setSigmaColor(1.5);
    wls_filter->filter(left_disp,left,filtered_disp,right_disp);
    
    conf_map = wls_filter->getConfidenceMap();

    // Get the ROI that was used in the last filter call:
    ROI = wls_filter->getROI();

    Mat raw_disp_vis;
    getDisparityVis(left_disp,raw_disp_vis,10);
    namedWindow("raw disparity", WINDOW_AUTOSIZE);
    imshow("raw disparity", raw_disp_vis);
    Mat filtered_disp_vis;
    getDisparityVis(filtered_disp,filtered_disp_vis,10);
    namedWindow("filtered disparity", WINDOW_AUTOSIZE);
    imshow("filtered disparity", filtered_disp_vis);
    waitKey();

    printf("Hello world\n");
	return 0;
}