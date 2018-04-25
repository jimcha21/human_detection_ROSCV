//#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <iostream>

using namespace cv;

int main(int argc, char** argv)
{
    String right_vid = "/home/jimcha/data/exterior/synced/videos/right4.mp4";
    String left_vid = "/home/jimcha/data/exterior/synced/videos/right4.mp4";
    VideoCapture r_capture(right_vid);
    VideoCapture l_capture(left_vid);

    Mat l_frame,r_frame;

    if( !r_capture.isOpened() || !l_capture.isOpened())
        throw "Error when reading steam_avi";

    namedWindow( "r", 1);
    namedWindow( "l", 1);
    for( ; ; )
    {
        l_capture >> l_frame;
        r_capture >> r_frame;

        if(l_frame.empty() || r_frame.empty())
            break;

        imshow("l", l_frame);
        imshow("r", r_frame);
        waitKey(200); // waits to display frame
    }
    waitKey(0); // key press to close window
    // releases and window destroy are automatic in C++ interface
}