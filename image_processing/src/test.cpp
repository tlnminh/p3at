#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <iostream>

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
    if( argc != 2)
    {
     cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
     return -1;
    }   

    Mat im_rgb, im_hsv;
    im_rgb = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file

    if(! im_rgb.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << endl ;
        return -1;
    }

    cvtColor(im_rgb,im_hsv, COLOR_BGR2HSV);
    vector<Mat> channels;
    split(im_hsv, channels);


//    Mat split = im_hsv[:,:,:];



    int iLowH = 0;
    int iHighH = 179;

    int iLowS = 174; 
    int iHighS = 236;

    int iLowV = 124;
    int iHighV = 174;

    int lowThreshold = 0;
    int ratio = 3;
    int kernel_size = 3;

    Mat im_thresholdedH, im_thresholdedS, im_thresholdedV;

    namedWindow( "Control", WINDOW_AUTOSIZE );// Create a window for display.
    cvCreateTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);

    cvCreateTrackbar( "Low Threshold:", "Control", &lowThreshold, 100);

    while(1)
    {
        //inRange(im_hsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), im_thresholded);
        inRange(channels[0], iLowH, iHighH, im_thresholdedH);
        inRange(channels[1], iLowS, iHighS, im_thresholdedS);
        inRange(channels[2], iLowV, iHighV, im_thresholdedV);

/*
        Mat im_blur;
        // Loc Trung Vi Median
        for ( int i = 1; i < 31; i = i + 2 )
        { 
            medianBlur ( im_thresholded, im_blur, 5 );
        }

        Mat im_canny;
        /// Canny detector
        Canny( im_blur, im_canny, lowThreshold, lowThreshold*ratio, kernel_size );

        vector<vector<Point> > im_contours;
        vector<Vec4i> hierarchy;
        RNG rng(12345);

        findContours( im_canny, im_contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
        Mat drawing = Mat::zeros( im_thresholded.size(), CV_8UC3 );
        for( int i = 0; i< im_contours.size(); i++ )
        {
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours( drawing, im_contours, i, color, 1, 8, hierarchy, 0, Point() );
        } 
        */
        imshow( "Original image", im_rgb );   
        //imshow( "Converted window", im_thresholded );
        imshow( "Hue", channels[0] );
        imshow( "Saturation", channels[1] );
        imshow( "Value", channels[2] );
        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
            cout << "esc key is pressed by user" << endl;
            break; 
       } 
    }
    return 0;
}

/*
rosrun image_processing image_processing_test /home/minhtran/catkin_ws/src/image_processing/test-images/1.jpg
*/