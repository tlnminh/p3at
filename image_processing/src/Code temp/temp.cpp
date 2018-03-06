#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <iostream>

using namespace cv;
using namespace std;

Mat S1 = getStructuringElement(MORPH_RECT, Size(3,1), Point(-1,-1));
Mat S2 = getStructuringElement(MORPH_RECT, Size(6,3), Point(-1,-1));

void PlateFinder(Mat src);
int main(int argc, char const *argv[])
{
      Mat input_image = cv::imread("/home/minhtran/catkin_ws/src/image_processing/test-images/car7.jpg");
      Mat gray;
      cvtColor(input_image, gray, COLOR_RGB2GRAY);
      normalize(gray, gray, 0, 255, CV_MINMAX);
      PlateFinder(gray);
      
      return 0;
}

void PlateFinder(Mat src)
{
      double w = src.size().width;
      double h = src.size().height;
      float  ratio_w = 0.25;
      float  ratio_h = 0.1;

      Mat modifiedImage = cvCreateImage(Size(w/2, h/2), IPL_DEPTH_8U, 1);     // Anh dung cho bien doi hinh thai hoc
      Mat pyramidDown = cvCreateImage(Size(w/2, h/2), IPL_DEPTH_8U, 1);
      Mat temp = cvCreateImage(Size(w/2, h/2), IPL_DEPTH_8U, 1);
      Mat thresholded = cvCreateImage(Size(w/2, h/2), IPL_DEPTH_8U, 1);         // Anh nhi phan voi nguong
      Mat mini_threshold = cvCreateImage(Size(w/2, h/2), IPL_DEPTH_8U, 1);
      Mat destination = cvCreateImage(Size(w/2, h/2), IPL_DEPTH_8U, 1);       // Anh da duoc lam ro vung bien so

      pyrDown(src, pyramidDown, Size(w/2, h/2));

      morphologyEx(pyramidDown, modifiedImage, 6, S2);            // 6: BLACKHAT
      normalize(modifiedImage, modifiedImage, 0, 255, CV_MINMAX);

      //Nhi phan hoa anh modifiedImage
      threshold(modifiedImage, thresholded, 10*(mean(modifiedImage).val[0]), 255, THRESH_BINARY);
      mini_threshold = thresholded.clone();

      double w2 = mini_threshold.size().width;
      double h2 = mini_threshold.size().height;
      // Su dung hinh chu nhat co size = 8x16 truot tren toan bo anh
      
      int cnt;
      int nonZero1, nonZero2, nonZero3, nonZero4;
      CvRect rect;

      IplImage copy = mini_threshold;
      IplImage* mini_threshold1 = &copy;

      for (int i = 0; i < mini_threshold1->width-32; i+=4)
      {
            for (int j = 0; j  < mini_threshold1->height-16; j+=4)
            {
                  rect = cvRect(i, j, 16, 8);
                  cvSetImageROI (mini_threshold1, rect);  //ROI = Region of Interest
                  nonZero1 = cvCountNonZero(mini_threshold1);
                  cvResetImageROI(mini_threshold1);

                  rect = cvRect(i+16, j, 16, 8);
                  cvSetImageROI (mini_threshold1, rect);  //ROI = Region of Interest
                  nonZero2 = cvCountNonZero(mini_threshold1);
                  cvResetImageROI(mini_threshold1);

                  rect = cvRect(i, j+8, 16, 8);
                  cvSetImageROI (mini_threshold1, rect);  //ROI = Region of Interest
                  nonZero3 = cvCountNonZero(mini_threshold1);
                  cvResetImageROI(mini_threshold1);

                  rect = cvRect(i+16, j+8, 16, 8);
                  cvSetImageROI (mini_threshold1, rect);  //ROI = Region of Interest
                  nonZero4 = cvCountNonZero(mini_threshold1);
                  cvResetImageROI(mini_threshold1);

                  cnt = 0;
                  if (nonZero1 > 15) { cnt++; }
                  if (nonZero2 > 15) { cnt++; }
                  if (nonZero3 > 15) { cnt++; }
                  if (nonZero4 > 15) { cnt++; }

                  if (cnt > 2)
                  {
                        rect = cvRect (i, j, 32, 16);
                        cvSetImageROI(dst, rect);
                        cvSetImageROI(mini_thresh, rect);
                        cvCopy(mini_thresh, dst);
                        cvResetImageROI(dst);
                        cvResetImageROI(mini_thresh);
                  }
            }
      }

      /*
      Rect roi_rect(0,0,w/5,h/10);
      Mat roi = mini_threshold(roi_rect);
      //Mat complement;
      //bitwise_not(roi,complement);
      //complement.copyTo(roi);

      cout << countNonZero(roi) << endl;
      */
      //rectangle(mini_threshold, Point(0,0), Point(100,100), Scalar(100,100,100), 2, 8);

      //namedWindow( "src", WINDOW_AUTOSIZE );// Create a window for display.
      imshow("src", src); //show the thresholded image
      imshow("modifiedImage", modifiedImage); //show the original image
      imshow("mini_threshold", mini_threshold); //show the original image
      imshow("src1", src1); //show the original image
      waitKey(0);
}
