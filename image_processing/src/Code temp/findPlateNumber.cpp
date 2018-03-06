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
      // Su dung hinh chu nhat kich thuoc (w/5,h/10) de chay het anh
      double max = 0;
      double mini = 1000, maxi, minj=1000, maxj;
      // Tim so diem anh trang lon nhat co trong 1 vung
      for (int i = 0; i < (1-ratio_w)*w2; i+=1)
      {
            for (int j = 0; j < (1-ratio_h)*h2; j+=1)
            {
                  Rect roi_rect(i,j,ratio_w*w2,ratio_h*h2);
                  Mat roi = mini_threshold(roi_rect);
                  
                  if (countNonZero(roi) > max) {max = countNonZero(roi);}
            }
      }
      // tim toa do bao cac vung chua so luong diem trang lon nhat
      for (int i = 0; i < (1-ratio_w)*w2; i+=1)
      {
            for (int j = 0; j < (1-ratio_h)*h2; j+=1)
            {
                  Rect roi_rect(i,j,ratio_w*w2,ratio_h*h2);
                  Mat roi = mini_threshold(roi_rect);
                  if (countNonZero(roi) == max)
                  {
                        if (i < mini) {mini = i;}
                        if (i > maxi) {maxi = i;}
                        if (j < minj) {minj = j;}
                        if (j > maxj) {maxj = j;}
                        //rectangle(mini_threshold, Point(i,j), Point(i+w2/5,j+h2/10), (255,255,255), 2, 8);
                                         
                  }
            }
      }
      Mat src1 = src.clone();
      // Ve hinh chu nhat ROI
      rectangle(src1, 2*Point(mini,minj), 2*Point(maxi+ratio_w*w2,maxj+ratio_h*h2), (255,255,255), 2, 8); 
      //Xoa het nhung diem ko can thiet
      rectangle(src1, Point(0,0), 2*Point(w2,minj), (0,0,0), -1, 8); 
      rectangle(src1, 2*Point(0,minj), 2*Point(mini,maxj+ratio_h*h2), (0,0,0), -1, 8); 
      rectangle(src1, 2*Point(maxi+ratio_w*w2,minj), 2*Point(w2,maxj+ratio_h*h2), (0,0,0), -1, 8); 
      rectangle(src1, 2*Point(0,maxj+ratio_h*h2), 2*Point(w2,h2), (0,0,0), -1, 8); 
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

