#include "opencv2/opencv.hpp"
#include <deque>
#include <deque>
using namespace cv;
// Get Relevant Point HeatMap
//Mat ClipFrameAround(Point pnt, &Mat frame){
//
//}
int getMaxAreaContourId(std::vector<std::vector<cv::Point>> contours)
{
    double maxArea = 0;
    int maxAreaContourId = -1;
    for (int j = 0; j < contours.size(); j++)
    {
        double newArea = cv::contourArea(contours.at(j));
        if (newArea > maxArea)
        {
            maxArea = newArea;
            maxAreaContourId = j;
        } // End if
    }     // End for
    return maxAreaContourId;
} // End function

int main(int, char **)
{
    VideoCapture cap(0); // open the default camera
    if (!cap.isOpened()) // check if we succeeded
        return -1;
    Mat edges;
    namedWindow("Tracking", 1);

    int UpdateEveryNFrames = 6;
    int FramesPassed = 0;
    float BallDiameter = .07; // m
    float PixelGravity;
    
    int FPS = 30.0;
    float TimeStepSize = 1.0 / FPS;

    int EulerSteps = 18;

    Scalar color = Scalar(0, 0, 255);

    Mat frame, hsv_frame, thresh_frame;
    std::deque<Point> Trajectory;
    std::deque<Point> Last_Trajectory;
    std::deque<std::deque<Point>> Predictions;
    
    for (;;)
    {
        FramesPassed++;
        // Get a new frame from camera
        cap >> frame;
        cvtColor(frame, hsv_frame, COLOR_BGR2HSV);
        inRange(hsv_frame, Scalar(16, 135, 129), Scalar(27, 230, 255), thresh_frame);
        medianBlur(thresh_frame, thresh_frame, 3);

        std::vector<std::vector<Point>> contours;
        findContours(thresh_frame, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

        std::vector<std::vector<Point>> contours_poly(contours.size());
        std::vector<Point2f> centers(contours.size());
        std::vector<float> radius(contours.size());

        for (size_t i = 0; i < contours.size(); i++)
        {
            approxPolyDP(contours[i], contours_poly[i], 3, true);
            minEnclosingCircle(contours_poly[i], centers[i], radius[i]);
        }
        Mat drawing = frame; 
        if (contours.size() > 2)
        {
            // Find the Ball by Largest Contour
            int max_contour_id = getMaxAreaContourId(contours);
            circle(drawing, centers[max_contour_id], (int)radius[max_contour_id], color, 2);

            // Update Real Tracking Path
            if (Trajectory.size() < 10)
            {
                Trajectory.push_back(centers[max_contour_id]);
            }
            else
            {
                Trajectory.push_back(centers[max_contour_id]);
                Trajectory.pop_front();
            }
            Point V0 = Point(0,0);
            //DrawTrueTrajectory(Trajectory,drawing);
            for (unsigned i = 0; i < Trajectory.size() - 1; i++)
            {
                V0 += Point(Trajectory.at(i).x - Trajectory.at(i + 1).x, Trajectory.at(i).y - Trajectory.at(i + 1).y);
                line(drawing, Trajectory.at(i) + Point(10, 0), Trajectory.at(i) + Point(-10, 0), Scalar(255, 0, 0), 6, 8);
                line(drawing, Trajectory.at(i) + Point(0, 10), Trajectory.at(i) + Point(0, -10), Scalar(255, 0, 0), 6, 8);
                line(drawing, Trajectory.at(i), Trajectory.at(i + 1), Scalar(255, 0, 0), 4, 8);
            }
            if (Trajectory.size() > 2 ){
                V0 = Point(Trajectory.at(Trajectory.size() - 1).x - Trajectory.at(Trajectory.size() - 2).x, Trajectory.at(Trajectory.size() - 1).y - Trajectory.at(Trajectory.size() - 2).y);
            } 

            int PixelsPerMeter = (2 * radius[max_contour_id])/ BallDiameter;

            Point start = Trajectory.back();
            Point curr = start;
            Point last = curr;
            //DrawPredictedTrajectory(Trajectory,drawing);
            for (int t = 1; t < EulerSteps; t++)
            {
                curr = Point(start.x + V0.x * t, start.y + V0.y * t - .5 * -9.81 * PixelsPerMeter * ((TimeStepSize*t) * (TimeStepSize*t)));
                line(drawing, curr + Point(10, 0), curr + Point(-10, 0), Scalar(255, 0, 255), 6, 8);
                line(drawing, curr + Point(0, 10), curr + Point(0, -10), Scalar(255, 0, 255), 6, 8);
                line(drawing, last, curr, Scalar(255, 0, 255), 4, 8);
                last = curr;
            }

            if (Trajectory.size() < 10)
            {
                Trajectory.push_back(centers[max_contour_id]);
            }
            else
            {
                Trajectory.push_back(centers[max_contour_id]);
                Trajectory.pop_front();
            }
            //Predictions.push();
            Last_Trajectory = Trajectory;
        }
        else
        {
            if (Last_Trajectory.size() > 0)
            {
                for (unsigned i = 0; i < Last_Trajectory.size() - 1; i++)
                {
                    line(drawing, Last_Trajectory.at(i) + Point(10, 0), Last_Trajectory.at(i) + Point(-10, 0), Scalar(255, 0, 0), 6, 8);
                    line(drawing, Last_Trajectory.at(i) + Point(0, 10), Last_Trajectory.at(i) + Point(0, -10), Scalar(255, 0, 0), 6, 8);
                    line(drawing, Last_Trajectory.at(i), Last_Trajectory.at(i + 1), Scalar(255, 0, 0), 4, 8);
                }
                if (Trajectory.size() > 0){
                    Trajectory.clear();
                }
            }
        }
        imshow("Tracking", drawing);
        if (waitKey(30) >= 0)
            break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
// g++ $(pkg-config --cflags --libs opencv4) -std=c++11  sample.cpp -o sample
