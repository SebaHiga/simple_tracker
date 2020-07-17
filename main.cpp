#include "opencv2/opencv.hpp"
#include <iostream>
#include "tracker.hpp"
#include "geometry.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <fstream>

using namespace cv;

geom::Point<int> searchByColor(cv::InputArray hsv, cv::OutputArray out, Scalar lower, Scalar upper);

int main(int argc, char** argv){
    if(argc < 1){
        printf("Please add a name file to interact.\n");
        return 0;
    }

    VideoCapture cam(0); // open the default camera

    if(!cam.isOpened())  // check if we succeeded
        return -1;
    
    Mat frame;
    geom::Point<int> p;
    Tracker track;

    int const resize_val = 2;

    namedWindow("frame", cv::WINDOW_AUTOSIZE);
    namedWindow("filtered", cv::WINDOW_AUTOSIZE);

    std::vector<geom::Point<int>> points;

    while(true)
    {
        Mat frame, hsv, filtered;

        cam >> frame; // get a new frame from camera
        flip(frame, frame, 1);
        cvtColor(frame, hsv, COLOR_BGR2HSV);

        points.clear();
        points.reserve(5);

        geom::Point<int> point = searchByColor(hsv, filtered, Scalar(109, 104, 54), Scalar(128, 255, 255));

        points.push_back(point);


        track.update(points);

        for (int i = 0; i < track.objects.size(); i++){
            if(track.objects[i].valid){
                std::stringstream ss;
                ss << track.objects[i].id;
                std::string str;
                str = ss.str();

                cv::circle(frame, point.cv_getPoint(),
                        3, Scalar(255, 255, 255), -1, 8);

                cv::circle(frame, track.objects[i].position.cv_getPoint(),
                        3, Scalar(0, 0, 255), -1, 8);

                putText(frame, str,
                        cv::Point(
                        track.objects[i].position.top, track.objects[i].position.left),
                        1, 1, Scalar(255, 255, 0), 2);

                for(int k = 1; k < track.objects[i].trackline_nonkf.size() - 1; k++){
                    cv::line(frame, track.objects[i].trackline_nonkf[k-1].cv_getPoint(),
                        track.objects[i].trackline_nonkf[k].cv_getPoint(), Scalar(0, 0, 0), 1, 4);
                }
                for(int k = 1; k < track.objects[i].trackline.size() - 1; k++){
                    cv::line(frame, track.objects[i].trackline[k-1].cv_getPoint(),
                        track.objects[i].trackline[k].cv_getPoint(), track.objects[i].getColor(), 2, 4);
                }
            }
        }

        // resize(frame, frame, Size(frame.cols/resize_val, frame.rows/resize_val));
        resize(filtered, filtered, Size(filtered.cols/resize_val, filtered.rows/resize_val));

        imshow("frame", frame);
        imshow("filtered", filtered);

        if(waitKey(1) == 27) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

geom::Point<int> searchByColor(cv::InputArray hsv, cv::OutputArray out, Scalar lower, Scalar upper){
    cv::Mat filtered, kernel, coord, segmented;

    int margin = 500;
    int const max_segment = 5;

    cv::inRange(hsv.getMat(), lower, upper, filtered);
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1), cv::Point(0, 0));
    cv::morphologyEx(filtered, filtered, cv::MORPH_CLOSE, kernel);

    cv::Moments m = moments(filtered, true);
    cv::Point p(m.m10/m.m00, m.m01/m.m00);


    out.assign(filtered);

    cv::Mat nonzero;
    findNonZero(filtered, nonzero);

    // if there's not much information quit
    if(nonzero.total() < 200){
        return geom::Point(0, 0);
    }

    int width = margin, height = margin;

    for (int i = 1; i < max_segment; i++){
        // if out of boundaries, correct roi margins
        margin *= 0.7;
        width = margin;
        height = margin;

        if( filtered.cols < (p.x - margin/2 + margin ) ){
            width = margin - 2 * (p.x + margin/2 - filtered.cols);
        }
        else if(( p.x - margin/2 ) < 0){
            width = margin - 2 * (margin/2 - p.x);
        }

        if( filtered.rows < (p.y - margin/2 + margin ) ){
            height = margin - 2 * (p.y + margin/2 - filtered.rows);
        }
        else if(( p.y - margin/2 ) < 0){
            height = margin - 2 * (margin/2 - p.y);
        }

        // Segment image and reanalyze 
        // Creating mask
        cv::Mat mask(filtered.rows, filtered.cols, CV_8UC1, Scalar(255, 255, 255));
        cv::Rect roi(p.x - (width/2), p.y - (height/2), width, height);
        rectangle(mask, roi, Scalar(0, 0, 0), -1);

        // Segment on image
        subtract(filtered, mask, segmented);

        findNonZero(segmented, nonzero);

        if(nonzero.total() < 10){
            return geom::Point(0, 0);
        }

        m = moments(segmented, true);
        p.x = m.m10/m.m00;
        p.y = m.m01/m.m00;

        rectangle(filtered, roi, Scalar(255, 255, 0));
    }

    Mat points;
    findNonZero(segmented, points);
    Rect boundaries = boundingRect(points);

    rectangle(filtered, boundaries, Scalar(255, 255, 0));

    return geom::Point(p.x, p.y);
}