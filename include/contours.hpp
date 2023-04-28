/*
Copyright © 2023 InterDigital
All Rights Reserved
This program contains proprietary information which is a trade secret/business secret of InterDigital R&D france is protected, even if unpublished, under applicable Copyright laws (including French droit d’auteur) and/or may be subject to one or more patent(s).
*/

#ifndef contours_hpp
#define contours_hpp

#include <opencv2/core.hpp>   //cv::Mat, cv::Point2i
#include <opencv2/imgproc.hpp> //cv::LINE_8
#include <vector>             //std::vector

typedef std::vector<cv::Point2i> Contour;
typedef std::vector<Contour> Contours;
typedef std::vector<Contour> Exteriors;
typedef std::vector<Contours> Interiors;

void      labeling(cv::Mat binIm, Exteriors &exteriors, Interiors &interiors);
void multiLabeling(cv::Mat binIm,  Contours &exteriors);   //Faster?
void multiLabeling(cv::Mat binIm, Exteriors &exteriors, Interiors &interiors);
void  thinLabeling(cv::Mat binIm, Exteriors &exteriors, Interiors &interiors);

void drawAllContours(cv::Mat image, Contours contours, cv::Scalar color,
  int thickness=1, int lineType=cv::LINE_8, int start=0);
void drawAllContours(cv::Mat image, Contours contours,
  int thickness=1, int lineType=cv::LINE_8, int start=0);
void drawAllContours(cv::Mat image, Contours contours, cv::Mat lut,
  int thickness=1, int lineType=cv::LINE_8, int start=0);

void    colorize(cv::Mat image, cv::Mat& colIm, cv::Mat lut);
cv::Mat colorize(cv::Mat image, cv::Mat& colIm);
cv::Mat colorize(cv::Mat image);

//double greenArea(Contour exterior, Contours interiors = Contours());
double pixelArea(Contour exterior, cv::Mat intMask);
int  contourArea(Contour exterior, cv::Mat intMask);
cv::Point2i computeBlobCenter(Contour exterior, cv::Mat intMask);
cv::Point2i     contourCenter(Contour exterior, cv::Mat intMask);

#endif /* contours_hpp */
