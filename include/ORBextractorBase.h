/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBEXTRACTORBASE_H
#define ORBEXTRACTORBASE_H

#include <vector>
#include <list>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>

using namespace cv;

namespace ORB_SLAM2
{

class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

extern const int PATCH_SIZE;
extern const int HALF_PATCH_SIZE;
extern const int EDGE_THRESHOLD;

extern const float factorPI;

extern float IC_Angle(const Mat& image, Point2f pt,  const vector<int> & u_max);
extern void computeOrbDescriptor(const KeyPoint& kpt,
                                 const Mat& img, const Point* pattern,
                                 uchar* desc);
extern int bit_pattern_31_[];
extern void computeOrientation(const Mat& image, vector<KeyPoint>& keypoints, const vector<int>& umax);
extern void computeDescriptors(const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors, const vector<Point>& pattern);

class ORBextractorBase
{
public:
    virtual ~ORBextractorBase(){}
    virtual void operator()( cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint>& keypoints, cv::OutputArray descriptors) = 0;
    virtual int GetLevels() = 0;
    virtual float GetScaleFactor() = 0;
    virtual std::vector<float> GetScaleFactors() = 0;
    virtual std::vector<float> GetInverseScaleFactors() = 0;
    virtual std::vector<float> GetScaleSigmaSquares() = 0;	
    virtual std::vector<float> GetInverseScaleSigmaSquares() = 0;
};

} //namespace ORB_SLAM

#endif

