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
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>

namespace ORB_SLAM2
{

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

