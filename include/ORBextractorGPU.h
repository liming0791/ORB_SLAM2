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

#ifndef ORBEXTRACTORGPU_H
#define ORBEXTRACTORGPU_H

#include <vector>
#include <list>
#include <opencv/cv.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/gpu/gpu.hpp>

namespace ORB_SLAM2
{

class ORBextractorGPU : public cv::gpu::ORB_GPU
{
public:
	ORBextractorGPU(int _nFeatures = 500, float _scaleFactor = 1.2f, int _nLevels = 8, int _edgeThreshold=31, int _firstLevel = 0);
	
	~ORBextractorGPU();
	
	int inline GetLevels(){ return nlevels; }
	
    float inline GetScaleFactor(){ return scaleFactor; }
	
    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }
	
    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }
	
    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }
	
    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }
	
private:
	int nfeatures;
	int nlevels;
	float scaleFactor;
	
	std::vector<float> mvScaleFactor;
    std::vector<float> mvLevelSigma2;
	
    std::vector<float> mvInvScaleFactor;
    std::vector<float> mvInvLevelSigma2;
	
};

}

#endif
