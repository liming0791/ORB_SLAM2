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

#include "ORBextractorGPU.h"

#include<limits.h>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/gpu/gpu.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include<stdint-gcc.h>

using namespace std;

namespace ORB_SLAM2
{

ORBextractorGPU::ORBextractorGPU(int _nFeatures, float _scaleFactor, int _nLevels, int _edgeThreshold, int _firstLevel): nlevels(_nLevels), nfeatures(_nFeatures), scaleFactor(_scaleFactor)
{
	mvScaleFactor.resize(nlevels);
    mvLevelSigma2.resize(nlevels);
    mvScaleFactor[0]=1.0f;
    mvLevelSigma2[0]=1.0f;
    for(int i=1; i<nlevels; i++)
    {
        mvScaleFactor[i]=mvScaleFactor[i-1]*scaleFactor;
        mvLevelSigma2[i]=mvScaleFactor[i]*mvScaleFactor[i];
    }
	
    mvInvScaleFactor.resize(nlevels);
    mvInvLevelSigma2.resize(nlevels);
    for(int i=0; i<nlevels; i++)
    {
        mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
        mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];
    }

    p_orb_gpu = new cv::gpu::ORB_GPU(_nFeatures, _scaleFactor, 
            _nLevels, _edgeThreshold, _firstLevel);

}

ORBextractorGPU::~ORBextractorGPU()
{
	

}

void ORBextractorGPU::operator()(cv::InputArray image, cv::InputArray mask, 
        std::vector<cv::KeyPoint>& keypoints, cv::OutputArray descriptors)
{
    cv::Mat im = image.getMat();
    cv::gpu::GpuMat g_im(im), g_keys, g_descriptors ;
    (*p_orb_gpu)(g_im,cv::gpu::GpuMat(),g_keys,g_descriptors);
    (*p_orb_gpu).downloadKeyPoints(g_keys, keypoints);
    cv::Mat mDescriptors = cv::Mat(g_descriptors);
    descriptors.create(mDescriptors.rows, 32, CV_8U);
    cv::Mat _descriptors = descriptors.getMat();
    mDescriptors.copyTo(_descriptors);
}

}
