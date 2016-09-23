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


#include "Converter.h"

namespace ORB_SLAM2
{

std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors)
{
    std::vector<cv::Mat> vDesc;
    vDesc.reserve(Descriptors.rows);
    for (int j=0;j<Descriptors.rows;j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;
}

g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvT)
{
    Eigen::Matrix<double,3,3> R;
    R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
         cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
         cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

    return g2o::SE3Quat(R,t);
}

cv::Mat Converter::toCvMat(const g2o::SE3Quat &SE3)
{
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
    return toCvMat(eigMat);
}

cv::Mat Converter::toCvMat(const g2o::Sim3 &Sim3)
{
    Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
    Eigen::Vector3d eigt = Sim3.translation();
    double s = Sim3.scale();
    return toCvSE3(s*eigR,eigt);
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m)
{
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,3,1> &m)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
            cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

cv::Mat Converter::toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t)
{
    cv::Mat cvMat = cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cvMat.at<float>(i,j)=R(i,j);
        }
    }
    for(int i=0;i<3;i++)
    {
        cvMat.at<float>(i,3)=t(i);
    }

    return cvMat.clone();
}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Mat &cvVector)
{
    Eigen::Matrix<double,3,1> v;
    v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

    return v;
}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Point3f &cvPoint)
{
    Eigen::Matrix<double,3,1> v;
    v << cvPoint.x, cvPoint.y, cvPoint.z;

    return v;
}

Eigen::Matrix<double,3,3> Converter::toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

std::vector<float> Converter::toQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    std::vector<float> v(4);
    v[0] = q.w();
    v[1] = q.x();
    v[2] = q.y();
    v[3] = q.z();

    //float sqsum = v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3];

    //cv::Mat _M = toMatrix(v);

    //Eigen::Matrix3d R = q.toRotationMatrix();

    //printf("\n===Original matrix:\n %f %f %f \n %f %f %f \n %f %f %f \n\n", 
    //        M.at<float>(0), M.at<float>(1), M.at<float>(2), 
    //        M.at<float>(3), M.at<float>(4), M.at<float>(5), 
    //        M.at<float>(6), M.at<float>(7), M.at<float>(8) );

    //printf("\n===Convert from Matrix: %f %f %f %f square sum: %f\n\n", v[0], v[1], v[2], v[3], sqsum);

    //printf("\n===New matrix from Mine:\n %f %f %f \n %f %f %f \n %f %f %f \n\n", 
    //        _M.at<float>(0), _M.at<float>(1), _M.at<float>(2), 
    //        _M.at<float>(3), _M.at<float>(4), _M.at<float>(5), 
    //        _M.at<float>(6), _M.at<float>(7), _M.at<float>(8) );

    //printf("\n===New matrix from Eigen:\n %f %f %f \n %f %f %f \n %f %f %f \n\n", 
    //        R(0,0), R(0,1), R(0,2),
    //        R(1,0), R(1,1), R(1,2),
    //        R(2,0), R(2,1), R(2,2)); 

    return v;
}

cv::Mat Converter::toMatrix(const std::vector<float> &q)
{
    cv::Mat RMat(3, 3, CV_32F);
    float* R = RMat.ptr<float>(0);

    float q02 = q[0]*q[0], q12 = q[1]*q[1], q22 = q[2]*q[2], q32 = q[3]*q[3];
    float dq0q1 = 2*q[0]*q[1], dq1q2 = 2*q[1]*q[2], dq2q3 = 2*q[2]*q[3], dq3q0 = 2*q[3]*q[0];
    float dq0q2 = 2*q[0]*q[2], dq1q3 = 2*q[1]*q[3];
    R[0] = q02 + q12 - q22 -q32;   R[3] = dq1q2 + dq3q0;          R[6] = dq1q3 - dq0q2;
    R[1] = dq1q2 - dq3q0;          R[4] = q02 - q12 + q22 - q32;  R[7] = dq2q3 + dq0q1;
    R[2] = dq1q3 + dq0q2;          R[5] = dq2q3 - dq0q1;          R[8] = q02 - q12 - q22 + q32;
    
	return RMat;
}

void Converter::GL2IMUAxis(std::vector<float> &_Q)
{
    if(_Q.size() == 4){
        // Quaternion
        float x = _Q[1], y = _Q[2], z = _Q[3];
        _Q[1] = -z;
        _Q[2] = x;
        _Q[3] = y;
    }else if(_Q.size() == 3){
        // Translation
        float x = _Q[0], y = _Q[1], z = _Q[2];
        _Q[0] = -z;
        _Q[1] = x;
        _Q[2] = y;
    }
        
}

void Converter::IMU2GLAxis(std::vector<float> &_Q)
{
    if(_Q.size() == 4){
        // Quaternion
        float x = _Q[1], y = _Q[2], z = _Q[3];
        _Q[1] = y;
        _Q[2] = z;
        _Q[3] = -x;
    }else if(_Q.size() == 3){
        // Translation
        float x = _Q[0], y = _Q[1], z = _Q[2];
        _Q[0] = y;
        _Q[1] = z;
        _Q[2] = -x;
    }
}

} //namespace ORB_SLAM
