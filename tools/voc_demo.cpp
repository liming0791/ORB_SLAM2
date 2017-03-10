#include <sys/time.h>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include "CLATCHextractor.h"
#include "ORBextractor.h"
#include "ORBVocabulary.h"

using namespace cv;
using namespace DBoW2;
using namespace ORB_SLAM2;

//struct timeval start, end;
//
//void timeBegin()
//{
//    gettimeofday(&start, 0);
//}
//
//void timeEnd()
//{
//    gettimeofday(&end, 0);
//    printf(" time cost: %f ms, ", ((end.tv_sec - start.tv_sec)*1000000u +
//             end.tv_usec - start.tv_usec)/1e3);
//}

const int TH_LOW = 50;
const float RATIO = 0.7;

static void help( char** argv )
{
    std::cout << "\nUsage: " << argv[0] << " [path/to/image1] [path/to/image2] \n"
              << "This is an example on how to use the keypoint descriptor presented in the following paper: \n"
              << "A. Alahi, R. Ortiz, and P. Vandergheynst. FREAK: Fast Retina Keypoint. \n"
              << "In IEEE Conference on Computer Vision and Pattern Recognition, 2012. CVPR 2012 Open Source Award winner \n"
              << std::endl;
}

std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors)                        
{
    std::vector<cv::Mat> vDesc;                                                                  
    vDesc.reserve(Descriptors.rows);                                                         
    for (int j=0;j<Descriptors.rows;j++)                                                     
        vDesc.push_back(Descriptors.row(j));                                                      
    return vDesc;                                                                                    
}

 // Bit set count operation from
 // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;

    // Bit count function got from:
    // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetKernighan
    // This implementation assumes that a.cols (CV_8U) % sizeof(uint64_t) == 0

    //const uint64_t *pa, *pb;
    //pa = a.ptr<uint64_t>(); // a & b are actually CV_8U
    //pb = b.ptr<uint64_t>();

    //uint64_t v, ret = 0;
    //for(size_t i = 0; i < a.cols / sizeof(uint64_t); ++i, ++pa, ++pb)
    //{
    //    v = *pa ^ *pb;
    //    v = v - ((v >> 1) & (uint64_t)~(uint64_t)0/3);
    //    v = (v & (uint64_t)~(uint64_t)0/15*3) + ((v >> 2) &
    //            (uint64_t)~(uint64_t)0/15*3);
    //    v = (v + (v >> 4)) & (uint64_t)~(uint64_t)0/255*15;
    //    ret += (uint64_t)(v * ((uint64_t)~(uint64_t)0/255)) >>
    //        (sizeof(uint64_t) - 1) * CHAR_BIT;
    //}

    //return ret;
}

int main( int argc, char** argv ) {

    if( argc != 3 ) {
        help(argv);
        return -1;
    }

    // Load images
    Mat imgA = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE );
    if( !imgA.data ) {
        std::cout<< " --(!) Error reading image " << argv[1] << std::endl;
        return -1;
    }

    Mat imgB = imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE );
    if( !imgB.data ) {
        std::cout << " --(!) Error reading image " << argv[2] << std::endl;
        return -1;
    }

    BFMatcher matcher(NORM_HAMMING, false);

    std::vector<KeyPoint> keypointsA, keypointsB;
    Mat descriptorsA, descriptorsB;

    //// clatch
    //printf("clatch\n");
    //CLATCHextractor clatch_extractor(1000, 1.2, 8, 20, 7);
    //clatch_extractor(imgA, cv::Mat(), keypointsA, descriptorsA);
    //clatch_extractor(imgB, cv::Mat(), keypointsB, descriptorsB);
    //// Compute BoW
    //vector<cv::Mat> DescA= toDescriptorVector(descriptorsA);
    //vector<cv::Mat> DescB= toDescriptorVector(descriptorsB);
    //CLATCHVocabulary CLATCHVoc("small_clatch_voc.yml.gz");
    //DBoW2::BowVector BowVecA, BowVecB;
    //DBoW2::FeatureVector FeatVecA, FeatVecB;
    //CLATCHVoc.transform(DescA,BowVecA,FeatVecA,4);
    //CLATCHVoc.transform(DescB,BowVecB,FeatVecB,4);
    //double score = CLATCHVoc.score(BowVecA, BowVecB);

    // orb
    printf("orb\n");
    ORBextractor orb_extractor(1000, 1.2, 8, 20, 7);
    orb_extractor(imgA, cv::Mat(), keypointsA, descriptorsA);
    orb_extractor(imgB, cv::Mat(), keypointsB, descriptorsB);
    // Compute BoW
    vector<cv::Mat> DescA= toDescriptorVector(descriptorsA);
    vector<cv::Mat> DescB= toDescriptorVector(descriptorsB);
    ORBVocabulary ORBVoc;
    ORBVoc.loadFromBinaryFile("ORBvoc.bin");
    DBoW2::BowVector BowVecA, BowVecB;
    DBoW2::FeatureVector FeatVecA, FeatVecB;
    ORBVoc.transform(DescA,BowVecA,FeatVecA,4);
    ORBVoc.transform(DescB,BowVecB,FeatVecB,4);
    double score = ORBVoc.score(BowVecA, BowVecB);

    printf("BoW Feature vector score: %f \n", score);

    // Iterator
    DBoW2::FeatureVector::const_iterator itA = FeatVecA.begin();
    DBoW2::FeatureVector::const_iterator itB = FeatVecB.begin();
    DBoW2::FeatureVector::const_iterator Aend = FeatVecA.end();
    DBoW2::FeatureVector::const_iterator Bend = FeatVecB.end();

    vector<DMatch> vocmatches;
    while (itA != Aend && itB != Bend) {
        if (itA->first==itB->first) {
            const vector<unsigned int> IndicesA = itA->second;
            const vector<unsigned int> IndicesB = itB->second;
            for (int i = 0; i < (int)IndicesA.size(); ++i) {
                int idxA = IndicesA[i];
                const cv::Mat &DA = descriptorsA.row(idxA);
                printf("\nFeature in A : %d \n", idxA);
                int bestDist1 = 256;
                int bestDist2 = 256;
                int tId = -1;
                for (int j = 0; j < (int)IndicesB.size(); ++j) {
                    int idxB = IndicesB[j];
                    const cv::Mat &DB = descriptorsB.row(idxB);
                    printf("\nFeature in B : %d ", idxB);
                    const int dist =  DescriptorDistance(DA,DB);
                    printf(",distance : %d \n", dist);
                    if (dist < bestDist1) {
                        bestDist2 = bestDist1;
                        bestDist1 = dist;
                        tId = idxB;
                    } else if (dist < bestDist2) {
                        bestDist2 = dist;
                    }
                }
                if(bestDist1<=TH_LOW && bestDist1 < RATIO*bestDist2)
                    vocmatches.push_back(DMatch(idxA, tId, bestDist1));
            }
            itA++;
            itB++;
        } else if (itA->first < itB->first) {
            itA = FeatVecA.lower_bound(itB->first);
        } else {
            itB = FeatVecB.lower_bound(itA->first);
        }
    }

    vector<DMatch> clatchmatches;
    matcher.match(descriptorsA, descriptorsB, clatchmatches);
    //3.filter matchs
    double max_dist = 0, min_dist = 500;
    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0, _end = (int)clatchmatches.size(); i < _end; i++ )
    { 
        double dist = clatchmatches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
    printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );
    std::vector< DMatch > good_clatchmatches;
    for( int i = 0, _end = (int)clatchmatches.size(); i < _end; i++ )
    { 
        int qId = clatchmatches[i].queryIdx;
        int tId = clatchmatches[i].trainIdx;
        int dist = DescriptorDistance(descriptorsA.row(qId), descriptorsB.row(tId));
        printf("Compare BFMatch dist : %f and Custom hamm : %d\n", clatchmatches[i].distance, dist);
        if( clatchmatches[i].distance < 0.6*max_dist )
        { 
            good_clatchmatches.push_back( clatchmatches[i]); 
        }
    }
    printf("-- Match num : %d \n", (int)good_clatchmatches.size());
    Mat  clatchimgMatch,vocimgMatch;
    drawMatches(imgA, keypointsA, imgB, keypointsB, good_clatchmatches, clatchimgMatch);
    drawMatches(imgA, keypointsA, imgB, keypointsB, vocmatches, vocimgMatch);

    imwrite("clatchmatchs.png", clatchimgMatch);
    imwrite("vocmatchs.png", vocimgMatch);
    waitKey(0);
}
