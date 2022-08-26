#include <iostream>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include "opencv2/opencv.hpp"




namespace ORB
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


class ORBExtrackor {
    public:
        ORBExtrackor();

        // 從config_file.yaml中讀取攝影機與orb的參數
        void SetConfig(std::string config_file);
        // 設定金字塔每一層所需要的特徵點數量
        void SetPyramid();
        void GrabRGB(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);

        void GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);

        void TrackRGBD(cv::InputArray _image, std::vector<cv::KeyPoint>& _keypoints, cv::OutputArray _descriptors);

        // 計算金字塔每一層的像素大小, 並且放到mvImagePyramid裡面
        void ComputePyramid(cv::Mat image);

        void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
        // ORB特徵點均勻化（四元樹演算法）,並且計算出金字塔每一層均勻化後所需要的特徵點數量
        std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

        // 對特徵點進行畸變校正
        void UndistortKeyPoints();
        // 對特定的特徵點加入深度
        void ComputeStereoFromRGBD(const cv::Mat &imDepth);

        void DrawORB(cv::Mat im, std::vector<cv::KeyPoint> Keys, std::string windows);

        // use open_cv extract orb feature
        void testTrackRGBD(const cv::Mat &im);
        void testextracte_orb(cv::Mat input,std::vector<cv::KeyPoint> &keypoint,cv::Mat &descriptor);

        // camera patameters
        float fx;
        float fy;
        float cx;
        float cy;

        float k1;
        float k2;
        float p1;
        float p2;

        float fps;
        float mbf;
        int nRGB;
        float mThDepth;
        float mDepthMapFactor;

        // ORB parameters
        int nfeatures;    // 特徵點數量
        float scaleFactor;// 金字塔相鄰兩層的比例
        int nlevels;      // 金字塔的層數
        int fIniThFAST;   // fast特徵的默認閥值
        int fMinThFAST;   // fIniThFAST提取失敗時使用的最小閥值

        // Calibration matrix and OpenCV distortion parameters.
        cv::Mat mK;
        static float invfx;
        static float invfy;
        cv::Mat mDistCoef;

        // Number of KeyPoints.
        int N;

        // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
        // In the stereo case, mvKeysUn is redundant as images must be rectified.
        // In the RGB-D case, RGB images can be distorted.
        // mvKeys (特徵點), mvKeysUn (校正畸變後的特徵點)
        std::vector<cv::KeyPoint> mvKeys, mvKeysUn;

        cv::Mat mDescriptors;

        // Corresponding stereo coordinate and depth for each keypoint.
        // "Monocular" keypoints have a negative value.
        std::vector<float> mvuRight;
        std::vector<float> mvDepth;


    protected:
        std::vector<float> mvScaleFactor;     // 金字塔每一層對第一層的比例
        std::vector<float> mvInvScaleFactor;  // mvScaleFactor的倒數
        std::vector<float> mvLevelSigma2;     // mvScaleFactor的平方
        std::vector<float> mvInvLevelSigma2;  // mvLevelSigma2的倒數
        std::vector<cv::Mat> mvImagePyramid;  // 保存每一層圖像加上高斯模糊層的圖像
        std::vector<int> mnFeaturesPerLevel;  // 金字塔每一層所需要的特徵點數量
        std::vector<int> umax;

        std::vector<cv::Point> pattern;

        //Color order (true RGB, false BGR, ignored if grayscale)
        bool mbRGB;
};

} //namespace ORB