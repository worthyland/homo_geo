#include <mavros_interaction/MavrosInteraction.h>
#include <homo_msgs/HomographyResult.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Eigen>



#include <opencv2/core/base.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp> 
using namespace Control;
using namespace std;
using namespace cv;
ros::Time totalBegin,totalEnd;



Quadrotor::state curUavState;
homo_msgs::HomographyResult homoMatrixFromPose,homoMatrixFromIMG;
Eigen::Vector3d posDesire;
Eigen::Vector3d n;
double d;
homo_msgs::HomographyResult 
HomographyCalculationFromPose(const Quadrotor::state& curState,Eigen::Vector3d& ref_posDesire_base,Eigen::Vector3d& n,double d)
{
    Eigen::Matrix3d enu_R_ned;
    enu_R_ned << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0;
    Eigen::Vector3d ref_pos_base(curState.pos);//

    Eigen::Vector3d ref_posError_base(ref_pos_base - ref_posDesire_base);
    Eigen::Vector3d refned_posError_basened(enu_R_ned*ref_posError_base);
    

    Eigen::Matrix3d ref_R_base = Eigen::Matrix3d::Identity();
    ref_R_base = curState.orientation.toRotationMatrix();
    // cout << "R:"<<enu_R_ned <<endl;
    Eigen::Matrix3d refned_R_basened = Eigen::Matrix3d::Identity();;
    refned_R_basened = enu_R_ned.transpose()*ref_R_base*enu_R_ned;
    // cout << "R2:"<<enu_R_ned <<endl;
     

    Eigen::Matrix3d refned_homoTmp_basened;
    refned_homoTmp_basened = refned_R_basened.transpose() - (refned_R_basened.transpose() * refned_posError_basened * n.transpose()) / d;
    cout << "--------------HomographyCalculationFromPose----------------"<< endl;
    cout << "refned_posError_basened:"<< refned_posError_basened << endl;
    cout << "refned_homoTmp_basened:" << endl;
    cout << refned_homoTmp_basened <<endl;

    homo_msgs::HomographyResult res;

    for(int i = 0;i<3;i++){
        for(int j = 0;j<3;j++){
                res.homography[i*3 + j] =refned_homoTmp_basened(i,j);
                
        }  
    }
    return res;
}



class HomographyPublish
{
private:
    /* data */
    ros::NodeHandle nh_,nhParam_;
    ros::Subscriber imageSub_;
    ros::Publisher  imagePub_;


    std::vector<cv::Point2f> cornersRef_, cornersCur_;
    int witdh_,hight_;//图片的大小
    cv::Mat HMat_;
    Eigen::Matrix3d HMatrix_;
    Eigen::Matrix3d cameraK_;

    cv::Mat curImg_,refImg_;

    homo_msgs::HomographyResult homoMatrixFromImg_;
public:
    HomographyPublish(const ros::NodeHandle& nh,const ros::NodeHandle& nhParam);
    ~HomographyPublish();
    void ImageCb(const sensor_msgs::ImageConstPtr& msg);
    Scalar randomColor(int64 seed);

    void operator() ();

};
HomographyPublish::HomographyPublish(const ros::NodeHandle& nh,const ros::NodeHandle& nhParam):nh_(nh),nhParam_(nhParam)
{
    nhParam_.param("witdh",witdh_,640);
    nhParam_.param("hight",hight_,480);
    imageSub_ = nh_.subscribe("/image_raw",10,&HomographyPublish::ImageCb,this);// /iris/usb_cam    /galaxy_camera
    imagePub_ = nh_.advertise<sensor_msgs::Image>("/image_draw",5,true);
}
HomographyPublish::~HomographyPublish()
{
}

void
HomographyPublish::ImageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cvPtr;
    try
    {
        cvPtr = cv_bridge::toCvCopy(msg);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    curImg_ = cvPtr->image; 

    double witdh_rate = witdh_/(double)curImg_.cols;
    double hight_rate = hight_/(double)curImg_.rows;
    // std::cout << "witdh_rate:"<<witdh_rate<<"\nhight_rate:"<<hight_rate<<std::endl;
    cv::resize(curImg_,curImg_,cv::Size(0,0),witdh_rate,hight_rate);
    // std::cout << "img_witdh:"<<curImg.cols<<"\nimhg_hight:"<<curImg.rows<<std::endl;

    bool foundCur = cv::findChessboardCorners(curImg_, cv::Size (4,3), cornersCur_);   
    std::cout << "foundCur:"<<foundCur<<std::endl;
    if(foundCur == true){
        HMat_ = cv::findHomography(cornersRef_, cornersCur_);
        // std::cout << "HMat_:\n" << HMat_ << std::endl;
        cv::cv2eigen(HMat_,HMatrix_);
        // HMatrix_(0,0) = HMat_.at<uchar>(0,0);HMatrix_(0,1) = HMat_.at<uchar>(0,1);HMatrix_(0,2) = HMat_.at<uchar>(0,2);
        // HMatrix_(1,0) = HMat_.at<uchar>(1,0);HMatrix_(1,1) = HMat_.at<uchar>(1,1);HMatrix_(1,2) = HMat_.at<uchar>(1,2);
        // HMatrix_(2,0) = HMat_.at<uchar>(2,0);HMatrix_(2,1) = HMat_.at<uchar>(2,1);HMatrix_(2,2) = HMat_.at<uchar>(2,2);  
        std::cout << "HMatrix_:\n" << HMatrix_ << std::endl;
 HMatrix_ = cameraK_.transpose() * HMatrix_ * cameraK_;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(HMatrix_,Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd C(3,1);
        C = svd.singularValues();
        std::cout << "C:\n" << C << std::endl;
        HMatrix_ = HMatrix_/C(1,0);    

        Mat img_draw_matches,ref_img_rgb8,cur_img_rgb8;
        cv::cvtColor(refImg_,ref_img_rgb8,cv::COLOR_GRAY2RGB);
        cv::cvtColor(curImg_,cur_img_rgb8,cv::COLOR_GRAY2RGB);
        hconcat(ref_img_rgb8, cur_img_rgb8, img_draw_matches);
        for (size_t i = 0; i < cornersRef_.size(); i++)
        {
            Mat pt1 = (Mat_<double>(3,1) << cornersRef_[i].x, cornersRef_[i].y, 1);
            Mat pt2 = HMat_ * pt1;
            pt2 /= pt2.at<double>(2);
            Point end( (int) (refImg_.cols + pt2.at<double>(0)), (int) pt2.at<double>(1) );
            line(img_draw_matches, cornersRef_[i], end, randomColor(cv::getTickCount()), 1);
        }    
        cvPtr->encoding = sensor_msgs::image_encodings::RGB8;
        cvPtr->image = img_draw_matches;
        imagePub_.publish(cvPtr->toImageMsg());  
    }else{
        HMatrix_ = Eigen::Matrix3d::Identity();
    }
    
    for(int i = 0;i<3;i++){
        for(int j = 0;j<3;j++){
                homoMatrixFromImg_.homography[i*3 + j] =refned_homoTmp_basened(i,j);
                
        }  
    }

}


Scalar 
HomographyPublish::randomColor(int64 seed)
{
    cv::RNG rng(seed);
    int icolor = (unsigned)rng;
    return Scalar(icolor & 255, (icolor >> 8) & 255, (icolor >> 16) & 255);
}
int main(int argc,char *argv[])
{
    ros::init(argc, argv, "homography_publish_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhParam("~");//加“～”才能读取
    ros::Subscriber imageSub;
    
    
    ros::Rate rate(60.0);
    MavrosInteraction uavInfo(nh,nhParam);
    ros::Publisher resultsPub = nh.advertise<homo_msgs::HomographyResult>("homography_pose", 1);

    posDesire << 0,0,1;
    n << 0,0,1;

    //初始化posDesire
    nhParam.param("posDesire/x",posDesire(0),0.0);
    nhParam.param("posDesire/y",posDesire(1),0.0);
    nhParam.param("posDesire/z",posDesire(2),1.0);
    //初始化n
    nhParam.param("n/x",n(0),0.0);
    nhParam.param("n/y",n(1),0.0);
    nhParam.param("n/z",n(2),1.0);
    //初始化d
    nhParam.param("d",d,1.0);

    //发布选择变量 
    int pubChoose = 0;
    nhParam.param("pubChoose",pubChoose,0);
    while(ros::ok()){
        
        curUavState = uavInfo.GetState();
        //uavInfo.ShowUavState(5);
        homoMatrixFromPose = HomographyCalculationFromPose(curUavState,posDesire,n,d);
        if(pubChoose == 0){}
        else if(pubChoose == 1){
            resultsPub.publish(homoMatrixFromPose);
        }else if(pubChoose == 2){
            resultsPub.publish(homoMatrixFromIMG);
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}