#include "aras_visual_servo_camera/aras_visual_servo_camera.h"
#include "math.h"

#define  PI 3.1415926535897932384626433832795028841971
VisualServoCamera::VisualServoCamera()
{
    image_transport::ImageTransport it(nh_);
    image_sub_ = it.subscribe("/usb_cam/image_raw", 1, &VisualServoCamera::imageCB,this);
    threshold_image_pub_ = it.advertise("/aras_visual_servo/camera/thresholded_image", 1);
    camera_data_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/aras_visual_servo/camera/data",1);
    // cv::namedWindow( "Orginal Image", CV_WINDOW_AUTOSIZE);
    // cv::namedWindow( "Thresholded Image1", CV_WINDOW_AUTOSIZE );
    
    // cv::moveWindow("Orginal Image",0,0);
    // cv::moveWindow("Thresholded Image1",0,320);
    // cvCreateTrackbar("LowH", "Thresholded Image1", &iLowH, 179); //Hue (0 - 179)  
    // cvCreateTrackbar("HighH", "Thresholded Image1", &iHighH, 179);  
    
    // cvCreateTrackbar("LowS", "Thresholded Image1", &iLowS, 255); //Saturation (0 - 255)  
    // cvCreateTrackbar("HighS", "Thresholded Image1", &iHighS, 255);  
    
    // cvCreateTrackbar("LowV", "Thresholded Image1", &iLowV, 255); //Value (0 - 255)  
    // cvCreateTrackbar("HighV", "Thresholded Image1", &iHighV, 255);  
}

void VisualServoCamera::imageCB(const sensor_msgs::ImageConstPtr &image_msg)
{
     float angle_NeedleNVein = 0;
     float angle_YPose = 0,  angle_XPose = 0;
     float YPose = 0 ,XPose = 0;
     float l = 166.949872716;
    try
    {
        color_image_ = cv_bridge::toCvShare(image_msg, "bgr8")->image;
        // cv::cvtColor(color_image_, grey_image_, CV_BGR2GRAY);
        // cv::threshold( grey_image_, threshold_image_, THRESHOLD_VALUE, MAX_BINARY_VALUE ,cv::THRESH_BINARY_INV );
        //图像处理部分
        Mat grayImage, result, result1, element, result2;
       //Mat img = imread("11.jpg");
        //imshow("原始图", img);
        //Mat roi = img;
        Mat roi = color_image_(Rect(240, 40, 140, 140));
        //Mat roi = img(Rect(240, 40, 300, 300));  //Philips采集图像裁剪的图像尺寸
        imshow("ROI", roi);
        
        cvtColor(roi, grayImage, CV_BGR2GRAY);     //灰度转换
        bilateralFilter(grayImage, result, 5,  5, 5 / 2);   //双边滤波，在去除噪声的同时，保留轮廓
        threshold(result, result, 0, 255, CV_THRESH_OTSU);   //大津法图像分割，分离背景信息
        imshow("after OTSU", result);
        element = getStructuringElement(MORPH_RECT, Size(2, 2));   //形态学运算中的开运算，在该项目中作用不明显
        morphologyEx(result, result1, MORPH_OPEN, element);
        imshow("after OPEN", result1);
        bilateralFilter(result1, result, 25,  25, 25 / 2); 
        //Canny(result1, result1, 3, 9, 3);
        //imshow("after Canny", result1);
               
        int v_center;      // v_center 取值范围 [3,9](mm)
        Mat threshold_output;
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;   /// 使用Threshold检测边缘  
        threshold(result1, threshold_output, 50, 255, THRESH_BINARY);  /// RETR_TREE RETR_EXTERNAL CV_CHAIN_APPROX_SIMPLE 找到轮廓  
        findContours(threshold_output, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));   /// 多边形逼近轮廓 + 获取矩形和圆形边界框  
        vector<vector<Point> > contours_poly(contours.size());
        vector<Rect> boundRect(contours.size());
        vector<Point2f>center(contours.size());
        vector<float>radius(contours.size());
        for (int i = 0; i < contours.size(); i++)
        {
            approxPolyDP(Mat(contours[i]), contours_poly[i], 9, true);
            //boundRect[i] = boundingRect(Mat(contours_poly[i]));
            minEnclosingCircle(contours_poly[i], center[i], radius[i]);
        }    /// 画多边形轮廓 + 包围的矩形框 + 圆形框  
        Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3);
        //for (int i = 0; i< contours.size(); i++)

        for (int i = 0; i< contours.size(); i++)
        {
            //  if(radius[i] > 26 && radius[i] < 30)
             {

                if(radius[i] > 1 && radius[i] < 20)
                {
                    Scalar color = Scalar(255);   //画圆形轮廓的颜色，由于在opencv中颜色信息的顺序为BGR，所以为蓝色

                    if((center[i].x> 42)&&(center[i] .x<117))
                    {
                        drawContours(drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
                    //rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
                        circle(drawing, center[i], (int)radius[i], color, 2, 8, 0);
                        
                        cout << "圆半径为："<<i<<"   "<<radius[i] << endl;
                        cout <<"圆心坐标为："<<i<<"   "<< center[i] << endl; 

                        cout <<"血管直径为 ："<<i<<"   "<< center[1]-center[0] <<"   4.69mm" <<endl; 

                        if(center[0].y>center[1].y)
                        {
                            int d= center.at(1).y-center.at(0).y;   
                            v_center=center.at(1).y-d/2;
                            cout <<"血管中心为 ："<<"   "<< v_center << endl;   
                            v_center=(v_center/52.0)*1000;
                            //v_center =700;
                            cout <<"血管中心到表皮的距离为 ："<<"   "<<  v_center<< endl;
                        }

                        //v_center =800;
                
                        angle_NeedleNVein = atan2((v_center)/100.0,11.5);
                        angle_XPose = PI/2-0.45588-0.32795-angle_NeedleNVein;

                        XPose = sqrt(pow(143.35-sqrt(l*l-pow(l*sin(angle_XPose),2)),2)+pow(l*sin(angle_XPose),2));
                        YPose = sqrt(132.25+pow((v_center/100.0),2));
                        cout << "电机1 位置 ："<<XPose*100-3688<< endl;
                        cout << "电机2 位置 ："<< YPose*100 +2100<< endl;
                        cout << " +++++++++++++++++++++++++++++++++++++++++++++++++++++++："<< endl;

                    }

  
                }
            //     cout << "圆半径为："<<radius[i] << endl;
            //     cout <<"圆心坐标为："<< center[i] << endl;
            }
            // Scalar color = Scalar(255);   //画圆形轮廓的颜色，由于在opencv中颜色信息的顺序为BGR，所以为蓝色
            // drawContours(drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
            // //rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
            // circle(drawing, center[i], (int)radius[i], color, 2, 8, 0);
            // if (radius[i] > 15)
            // {
            //     cout << "圆半径为："<<radius[i] << endl;
            //     cout <<"圆心坐标为："<< center[i] << endl;
            // }
        }   /// 显示在一个窗口  
        
        namedWindow("Contours", CV_WINDOW_AUTOSIZE);

        imshow("Contours", drawing);            
        
        //namedWindow("Contours", CV_WINDOW_AUTOSIZE);

        //imshow("Contours", drawing);


        //Mat pointsf;
        //Mat(contours[0]).convertTo(pointsf, CV_32F);
        //RotatedRect box = fitEllipse(pointsf);
        ////cout << box.center;
        imwrite("dst.jpg", drawing);
       
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_msg->encoding.c_str());
    }

}

bool VisualServoCamera::calculateKernel(const cv::Mat *image, float kernel[KERNEL_SIZE], cv::Mat &jacobian_mat , cv::Mat &jacobian_inverse_mat)
{
    //we should add mutex-lock
    if(jacobian_mat.rows !=KERNEL_SIZE && jacobian_mat.cols!=DOF)
    {
        ROS_ERROR("the jacobian matrix size is not %d x %d",KERNEL_SIZE,DOF);
        return false;
    }
    //just in case
    for(int i=0;i<KERNEL_SIZE;i++)
    {
        for(int j=0;j<DOF;j++)
        {
            jacobian_mat.at<double>(i,j)=0;
        }
    }

    jacobian_mat.at<double>(3,3)=-1;

    int img_height = image->rows;
    int img_width = image->cols;
    double jxx,jxtheta,jyy,jytheta;

    for(int i=0;i<KERNEL_SIZE;i++)
    {
        kernel[i] = 0;
    }

    double kx = 0;
    double ky = 0;

    for(int i=0;i<img_height;i++ )
    {
        kx = pow(((i+1)*DELTA_X),KERNEL_P);//k_x(x,y) = x^(-2);
        jxx =  KERNEL_P * pow(((i+1)*DELTA_X),(KERNEL_P-1));//dk_x(x,y)/dx = -2 * x^(-3)
        for(int j=0;j<img_width;j++)
        {
            double pixel_value = (double)image->at<uchar>(i,j)/MAX_BINARY_VALUE;

            ky = pow(((j+1)*DELTA_Y),KERNEL_Q);

            jyy = KERNEL_Q * pow(((j+1)*DELTA_Y),(KERNEL_Q-1));
            jxtheta = jxx*(j + 1)*DELTA_Y;
            jytheta = jyy*(i + 1)*DELTA_X;
            kernel[0] = kernel[0] + kx * pixel_value*DELTA_Y*DELTA_X;
            kernel[1] = kernel[1] + ky * pixel_value*DELTA_Y*DELTA_X;
            kernel[2] = kernel[2] + pixel_value*DELTA_Y*DELTA_X;

            jacobian_mat.at<double>(0,0) = jacobian_mat.at<double>(0,0) - jxx * pixel_value; //jacobianxx
            jacobian_mat.at<double>(0,3) = jacobian_mat.at<double>(0,3) + jxtheta * pixel_value;//jacobianxtheta
            jacobian_mat.at<double>(1,1) = jacobian_mat.at<double>(1,1) - jyy * pixel_value;//jacobianyy
            jacobian_mat.at<double>(1,3) = jacobian_mat.at<double>(1,3) - jytheta * pixel_value;//jacobianytheta
            jacobian_mat.at<double>(2,2) = jacobian_mat.at<double>(2,2) + 2 * pixel_value;//jacobianz

        }
    }

    cv::Moments image_moments = cv::moments(*image);
    kernel[3] = 0.5 * atan2( (2*image_moments.mu11/255.0) , (image_moments.mu20/255.0-image_moments.mu02/255.0) );
    cv::invert(jacobian_mat,jacobian_inverse_mat);
}

void VisualServoCamera::publishCameraData(const float kernel[KERNEL_SIZE] ,const cv::Mat jacobian_inverse_mat)
{
    std_msgs::Float64MultiArray camera_data_msg;
    for(int i=0;i<jacobian_inverse_mat.rows;i++)
    {
        std::cout<<"                                     "<<std::endl;
        for(int j=0;j<jacobian_inverse_mat.cols;j++)
        {
            camera_data_msg.data.push_back(jacobian_inverse_mat.at<double>(i,j));
            std::cout<<jacobian_inverse_mat.at<double>(i,j)<<std::endl;
 
        }
        
    }
    for(int i=0;i<KERNEL_SIZE;i++)
    {
        camera_data_msg.data.push_back(kernel[i]);
        std::cout<<kernel[i]<<std::endl;
    }
    camera_data_pub_.publish(camera_data_msg);
}

VisualServoCamera::~VisualServoCamera()
{

}
