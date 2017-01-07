//==============================================================================
//函数名： StateExecutor类的实现部分以及相关继承类的实现
//作者：王华威
//日期:  2015-7-10
//功能: 用于状态转换时执行相应动作
//修改记录：
//==============================================================================
#include <ros/ros.h>
#include "my_events.hpp"
#include <geometry_msgs/Twist.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "autotrack.hpp"
#include <findPoint.h>

//baofei ----test ---
//test----baofei
#include <sys/time.h>
extern struct timeval now_time;

extern int temp_count;


bool colorThresh(cv::Mat& in, cv::Mat& out, cv::Mat& mask, int high_threshold, int low_threshold, bool flag_reverse=0);

namespace autotrack {
//static int R_SIZE=200;
//static int B_SIZE=1500;
//StateExecutor

StateExecutor::StateExecutor() 
{
	ptset_line.clear();
	ptset_QR.clear();
	
	//Hellen add 160122
	R_SIZE = 200;
	B_SIZE = 1500;
	bBlueClrThr_Low = 150;
	bBlueClrThr_High = 200;
	velocity_x = 0.1;
	
	ros::param::get("AT_R_SIZE", R_SIZE);
	ros::param::get("AT_B_SIZE", B_SIZE);	
	ros::param::get("AT_BlueClrThr_Low", bBlueClrThr_Low);	
	ros::param::get("AT_BlueClrThr_High", bBlueClrThr_High);
	ros::param::get("AP_LINEAR_X", velocity_x);	
					
	std::cout << "StateExecutor   AT_R_SIZE" << R_SIZE<< "	AT_B_SIZE"<<B_SIZE<<std::endl;	
	std::cout << "StateExecutor   AT_BlueClrThr_Low" << bBlueClrThr_Low<< "  AT_BlueClrThr_High"<<bBlueClrThr_High<<std::endl;		

};

geometry_msgs::Twist StateExecutor::getTwist() {
	return twist_msg;
}
void StateExecutor::getPointset(const cv::Mat& in,
		std::vector<cv::Point>& ptset) {
	if (in.empty() || in.channels() != 1)
	{
		std::cout<<"getPointset图片空"<<std::endl;
		return ;
	}

	ptset.clear();
	cv::Point pt;
	for (int i = 0; i < in.rows; i++) {
		const uchar* dataI = in.ptr<uchar>(i);
		for (int j = 0; j < in.cols; j++) {
			if (dataI[j] == 0)
				continue;
			else {
				pt.y = i;
				pt.x = j;
				ptset.push_back(pt);
			} //if

		} //for2 loop
	} //for1 loop
} ///getPointset();待优化：分类记录点或者去除孤立区域

bool StateExecutor::processImg(const cv::Mat& src) {
	if (src.empty())
	{
		std::cout<<"processImg图片空"<<std::endl;
		return -1;
	}

    ////RGB转HSV
	cv::Mat hsv;
	cv::cvtColor(src, hsv, CV_BGR2HSV_FULL);
	std::vector<cv::Mat> hsv_planes;
	split(hsv, hsv_planes);
	cv::Mat h_ = hsv_planes[0];
	cv::Mat s_ = hsv_planes[1];
	cv::Mat h_QR, h_line;

	colorThresh(h_, h_line, s_, 240, 15, 1);

    //Hellen del 20160923: 这里高斯滤波会引入额外的蓝色噪点，导致蓝色检测点增多；
    //cv::GaussianBlur(h_, h_, cv::Size(7, 7), 1);

	colorThresh(h_, h_QR, s_, bBlueClrThr_Low, bBlueClrThr_High, 0); //进行颜色滤波对蓝色，检查结果Mat h_QR  old para:145,175
	; //进行颜色滤波对红色，检查结果Mat h_line
	
	//baofei added
	cv::Mat h_line_new(h_line.rows,h_line.cols,CV_8U,Scalar(0));
	
	#if 0
	ros::param::get("IMG_SHOW", bImgShow);	
	if(bImgShow !=0)				
	{	
		cv::imshow("R3",h_line_new);
	}
	#endif
	

	//baofei test -----
    #if 0
	if (temp_count == 0)
	{
		gettimeofday(&now_time,NULL);
		long double _time = (long double)now_time.tv_sec * 1000 + now_time.tv_usec / 1000;
		std::cout<<"now time before select line:"<<std::fixed<< _time <<std::endl;
	}
    #endif

	//Baofei del 160923 这个函数处理很耗时，会占用CPU，导致小车行走抖动；
    //SelectBestRedLine(h_line,h_line_new);
	
	#if 0
	cv::Mat_<uchar> ker_=(cv::Mat_<uchar>(3,3)<<1,1,1,1,1,1,1,1,1);
	cv::morphologyEx(h_line,h_line,CV_MOP_OPEN,ker_);
	cv::morphologyEx(h_QR,h_QR,CV_MOP_OPEN,ker_);
	#endif
	

	#if 0
	//baofei test -----
	if (temp_count == 0)
	{
		gettimeofday(&now_time,NULL);
		long double _time = (long double)now_time.tv_sec * 1000 + now_time.tv_usec / 1000;
		std::cout<<"now time after select line:"<<std::fixed<< _time <<std::endl;
	}
    #endif

	getPointset(h_QR, ptset_QR);
//	getPointset(h_line_new, ptset_line);
	getPointset(h_line, ptset_line);


	#if 0
	//baofei test -----
	if (temp_count == 0)
	{
		gettimeofday(&now_time,NULL);
		long double _time = (long double)now_time.tv_sec * 1000 + now_time.tv_usec / 1000;
		std::cout<<"now time after getPointset:"<<std::fixed<< _time <<std::endl;
	}
	#endif

	#if 0	
	if(bImgShow !=0)				
	{	
		cv::imshow("R1",h_line);
		cv::imshow("R2",h_line_new);
		//	cv::imshow("B",h_QR);
	}

	cv::waitKey(30);
	#endif

    #if 0
    if ( imgcount1 < 10)
    {
		if(ptset_QR.size() > 1000 && ptset_line.size() < 10)
		{
		    time_t now_time;	
			now_time=time(NULL);
			string ss=ctime(&now_time);
			cv::imwrite("AT_IMG_PROC_ERR1"+ss+".png",src);

			cv::imwrite("AT_IMG_PROC_HQR_ERR1"+ss+".png",h_QR);
			cv::imwrite("AT_IMG_PROC_NEW_ERR1"+ss+".png",h_line_new);
			cv::imwrite("AT_IMG_PROC_LINE_ERR1"+ss+".png",h_line);
            imgcount1++;
		}
    }

    if ( imgcount2 < 10)
    {
		if(ptset_QR.size() > 1500 && ptset_line.size() > 6000)
		{
		    time_t now_time;	
			now_time=time(NULL);
			string ss=ctime(&now_time);
			cv::imwrite("AT_IMG_PROC_ERR2"+ss+".png",src);
            imgcount2++;
		}
    }
	#endif

	return 1;
}

//baofei added
bool StateExecutor::SelectBestRedLine(cv::Mat h_line,cv::Mat &h_line_new)
{
	vector<vector<cv::Point> > contours;
	vector<Vec4i> hierarchy;
	findContours( h_line, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );
	
	//第2步：删除掉小块红色,同时计算是否存在下面的连通域和上面的连通域
	int count=0; //用来统计有效连通域的数量
	int bottom_index = -1; //底部连通域的index
	int upper_index = -1; //顶部连通域的index
	int optical_index = 0;

	for (int i=0; i<contours.size();i++)
	{
		float area = contourArea(contours[i]);
		if (area>1000)  //面积阈值 - 待调整
		{
			count ++;
			Rect rect = boundingRect(contours[i]);
			int bottom = rect.y+rect.height;
			int left_edge = rect.x;
			int right_edge = rect.x+rect.width;
			//如果靠近底部且不靠2边
			if ( (bottom > (h_line.rows - 10)) && (left_edge > (h_line.cols/8) ) && (right_edge < (h_line.cols *7/8) ) ) //320像素是宽度
			{
				bottom_index = i;
				break;
			}
			int upper = rect.y;
			
			//如果靠近顶部且不靠2边
			if ( (rect.y < 10)  && (left_edge > (h_line.cols/8) ) && (right_edge < (h_line.cols *7/8) ) )  //320像素是宽度
			{
				upper_index = i;
				
			}
			
		}	
	}			

	//如果有底部有连通域
	if (bottom_index != -1)
	{
		optical_index = bottom_index;
	}
	else if (upper_index!=-1)
	{
		optical_index = upper_index;
	}
	else
	{
		//error
	}
	
	std::vector<cv::Point> line_pointset;
  		for(int i=0;i<h_line.rows;i++)
   		for(int z=0;z<h_line.cols;z++)
   	{
   		Point2f p(z,i);//ccccccccccccc
   		double flag=cv::pointPolygonTest(contours[optical_index],p,false);
   		if(flag>=0)
   			line_pointset.push_back(p);
   	}

	//画图
	for (int i=1; i<line_pointset.size();i++)
	{
		h_line_new.at<uchar>(line_pointset[i])=255;
	}


}


bool StateReady::onInit() {
	std::cout<<"___________________________now Robot is Ready "<<std::endl;
	setTwist();
	return true;
}
;
void StateReady::stateHandle(const cv::Mat& src, AutoTrackEvent& event) {
//判断图像中是否存在轨迹线或者二维码
	setTwist();
	event.flag_stateswitch = 0;
	if (src.empty()) {
		return;
	}

	processImg(src);
	std::cout<<"QR :"<< ptset_QR.size()<<"   line: "<<ptset_line.size()<<std::endl;
	///图像中二维码与轨迹线都没有发现，小于500时，保持ready状态，置位事件完成标志
	if (ptset_QR.size() < B_SIZE && ptset_line.size() < R_SIZE) {
		event.next_state = AutoTrackEvent::ready;
		event.report_=1;
		setTwist();
	} else if (R_SIZE < ptset_line.size() && ptset_QR.size() < B_SIZE) { //发现轨迹线色域，未发现QR色域进入ap态
		event.next_state = AutoTrackEvent::autotrack;
		event.report_=0;
	} else {//发现二维码的
		if (event.flag_highspeed == 0) {//常速模式下，停止

			
			cv::Mat src2 = src;
		    //cv::Point2d qrPoint;	
		    cv::Point qrPoint;
			
			//bool flag = findPoint(src2, qrPoint);

			bool flag = findROICenter(src2, qrPoint);				
			
			if(flag)  //发现QR色域,停止
			{
				std::cout<<"findROICenter!!!!!!!!!!!!!!!!!!!!  qrPoint.x: "<<qrPoint.x<<"qrPoint.y: "<<qrPoint.y<<std::endl;
				event.next_state = AutoTrackEvent::ready;
				event.report_ = 1;
				//setTwist();
			}
			else
			{
				std::cout<<"can't find qr points!!!!!!!!!!!!!!!!!!!"<<std::endl;
				if (R_SIZE < ptset_line.size())    //有轨迹线色域,继续巡线
				{
					event.next_state = AutoTrackEvent::autotrack;
				}
				else
				{
					event.next_state = AutoTrackEvent::ready;
					event.report_ = 1;
					setTwist();					
				}
			}				
		}
		else
		{
			//std::cout<< "111111111111111111111111111"<<std::endl;
			event.next_state = AutoTrackEvent::autotrackhigh;
			event.report_=0;
		}
	}

	event.flag_stateswitch = 1;
}

bool StateAT::onInit(){
	std::cout<<"___________________________now Robot is  autoTracking Line"<<std::endl;
	return true;
};
void StateAT::stateHandle(const cv::Mat& src,AutoTrackEvent& event){
	if (src.empty()) {
		std::cout << "StateAutoPatrol stateHandle picture is empty"
				<< std::endl;
		return;
	}
	processImg(src);
	std::cout<<"QR :"<< ptset_QR.size()<<"   line: "<<ptset_line.size()<<std::endl;
	if (event.flag_highspeed == 1) {//if 切换高速状态
		event.next_state=AutoTrackEvent::autotrackhigh;
		event.flag_stateswitch = 1;
		curve_counts=5; //15
		return;
	}

	if (ptset_QR.size() < B_SIZE && ptset_line.size() < R_SIZE) {
		if (ptset_pre.size() > 1000) { //进入ap转弯等待
			setTwist(calcObjTwist(ptset_line, src.cols, src.rows));
		} else {
			event.next_state = AutoTrackEvent::ready;
			event.report_ = 3;
			event.flag_stateswitch = 1;
			curve_counts=5; //15
			setTwist();
		}
	} else if (R_SIZE < ptset_line.size() && ptset_QR.size() < B_SIZE) { //发现轨迹线色域，未发现QR色域进入ap态
		event.next_state = AutoTrackEvent::autotrack;
		setTwist(calcObjTwist(ptset_line,src.cols,src.rows));
		event.report_=0;
	} else 
	{//发现二维码的
		////////////////////////////////////////////////////
        cv::Mat src2 = src;
		cv::Point2f cen_(src2.cols / 2, src2.rows / 2);	
		cv::Point qrPoint;
		//cv::Point2d qrPoint;
		
		//bool flag = findPoint(src2, qrPoint);
		bool flag = findROICenter(src2, qrPoint);
		
		if(flag)  //发现QR色域,停止
		{
			std::cout<<"findROICenter  Point!!!!!!!!!!!!!!!!!!!! qrPoint.y: "<<qrPoint.y<<"   qrPoint.x:"<<qrPoint.x<<" cen_.x:"<<cen_.x<<"  cen_.y: "<<cen_.y<<std::endl;

         	cv::Point tmpPoint;
            tmpPoint.x = qrPoint.x;
            tmpPoint.y = qrPoint.y;
            
	        cv::Mat showimg;
			showimg = src2;
			showPoint(showimg, tmpPoint);

            if (qrPoint.y + 20 > cen_.y)
            {
				adjust_counts=200;
				event.next_state = AutoTrackEvent::ready;
				event.report_ = 1;   //finish			
				event.flag_stateswitch = 1;
			    curve_counts=5; //15				
				setTwist();	

                //Hellen added for test 160317 
                #if 0
			    time_t now_time;
		        now_time=time(NULL);
		        string ss=ctime(&now_time);		
		        cv::imwrite("AP_findpoint"+ss+".png",showimg);		
	            std::cout<<"find qr Point and save the image!!!! , file name =  AP_findpoint"<<ss<<".png"<<std::endl;	
                #endif
				
			    return;			
			} 
			else 
			{
				if(adjust_counts!=0)
				{
					setTwist(velocity_x*0.6);
					event.flag_stateswitch = 0;
					adjust_counts--;
					std::cout<<" aroundCenter not set the accurate loacation!!!  adjust_counts="<<adjust_counts<<std::endl;				
				}
				else
				{
					std::cout<<"aroundCenter end!!!!!!!!!!  adjust_counts:"<<adjust_counts<<std::endl;
					adjust_counts = 200;  						
					event.next_state = AutoTrackEvent::ready;
					event.report_ = 1;   //finish		
					event.flag_stateswitch = 1;
					curve_counts=5; //15				
					setTwist();	

                    //Hellen added for test 160317 
                    #if 0                    
			    	time_t now_time;
		        	now_time=time(NULL);
		        	string ss=ctime(&now_time);		
		        	cv::imwrite("AP_findpoint"+ss+".png",src2);		
	            	std::cout<<"find qr Point and save the image!!!! , file name =  AP_findpoint"<<ss<<".png"<<std::endl;	
					#endif

					return;	
					
				}	
				std::cout<<" pppppppppppppppppppp event.next_state = "<<event.next_state<<std::endl; 			
			} 
		}
		else
		{
			std::cout<<"can't find qr points!!!!!!!!!!!!!!!!!!!"<<std::endl;
			if (R_SIZE < ptset_line.size())    //有轨迹线色域,继续巡线
			{
                geometry_msgs::Twist tmp=calcObjTwist(ptset_line,src.cols,src.rows);
				tmp.linear.x*=0.6;
				tmp.angular.z*=0.6;
		        setTwist(tmp);
		        event.flag_stateswitch = 0;
                event.next_state = AutoTrackEvent::autotrack;				
		        return;
			}
			else
			{
				event.next_state = AutoTrackEvent::ready;
				event.report_ = 1;
				setTwist();	
		        event.flag_stateswitch = 1;	
				curve_counts=5; //15
              
                //Hellen added for test 160317                
		     	time_t now_time;
		       	now_time=time(NULL);
		       	string ss=ctime(&now_time);		
		       	cv::imwrite("AP_findpoint_ERR"+ss+".png",src2);		
	           	std::cout<<"NOT  find qr Point and save the image!!!! , file name =  AP_findpoint_ERR"<<ss<<".png"<<std::endl;	
    			
			}
		}	
			
	}



};

geometry_msgs::Twist StateAT::calcObjTwist(std::vector<cv::Point> ptset,
		const int& width, const int& height) {
	//判断输入的目标像素点个数，将其同上一帧做比较
		 //如果像素点锐减，有可能为直角转弯时发生,进入转弯倒计数，返回上一帧得到的转速；
		//否则更新ptset_world_pre;

	if (curve_counts==0 || float(ptset.size()) *1.7 >= float(ptset_pre.size())) {//如果转弯计数减至0或当前帧路径点足够多，
		//则清空前一帧路径点，否则循环递减转弯计数。
		ptset_pre.clear();
		ptset_pre = ptset; //保存当前帧获得的世界坐标
		curve_counts=5;  //15
	}
	else{
		curve_counts--;
		return t_pre;
	}

	geometry_msgs::Twist t;
	t.angular.x = 0;
	t.angular.y = 0;
	t.angular.z = 0;
	t.linear.x = 0;
	t.linear.y = 0;
	t.linear.z = 0;
	float K1=0.4;float K2=0.005;
	float obj_t=0.0;

	//根据传入的ptset，获取头行点和尾行点的中点
	getObjPoint(ptset,height/2);
	std::cout<<"mid_point.x"<<mid_point.x <<"    mid_point.y"<<mid_point.y<<std::endl;
	obj_t+=getDiffangle( cv::Point(width/2,height) )*K1;
	obj_t+=getDiffdist( cv::Point(width/2,height/2) )*K2;
	//将获得的合计偏差值，加入PD闭环控制其中，计算输出角速度
	obj_t=myPID(obj_t);
	float max_min = 0.3;  //0.7
	if (obj_t > max_min)
		obj_t = max_min;
	if (obj_t < -max_min)
		obj_t = -max_min;
	t.angular.z = obj_t;
	t.linear.x = 0.1;
	t_pre=t;
	return t;
}
;
float StateAT::myPID(float cur_,float obj_){
	float result=0.0;
	pid_.set_point=obj_;
	float derror_,error_;
	error_=pid_.set_point-cur_;//p_
	pid_.sum_error+=error_;//i_
	derror_=error_-pid_.last_error;
	pid_.pre_error=pid_.last_error;
	pid_.last_error=error_;

	result=pid_.p_*error_+
			pid_.d_*derror_;

	return result;

};
float StateAT::getDiffangle(cv::Point cen_){
	float result=0.0;
	float delta_x=mid_point.x-cen_.x;
	float delta_y=mid_point.y-cen_.y;
	delta_y= abs(delta_y);

	if (delta_y <= 1) {
		if (delta_x > 1)
			result = 1.57;
		if (delta_x < -1)
			result = -1.57;
		if (abs(delta_x) <= 1)
			result = 0;
	} else
		result = atan((delta_x) / (delta_y));
	std::cout<<"angle: "<<result<<std::endl;
	return result;
};
float StateAT::getDiffdist(cv::Point cen_) {
	float result = 0.0;
	result = mid_point.x - cen_.x;
	std::cout<<"dist: "<<result<<std::endl;

	return result;
}
;
void StateAT::getObjPoint(std::vector<cv::Point>& ptset,const int& obj_,int eps){
	if(ptset.empty())
		return;
	std::vector<cv::Point>::iterator iter;
	int count_=0;
	float accum_x=0.0;
	float accum_y=0.0;
	cv::Point temp_pt=ptset[0];
	float min_diff=abs(temp_pt.y-obj_);
	for( iter=ptset.begin()+1;iter!=ptset.end()-1;iter++)
	{
		float diffy=abs(iter->y-obj_);
		if (diffy < eps) {
			accum_y += iter->y;
			accum_x += iter->x;
			count_++;
		}
		if(diffy<min_diff)
		{
			temp_pt=*iter;
			min_diff=diffy;
		}
	}

	if(count_<=1)
		mid_point=temp_pt;
	else {
		mid_point.x = accum_x / count_;
		mid_point.y = accum_y / count_;
	}
	
};

///////////////////////////////////////////////////////////////////////////////////////////////

bool StateAThigh::onInit(){
	std::cout<<"___________________________highspeed!!!  autoTracking Line "<<std::endl;
	return true;
};

void StateAThigh::stateHandle(const cv::Mat& src,AutoTrackEvent& event){
	if (src.empty()) {
		std::cout << "StateAutoPatrol stateHandle picture is empty"
				<< std::endl;
		return;
	}
	processImg(src);
	std::cout<<"QR :"<< ptset_QR.size()<<"   line: "<<ptset_line.size()<<std::endl;
	if (event.flag_highspeed == 0) {//if 切换常速状态
		event.next_state=AutoTrackEvent::autotrack;
		event.flag_stateswitch = 1;
		curve_counts=5;
		return;
	}

	if (ptset_QR.size() < B_SIZE && ptset_line.size() < R_SIZE) {
		if (ptset_pre.size() > 1000) { //进入ap转弯等待
			setTwist(calcObjTwist(ptset_line, src.cols, src.rows));
		} else {
			event.next_state = AutoTrackEvent::ready;
			event.report_ = 3;
			event.flag_stateswitch = 1;
			curve_counts=5;
			setTwist();
		}

	} else if (R_SIZE < ptset_line.size() && ptset_QR.size() < B_SIZE) { //发现轨迹线色域，未发现QR色域进入ap态
		event.next_state = AutoTrackEvent::autotrackhigh;
		setTwist(calcObjTwist(ptset_line,src.cols,src.rows));
		event.report_=0;
	} else {//发现二维码的
			//Hellen del 20161018 在高速经过二维码时不需要降速处理
			#if 0
            if( ptset_QR.size() > B_SIZE && ptset_QR.size() < 1000)
            {
                speedPara = 0.7; 
            }
            else if (ptset_QR.size() > 1000)
            {
                speedPara = 0.5; 
            }
            else
            {
                speedPara = 1.0; 
            }
			#endif
     
	        setTwist(0.2*speedPara, 0, 0, 0, 0, 0.0);    //If going through qr,go ahead.  Hellen add 160603;
			event.next_state = AutoTrackEvent::autotrackhigh;
			event.report_ = 0;
                
            #if 0
            if ( ptset_QR.size() > 1500)
            {
                time_t now_time;	
				now_time=time(NULL);
				string ss=ctime(&now_time);
				cv::imwrite("AT_HIGE_SPEED"+ss+".png",src);
            }
            #endif
	}


};

geometry_msgs::Twist StateAThigh::calcObjTwist(std::vector<cv::Point> ptset,
		const int& width, const int& height) {
	//判断输入的目标像素点个数，将其同上一帧做比较
		 //如果像素点锐减，有可能为直角转弯时发生,进入转弯倒计数，返回上一帧得到的转速；
		//否则更新ptset_world_pre;

	if (curve_counts==0 || float(ptset.size()) *1.7 >= float(ptset_pre.size())) {//如果转弯计数减至0或当前帧路径点足够多，
		//则清空前一帧路径点，否则循环递减转弯计数。
		
		std::cout<<"calcObjTwist 1111111: ptset.size(): "<<ptset.size()<<" ptset_pre.size(): "<<ptset_pre.size()<<std::endl;		
		ptset_pre.clear();
		ptset_pre = ptset; //保存当前帧获得的世界坐标
		curve_counts=5; //15
	}
	else{
		curve_counts--;
		return t_pre;
	}

	geometry_msgs::Twist t;
	t.angular.x = 0;
	t.angular.y = 0;
	t.angular.z = 0;
	t.linear.x = 0;
	t.linear.y = 0;
	t.linear.z = 0;
	float K1=0.7;float K2=0.006;
	float obj_t=0.0;

	//根据传入的ptset，获取obj点
	getObjPoint(ptset,height/2);
std::cout<<"mid_point.x"<<mid_point.x <<"    mid_point.y"<<mid_point.y<<std::endl;
	obj_t+=getDiffangle( cv::Point(width/2,height) )*K1;
	obj_t+=getDiffdist( cv::Point(width/2,height/2) )*K2;
	//将获得的合计偏差值，加入PD闭环控制其中，计算输出角速度
	obj_t=myPID(obj_t);
	float max_min = 0.4;  //0.7
	if (obj_t > max_min)
		obj_t = max_min;
	if (obj_t < -max_min)
		obj_t = -max_min;
	t.angular.z = obj_t;

	t.linear.x = velocity_x;

	t_pre=t;
	return t;
}
;
float StateAThigh::myPID(float cur_,float obj_){
	float result=0.0;
	pid_.set_point=obj_;
	float derror_,error_;
	error_=pid_.set_point-cur_;//p_
	pid_.sum_error+=error_;//i_
	derror_=error_-pid_.last_error;
	pid_.pre_error=pid_.last_error;
	pid_.last_error=error_;

	result=pid_.p_*error_+
			pid_.d_*derror_;

	return result;

};
float StateAThigh::getDiffangle(cv::Point cen_){
	float result=0.0;
	float delta_x=mid_point.x-cen_.x;
	float delta_y=mid_point.y-cen_.y;
	delta_y= abs(delta_y);

	if (delta_y <= 1) {
		if (delta_x > 1)
			result = 1.57;
		if (delta_x < -1)
			result = -1.57;
		if (abs(delta_x) <= 1)
			result = 0;
	} else
		result = atan((delta_x) / (delta_y));
	std::cout<<"angle: "<<result<<std::endl;

	return result;
};
float StateAThigh::getDiffdist(cv::Point cen_) {
	float result = 0.0;
	result = mid_point.x - cen_.x;
	std::cout<<"dist: "<<result<<std::endl;

	return result;
}
;
void StateAThigh::getObjPoint(std::vector<cv::Point>& ptset,const int& obj_,int eps){
	if(ptset.empty())
		return;
	std::vector<cv::Point>::iterator iter;
	int count_=0;
	float accum_x=0.0;
	float accum_y=0.0;
	cv::Point temp_pt=ptset[0];
	float min_diff=abs(temp_pt.y-obj_);
	for( iter=ptset.begin()+1;iter!=ptset.end()-1;iter++)
	{
		float diffy=abs(iter->y-obj_);
		if (diffy < eps) {
			accum_y += iter->y;
			accum_x += iter->x;
			count_++;
		}
		if(diffy<min_diff)
		{
			temp_pt=*iter;
			min_diff=diffy;
		}
	}

	if(count_<=1)
		mid_point=temp_pt;
	else {
		mid_point.x = accum_x / count_;
		mid_point.y = accum_y / count_;
	}
	std::cout<<"mid_point.x"<<mid_point.x <<"    mid_point.y"<<mid_point.y<<std::endl;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


} //namespace

