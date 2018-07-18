	#include <stdio.h>
	#include <iostream>
	#include "opencv2/core/core.hpp"//因为在属性中已经配置了opencv等目录，所以把其当成了本地目录一样
	#include "opencv2/features2d/features2d.hpp"
	#include "opencv2/highgui/highgui.hpp"
	#include "opencv2/nonfree/nonfree.hpp" 
	#include "windows.h"
	#include "opencv2/legacy/legacy.hpp"
	#include <fstream>
	#include <ios>
	#include <string>
	#include <comutil.h>
	#include <direct.h>  
	#include <io.h>  
	#include <windows.h>
	#include "ximage.h"


	#define min1(a,b)            (((a) < (b)) ? (a) : (b))
	#define max1(a,b)            (((a) > (b)) ? (a) : (b))


 

	int  downScale_qt = 4 ;//qt界面降采样比例设置，若未选择，则默认为不降采样
	string dir,dir1,dir2,dir3,dir4,dir5;
	string dir_p1,dir_p2;
	string dir_match_line;	//	两幅图之间画出配准连线
	string dir1_down,dir2_down;	//降采样图像保存路径
	double detectorThres1 = NULL;
	double detectorThres2 = NULL;
	double pointFilterThres = 0.52;
	double pyramidTime;

	string fileName_txt = "";//参数结果保存的txt文件名
	string combine_name = "";//组合算法结果名称

	//DS2()函数中会用到的一些参数变量等
	int detectorFlag=1;	//检测子的标志位
	int descriptorFlag=1;	//描述子的标志位

	int FeaturePairs=0;	//统计最后正确匹配的点对数
	double rmse=0.0;	//DS2函数中统计精度

	double t_sj,t_sm,t_rj,t_rm;	//检测子描述子时间
	double t_pt,t_choose,t_H;
	double t_bigline;//大图画框计时
	double t_down;//降采样计时
	double t_save;	//统计大图重构画框图的时间
	double rmse_combine =0.0;
	int pt_s,pt_r,pt_pairs;
	vector<CvMat*> H_2;

	double t_combine=0.0;
	double scale =0.25;
 


	ofstream outfile;	//将一些变量输出到txt文件
	vector <IplImage*> datu1;
	vector<string> lujing;

	//vector<Mat> H1;
	vector<CvMat*>H2;

	vector<int>ptpairs;
	vector<IplImage*> img;
	double t_ceshi;
	void Combine_Regiatration(string SrcSenImagePath,string SrcReImagePath,int DetectorFlag,int DescriptorFlag,int Threshold1/*,int Threshold2 */);


void Combine_RegiatrationMat(Mat uav,Mat ref,int DetectorFlag,int DescriptorFlag,int Threshold1/*,int Threshold2*/)
	{

		 t_ceshi = cvGetTickCount();

		cv::initModule_nonfree();
		//img向量，用来保存本次配准中的图像
	

		IplImage* ip_temp1 =&IplImage(uav);
		IplImage* img1 = cvCloneImage(ip_temp1);
		IplImage* ip_temp2 =&IplImage(ref);
		IplImage* img2 = cvCloneImage(ip_temp2);
		Ptr<DescriptorMatcher> descriptor_matcher = DescriptorMatcher::create("BruteForce"/*"FlannBased"*/);//创建特征匹配器

		//double t =(double)cvGetTickCount(); //统计配准总时间
	
		//特征点检测
		vector <KeyPoint> m_LeftKey,m_RightKey;		//特征点
		Mat descriptors1,descriptors2;							//描述子
		Mat img_m_LeftKey,img_m_RightKey;				//画出特征点的输出结果图像

		fileName_txt += Int_to_String(Threshold1);
		fileName_txt += "_";
		combine_name +=Int_to_String(Threshold1);
		combine_name += "_";
	
		double t3;
		t3 = cvGetTickCount();

		switch(DetectorFlag)
		{
		case 1:
			{

				SurfFeatureDetector detector(Threshold1);/*1
						SurfFeatureDetector detector2(Threshold1/*Threshold2)*/
						t_sj = cvGetTickCount();
						detector.detect(img1,m_LeftKey);
					
					//	t_sj = (double)(cvGetTickCount() - t_sj)/(cvGetTickFrequency()*1000.*1000.);
			
					//	t_rj = cvGetTickCount();
						detector.detect(img2,m_RightKey);
					//	t_rj = (double)(cvGetTickCount() - t_rj)/(cvGetTickFrequency()*1000.*1000.);
			
						t_sj = (double)(cvGetTickCount() - t_sj)/(cvGetTickFrequency()*1000.*1000.);
					
						fileName_txt += "SURF_";
						combine_name += "SURF_";
				break;
			}

		case 2:
			{
				FastFeatureDetector detector(Threshold1);


				t_sj = cvGetTickCount();
				detector.detect(img1,m_LeftKey);
				t_sj = (double)(cvGetTickCount() - t_sj)/(cvGetTickFrequency()*1000.*1000.);

				t_rj = cvGetTickCount();
				detector.detect(img2,m_RightKey);
				t_rj = (double)(cvGetTickCount() - t_rj)/(cvGetTickFrequency()*1000.*1000.);
				fileName_txt += "FAST_";
				combine_name += "FAST_";
				break;
			}
		}

		t3 = (double)(cvGetTickCount() - t3)/(cvGetTickFrequency()*1000.*1000.);
		cout<<"t3:"<<t3<<endl;

		double t4;
		t4 = cvGetTickCount();

		switch(DescriptorFlag)
		{
		case 1:
			{

				SurfFeatureDetector descriptor_extractor(Threshold1);

							 t_sm = cvGetTickCount();
				descriptor_extractor.compute(img1,m_LeftKey,descriptors1);
				//			

				//			 t_rm = cvGetTickCount();
				descriptor_extractor.compute(img2,m_RightKey,descriptors2);
				//			 t_rm = (double)(cvGetTickCount() - t_rm)/(cvGetTickFrequency()*1000.*1000.);
				//
				 t_sm = (double)(cvGetTickCount() - t_sm)/(cvGetTickFrequency()*1000.*1000.);
				fileName_txt += "SURF.txt";
				combine_name += "SURF";
				break;

	
			}

		case 2:
			{
				BRISK descriptor_extractor(Threshold1);

				t_sm = cvGetTickCount();
				descriptor_extractor.compute(img1,m_LeftKey,descriptors1);
			//	t_sm = (double)(cvGetTickCount() - t_sm)/(cvGetTickFrequency()*1000.*1000.);

			//	t_rm = cvGetTickCount();
				descriptor_extractor.compute(img2,m_RightKey,descriptors2);
			//	t_rm = (double)(cvGetTickCount() - t_rm)/(cvGetTickFrequency()*1000.*1000.);
				t_sm = (double)(cvGetTickCount() - t_sm)/(cvGetTickFrequency()*1000.*1000.);

				fileName_txt += "BRISK.txt";
				combine_name += "BRISK";
				break;
			}

		case 3:
			{
				//OrbFeatureDetector descriptor_extractor(Threshold1);
				ORB descriptor_extractor(Threshold1);

				t_sm = cvGetTickCount();
				descriptor_extractor.compute(img1,m_LeftKey,descriptors1);
				t_sm = (double)(cvGetTickCount() - t_sm)/(cvGetTickFrequency()*1000.*1000.);

				t_rm = cvGetTickCount();
				descriptor_extractor.compute(img2,m_RightKey,descriptors2);
				t_rm = (double)(cvGetTickCount() - t_rm)/(cvGetTickFrequency()*1000.*1000.);
				fileName_txt += "ORB.txt";
				combine_name += "ORB";
				break;
			}
		case 4:
			{
				FREAK descriptor_extractor;

				t_sm = cvGetTickCount();
				descriptor_extractor.compute(img1,m_LeftKey,descriptors1);
				t_sm = (double)(cvGetTickCount() - t_sm)/(cvGetTickFrequency()*1000.*1000.);

				t_rm = cvGetTickCount();
				descriptor_extractor.compute(img2,m_RightKey,descriptors2);
				t_rm = (double)(cvGetTickCount() - t_rm)/(cvGetTickFrequency()*1000.*1000.);
				fileName_txt += "FREAK.txt";
				combine_name += "FREAK";
				break;
			}

		case 5:
			{
				BriefDescriptorExtractor descriptor_extractor(32);
			
				t_sm = cvGetTickCount();
				descriptor_extractor.compute(img1,m_LeftKey,descriptors1);
				t_sm = (double)(cvGetTickCount() - t_sm)/(cvGetTickFrequency()*1000.*1000.);

				t_rm = cvGetTickCount();
				descriptor_extractor.compute(img2,m_RightKey,descriptors2);
				t_rm = (double)(cvGetTickCount() - t_rm)/(cvGetTickFrequency()*1000.*1000.);
				fileName_txt += "BRIEF.txt";
				combine_name += "BRIEF";
				break;
			}

		default:
			break;
		}

		t4 = (double)(cvGetTickCount() - t4)/(cvGetTickFrequency()*1000.*1000.);
		cout<<"t4:"<<t4<<endl;

		pt_s =m_LeftKey.size();
		pt_r =m_RightKey.size();

		//特征匹配
		vector<DMatch> matches;//匹配结果，DMatch是一个结构体
	
		t_pt = cvGetTickCount();
		descriptor_matcher->match(descriptors1,descriptors2,matches);//匹配两个图像的特征矩阵
		t_pt = (double)(cvGetTickCount() - t_pt)/(cvGetTickFrequency()*1000.*1000.);
	
		t_choose =cvGetTickCount();
		double max_dist = 0;
		double min_dist = 100;
		for(int i=0;i<matches.size();i++)
		{
			double dist = matches[i].distance;
			if(dist < min_dist) 
				min_dist = dist;
			if(dist > max_dist)
				max_dist = dist;
		}
	//设置阈值，筛选较好的匹配点  
		vector<DMatch> goodMatches;
	//	float xishu=0.38;
		for(int i=0;i<matches.size();i++)
		{
			if(matches[i].distance < (pointFilterThres *max_dist))	//这里的阈值设置也是比较重要的
		//	if( matches[i].distance <= max(2*min_dist, 0.02) )
			{
				goodMatches.push_back(matches[i]);
			}
		}

		pt_pairs = goodMatches.size();

		//RANSAC匹配过程
		vector<DMatch> m_Matches = goodMatches;
		//分配空间
		int ptCount = m_Matches.size();//int ptCount = (int)m_Matches.size()不用这句里的强制转换试试
		Mat p1(ptCount,2,CV_32F);//创建一个Mat结构，点数行，2列，32位浮点型
		Mat p2(ptCount,2,CV_32F);

		//把得到的Keypoint转换为Mat
		Point2f pt;
		for(int i=0;i<m_Matches.size();i++)
		{
			pt = m_LeftKey[m_Matches[i].queryIdx].pt;
			p1.at<float>(i,0) = pt.x;//第一列表示x坐标；第二列表示y；//迭代器，at函数用来读取矩阵中的某个像素
			p1.at<float>(i,1) = pt.y;
		
		
			pt = m_RightKey[m_Matches[i].trainIdx].pt;
			p2.at<float>(i,0) = pt.x;
			p2.at<float>(i,1) = pt.y;
		}

		//用RANSAC方法计算F
		Mat m_Fundamental;//计算基本矩阵
		vector<uchar> m_RANSACStatus;//该变量用来存储RANSAC之后，每个点的状态
		findFundamentalMat(p1,p2,m_RANSACStatus,FM_RANSAC);//最后一位是标志位，表示使用RANSAC算法

		//计算野点个数
		int OutlinerCount = 0;
		for(int i=0;i<m_Matches.size();i++)
		{
	
			if(m_RANSACStatus[i] == 0)			//状态为0表示是野点
			{
				OutlinerCount++;
			}
		}
		int InlinerCount = ptCount - OutlinerCount;
	//	cout<<"内点个数为："<<InlinerCount<<endl;
	
		//创建3个变量用来保存内点和匹配关系
		vector<Point2f> m_LeftInlier;
		vector<Point2f> m_RightInlier;
		vector<DMatch> m_InlierMatches;
		m_LeftInlier.resize(InlinerCount);		
		m_RightInlier.resize(InlinerCount  );
		m_InlierMatches.resize(InlinerCount );

		InlinerCount = 0;
		for(int i =0;i<ptCount;i++)
		{
		
			if(m_RANSACStatus[i] != 0)	//将正确匹配的点赋给内点
			{
				m_LeftInlier[InlinerCount].x = p1.at<float>(i,0);
				m_LeftInlier[InlinerCount].y = p1.at<float>(i,1);
				m_RightInlier[InlinerCount].x = p2.at<float>(i,0);
				m_RightInlier[InlinerCount].y = p2.at<float>(i,1);
				m_InlierMatches[InlinerCount].queryIdx = InlinerCount;
				m_InlierMatches[InlinerCount].trainIdx = InlinerCount;

				//if(m_RightInlier[InlinerCount].x < inlier_minRx)
				//	inlier_minRx = m_RightInlier[InlinerCount].x;		//存储内点中右图的最小坐标
				InlinerCount++;
			}
		}

	t_choose =(double)(cvGetTickCount() - t_choose)/(cvGetTickFrequency()*1000.*1000.);
	cout<<"t_choose:"<<t_choose<<endl;
	
		//矩阵H用来存储RANSAC变换得到的单应性矩阵

		/*计算H矩阵的时间*/
		t_H =(double)cvGetTickCount();
		Mat H_h= findHomography(m_LeftInlier,m_RightInlier,CV_RANSAC);	//使用内点求解H

		t_H =(double)(cvGetTickCount() - t_H)/(cvGetTickFrequency()*1000.*1000.);

		//求精度程序
	

		FeaturePairs = InlinerCount;

		double x_new,y_new;
		vector<Point2f> m_newRightInlier;


		for(int n=0;n<InlinerCount;n++)
		{
			/*x_new = m_LeftInlier[n].x*h11 + m_LeftInlier[n].y*h12 + h13;
			y_new = m_LeftInlier[n].x*h21 + m_LeftInlier[n].y*h22 + h23;
			rmse_combine += sqrt((x_new - m_RightInlier[n].x) *(x_new - m_RightInlier[n].x) +(y_new - m_RightInlier[n].y) *( y_new - m_RightInlier[n].y ));
		*/
			perspectiveTransform(m_LeftInlier,m_newRightInlier,H_h);
			x_new = m_newRightInlier[n].x;
			y_new = m_newRightInlier[n].y;

			rmse_combine += sqrt((x_new - m_RightInlier[n].x) *(x_new - m_RightInlier[n].x) +(y_new - m_RightInlier[n].y) *( y_new - m_RightInlier[n].y ));

		}

			rmse_combine = rmse_combine/InlinerCount;
		//	cout<<"H矩阵"<<H_h<<endl;

			//计算降采样图的H矩阵、原图H变换矩阵
			   double a[9];
				CvMat* H11;
				H11 =cvCreateMat(3,3,CV_64FC1);
				cvInitMatHeader(H11,3,3,CV_64FC1,a,CV_AUTOSTEP);
				cout<<H11<<endl;
				CvMat temp =H_h;
				cvCopy(&temp,H11);
				scale =cvmGet(H11,0,0);
				int  downrate = 1 / scale;

			double smallhx,smallhy,ha,hb;///该参数表示两个裁剪出来的小图的平移参数
			double bighx,bighy;//该参数表示待配准图像相对于参考图像的平移参数
			//px = cvmGet( H, 0, 2 );
			//py = cvmGet( H, 1, 2 );
			smallhx = cvmGet( H11, 0, 2 ); //这里算出的是待配准图像中感兴趣区域相对于参考图像的平移距离
			smallhy = cvmGet( H11, 1, 2 );
			ha = cvmGet(H11,2,0);
			hb = cvmGet(H11,2,1);

			CvMat *H1; //定义大图之间的变换矩阵，尤其是平移量进行更新

			bighx = smallhx * downScale_qt;
			bighy = smallhy *downScale_qt;
			H1 = H11;
			cvmSet(H1,0,2,bighx);
			cvmSet(H1,1,2,bighy);
			cvmSet(H1,2,0,ha*downrate);
			cvmSet(H1,2,1,hb*downrate);
			double h11,h12,h21,h22,sfx,sfy,rotate;

			h11 = cvmGet(H1,0,0);
			h12 = cvmGet(H1,0,1);
			h21 = cvmGet(H1,1,0);
			h22 = cvmGet(H1,1,1);
			sfx = cvSqrt(h11*h11+h21*h21);
			sfy = cvSqrt(h22*h22+h12*h12);
			rotate = (180*atan(h21/h11))/3.1415926;
			//cout<<rotate<<"度"<<endl;	//输出旋转角度。当图像旋转30度之后，SURF-BRISK计算的变换矩阵就有问题了
	
			t_ceshi = (double)(cvGetTickCount() - t_ceshi)/(cvGetTickFrequency()*1000.*1000.);	//得到大图之间的H矩阵关系，完成配准。

			cout<<"配准时间t_ceshi:"<<t_ceshi<<endl;	//t_ceshi记录的就是从开始配准到求出关系变换矩阵H的时间

		t_save =cvGetTickCount();
		Mat img1_1,img2_2;
		string dir_image1 = lujing[1];
		string dir_image2 = lujing[0];
		img1_1 =imread(dir_image1);		//1:xiao图
		img2_2 = imread(dir_image2);	//2:da图

		CvPoint pi11,pi22,pi3,pi4; 
		//	p22.x=bighx;  p22.y=bighy;	//左下角坐标
		pi22.x = smallhx*downScale_qt;	//downScale_qt:固定值
		pi22.y = smallhy*downScale_qt;
		pi11.x=(img1_1.cols)*h11+pi22.x;  pi11.y=h21*(img1_1.cols)+pi22.y; 
		pi3.x=h12*(img1_1.rows)+pi22.x; pi3.y=h22*(img1_1.rows)+pi22.y; 
		pi4.x=(img1_1.cols)*h11+h12*(img1_1.rows)+pi22.x;
		pi4.y=h21*(img1_1.cols)+h22*(img1_1.rows)+pi22.y;
		IplImage* ip3 = &IplImage(img2_2);
		cvLine(ip3,pi11,pi22,CV_RGB(255,0,0),2,CV_AA);
		cvLine(ip3,pi22,pi3,CV_RGB(255,0,0),2,CV_AA);
		cvLine(ip3,pi3,pi4,CV_RGB(255,0,0),2,CV_AA);
		cvLine(ip3,pi4,pi11,CV_RGB(255,0,0),2,CV_AA);
	//	t_bigline = (double)(cvGetTickCount() - t_bigline)/(cvGetTickFrequency()*1000.*1000.);
	//	cout<<"大图画框时间："<<t_bigline<<endl;
		Mat resultN(ip3, true);
	//	imshow("datu",resultN);


		dir4 +=  combine_name;
		dir5 = dir4 +"_big.jpg";
	
	
		imwrite(dir5,resultN);
		t_save =(double)(cvGetTickCount() - t_save)/(cvGetTickFrequency()*1000.*1000);
		cout<<"大图画框保存时间："<<t_save<<endl;

		Mat img_matches;
			//使用红色连接匹配的特征点对，绿色未匹配点对
			drawMatches(img1,m_LeftKey,img2,m_RightKey,goodMatches,img_matches,
					Scalar::all(-1),CV_RGB(0,255,0),Mat(),2);//最后一位是标志位，显示画圈的大小等
			imshow("Match_surf",img_matches);

	
	
	}


	int main()
	{
		//******测试SURF+BRISK（包含降采样、配准等时间）
		
		//1:对应大图ref；2:对应小图uav
	
		int th=1000;
		/*int detectorFlag =1;
		int descriptorFlag =1;*/
		dir3 = "e:\\张英测试报告\\0312SURF_BRISK\\1\\";
	
		dir1 = dir3 + "30002.jpg";	//	大图、参考图像
		dir2 = dir3 + "2.jpg";// 小图
		dir_p1 = dir3 + "p1.jpg";
		dir_p2 = dir3 + "p2.jpg";
	
		Mat img1 =imread(dir1);	//大图
		Mat img2 = imread(dir2);	//小图
		
		dir1_down = dir3 + "1_4.jpg";
		dir2_down = dir3 + "2_4.jpg";
		
		dir = dir3 + "ResultFiles" ;//算法结果文件夹
	
		
		dir_match_line =dir3 + "img_matches.jpg";	//显示两幅图像连线的匹配
	
		lujing.push_back(dir1);
		lujing.push_back(dir2);
		lujing.push_back(dir1_down);
		lujing.push_back(dir2_down);
		
	
		fileName_txt = "";
		combine_name = "";
	
		 remove_directory(dir);
		 _mkdir(dir.c_str());
		
		
	 dir4 = dir  + "\\_";
		
		fileName_txt += dir4 ;//将参数结果保存在参考图像文件夹下的txt文档中
		fileName_txt += double_to_String(scale);
		fileName_txt +="_";
	
	//	 t_combine = cvGetTickCount();
	
	
	
		Combine_Regiatration(dir2_down,dir1_down,detectorFlag,descriptorFlag,th/*,th2*/);		//前两个参数是路径
	//	Combine_RegiatrationMat(img2_down,img1_down,detectorFlag,descriptorFlag,th);			//前两个参数是Mat格式图像
	
	//	SURF_SURF(img2_down,img1_down,th);		//直接使用SURF-SURF算法，实参是降采样之后的图像
	//	SURF_BRISK(img2_down,img1_down,th,detector,descriptor_extractor);	//直接用SURF-BRISK算法，实参是降采样图像
	//	SurfFeatureDetector detector(th);
	//	BRISK  descriptor_extractor;
	//	SurfFeatureDetector descriptor_extractor1;
	//  SURF_BRISK2(img2,img1,th,detector,descriptor_extractor);	//检测子、描述子的定义在配准函数外面；实参是原图
	 // SURF_SURF2(img2,img1,th,detector,descriptor_extractor1);
	
		ofstream txt_out(fileName_txt.c_str(),ios::app);
		
			txt_out << t_sj << endl;		//场景检测子时间
			//QString ReferanceDetectorTime = QString::number(t_rj,'g',6);		//参考检测子时间
		//	txt_out << t_rj << endl;	//参考检测子时间
			//QString SceneDescriptorTime = QString::number(t_sm,'g',6);		//场景描述子时间
			txt_out << t_sm << endl;
			//QString referanceDescriptorTime = QString::number(t_rm,'g',6);	//参考描述子时间
		//	txt_out << t_rm << endl;
			//QString DescriptorsMatchTime =QString::number(t_pt,'g',6);		//描述子匹配的时间
			txt_out << t_pt << endl;
			//QString ScenePoint = QString::number(pt_s,'g',6);						//场景特征点数
			txt_out << pt_s << endl;
			//QString ReferancePoint = QString::number(pt_r,'g',6);					//参考特征点数
			txt_out << pt_r << endl;
			//QString MatchPairs = QString::number(pt_pairs,'g',6);					//系数筛选第一次粗匹配特征点对
			txt_out << pt_pairs << endl;
			//QString MatchChoosePairs = QString::number(FeaturePairs,'g',6);//RANSAC筛选后的特征点对
			txt_out << FeaturePairs << endl;
			//QString HTime = QString::number(t_H,'g',6);								//计算H矩阵的时间
			txt_out << t_H << endl;
			//QString CombineTime =QString::number(t_combine,'g',6);			//组合算法总时间
			//txt_out << t_combine << endl;	//t_combine在SiftTest项目中，combine函数中包含在原图中画框定位要花费的时间
			
			txt_out<<t_down<<endl;
			txt_out<<t_ceshi<<endl;
			//QString JingDu = QString::number(rmse_combine,'g',6);				//精度
		//	txt_out << rmse_combine << endl;
		//	txt_out<<t_save<<endl;
	
		waitKey(0);
	
	}