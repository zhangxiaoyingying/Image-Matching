	#include <stdio.h>
	#include <iostream>
	#include "opencv2/core/core.hpp"//��Ϊ���������Ѿ�������opencv��Ŀ¼�����԰��䵱���˱���Ŀ¼һ��
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


 

	int  downScale_qt = 4 ;//qt���潵�����������ã���δѡ����Ĭ��Ϊ��������
	string dir,dir1,dir2,dir3,dir4,dir5;
	string dir_p1,dir_p2;
	string dir_match_line;	//	����ͼ֮�仭����׼����
	string dir1_down,dir2_down;	//������ͼ�񱣴�·��
	double detectorThres1 = NULL;
	double detectorThres2 = NULL;
	double pointFilterThres = 0.52;
	double pyramidTime;

	string fileName_txt = "";//������������txt�ļ���
	string combine_name = "";//����㷨�������

	//DS2()�����л��õ���һЩ����������
	int detectorFlag=1;	//����ӵı�־λ
	int descriptorFlag=1;	//�����ӵı�־λ

	int FeaturePairs=0;	//ͳ�������ȷƥ��ĵ����
	double rmse=0.0;	//DS2������ͳ�ƾ���

	double t_sj,t_sm,t_rj,t_rm;	//�����������ʱ��
	double t_pt,t_choose,t_H;
	double t_bigline;//��ͼ�����ʱ
	double t_down;//��������ʱ
	double t_save;	//ͳ�ƴ�ͼ�ع�����ͼ��ʱ��
	double rmse_combine =0.0;
	int pt_s,pt_r,pt_pairs;
	vector<CvMat*> H_2;

	double t_combine=0.0;
	double scale =0.25;
 


	ofstream outfile;	//��һЩ���������txt�ļ�
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
		//img�������������汾����׼�е�ͼ��
	

		IplImage* ip_temp1 =&IplImage(uav);
		IplImage* img1 = cvCloneImage(ip_temp1);
		IplImage* ip_temp2 =&IplImage(ref);
		IplImage* img2 = cvCloneImage(ip_temp2);
		Ptr<DescriptorMatcher> descriptor_matcher = DescriptorMatcher::create("BruteForce"/*"FlannBased"*/);//��������ƥ����

		//double t =(double)cvGetTickCount(); //ͳ����׼��ʱ��
	
		//��������
		vector <KeyPoint> m_LeftKey,m_RightKey;		//������
		Mat descriptors1,descriptors2;							//������
		Mat img_m_LeftKey,img_m_RightKey;				//�����������������ͼ��

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

		//����ƥ��
		vector<DMatch> matches;//ƥ������DMatch��һ���ṹ��
	
		t_pt = cvGetTickCount();
		descriptor_matcher->match(descriptors1,descriptors2,matches);//ƥ������ͼ�����������
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
	//������ֵ��ɸѡ�Ϻõ�ƥ���  
		vector<DMatch> goodMatches;
	//	float xishu=0.38;
		for(int i=0;i<matches.size();i++)
		{
			if(matches[i].distance < (pointFilterThres *max_dist))	//�������ֵ����Ҳ�ǱȽ���Ҫ��
		//	if( matches[i].distance <= max(2*min_dist, 0.02) )
			{
				goodMatches.push_back(matches[i]);
			}
		}

		pt_pairs = goodMatches.size();

		//RANSACƥ�����
		vector<DMatch> m_Matches = goodMatches;
		//����ռ�
		int ptCount = m_Matches.size();//int ptCount = (int)m_Matches.size()����������ǿ��ת������
		Mat p1(ptCount,2,CV_32F);//����һ��Mat�ṹ�������У�2�У�32λ������
		Mat p2(ptCount,2,CV_32F);

		//�ѵõ���Keypointת��ΪMat
		Point2f pt;
		for(int i=0;i<m_Matches.size();i++)
		{
			pt = m_LeftKey[m_Matches[i].queryIdx].pt;
			p1.at<float>(i,0) = pt.x;//��һ�б�ʾx���ꣻ�ڶ��б�ʾy��//��������at����������ȡ�����е�ĳ������
			p1.at<float>(i,1) = pt.y;
		
		
			pt = m_RightKey[m_Matches[i].trainIdx].pt;
			p2.at<float>(i,0) = pt.x;
			p2.at<float>(i,1) = pt.y;
		}

		//��RANSAC��������F
		Mat m_Fundamental;//�����������
		vector<uchar> m_RANSACStatus;//�ñ��������洢RANSAC֮��ÿ�����״̬
		findFundamentalMat(p1,p2,m_RANSACStatus,FM_RANSAC);//���һλ�Ǳ�־λ����ʾʹ��RANSAC�㷨

		//����Ұ�����
		int OutlinerCount = 0;
		for(int i=0;i<m_Matches.size();i++)
		{
	
			if(m_RANSACStatus[i] == 0)			//״̬Ϊ0��ʾ��Ұ��
			{
				OutlinerCount++;
			}
		}
		int InlinerCount = ptCount - OutlinerCount;
	//	cout<<"�ڵ����Ϊ��"<<InlinerCount<<endl;
	
		//����3���������������ڵ��ƥ���ϵ
		vector<Point2f> m_LeftInlier;
		vector<Point2f> m_RightInlier;
		vector<DMatch> m_InlierMatches;
		m_LeftInlier.resize(InlinerCount);		
		m_RightInlier.resize(InlinerCount  );
		m_InlierMatches.resize(InlinerCount );

		InlinerCount = 0;
		for(int i =0;i<ptCount;i++)
		{
		
			if(m_RANSACStatus[i] != 0)	//����ȷƥ��ĵ㸳���ڵ�
			{
				m_LeftInlier[InlinerCount].x = p1.at<float>(i,0);
				m_LeftInlier[InlinerCount].y = p1.at<float>(i,1);
				m_RightInlier[InlinerCount].x = p2.at<float>(i,0);
				m_RightInlier[InlinerCount].y = p2.at<float>(i,1);
				m_InlierMatches[InlinerCount].queryIdx = InlinerCount;
				m_InlierMatches[InlinerCount].trainIdx = InlinerCount;

				//if(m_RightInlier[InlinerCount].x < inlier_minRx)
				//	inlier_minRx = m_RightInlier[InlinerCount].x;		//�洢�ڵ�����ͼ����С����
				InlinerCount++;
			}
		}

	t_choose =(double)(cvGetTickCount() - t_choose)/(cvGetTickFrequency()*1000.*1000.);
	cout<<"t_choose:"<<t_choose<<endl;
	
		//����H�����洢RANSAC�任�õ��ĵ�Ӧ�Ծ���

		/*����H�����ʱ��*/
		t_H =(double)cvGetTickCount();
		Mat H_h= findHomography(m_LeftInlier,m_RightInlier,CV_RANSAC);	//ʹ���ڵ����H

		t_H =(double)(cvGetTickCount() - t_H)/(cvGetTickFrequency()*1000.*1000.);

		//�󾫶ȳ���
	

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
		//	cout<<"H����"<<H_h<<endl;

			//���㽵����ͼ��H����ԭͼH�任����
			   double a[9];
				CvMat* H11;
				H11 =cvCreateMat(3,3,CV_64FC1);
				cvInitMatHeader(H11,3,3,CV_64FC1,a,CV_AUTOSTEP);
				cout<<H11<<endl;
				CvMat temp =H_h;
				cvCopy(&temp,H11);
				scale =cvmGet(H11,0,0);
				int  downrate = 1 / scale;

			double smallhx,smallhy,ha,hb;///�ò�����ʾ�����ü�������Сͼ��ƽ�Ʋ���
			double bighx,bighy;//�ò�����ʾ����׼ͼ������ڲο�ͼ���ƽ�Ʋ���
			//px = cvmGet( H, 0, 2 );
			//py = cvmGet( H, 1, 2 );
			smallhx = cvmGet( H11, 0, 2 ); //����������Ǵ���׼ͼ���и���Ȥ��������ڲο�ͼ���ƽ�ƾ���
			smallhy = cvmGet( H11, 1, 2 );
			ha = cvmGet(H11,2,0);
			hb = cvmGet(H11,2,1);

			CvMat *H1; //�����ͼ֮��ı任����������ƽ�������и���

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
			//cout<<rotate<<"��"<<endl;	//�����ת�Ƕȡ���ͼ����ת30��֮��SURF-BRISK����ı任�������������
	
			t_ceshi = (double)(cvGetTickCount() - t_ceshi)/(cvGetTickFrequency()*1000.*1000.);	//�õ���ͼ֮���H�����ϵ�������׼��

			cout<<"��׼ʱ��t_ceshi:"<<t_ceshi<<endl;	//t_ceshi��¼�ľ��Ǵӿ�ʼ��׼�������ϵ�任����H��ʱ��

		t_save =cvGetTickCount();
		Mat img1_1,img2_2;
		string dir_image1 = lujing[1];
		string dir_image2 = lujing[0];
		img1_1 =imread(dir_image1);		//1:xiaoͼ
		img2_2 = imread(dir_image2);	//2:daͼ

		CvPoint pi11,pi22,pi3,pi4; 
		//	p22.x=bighx;  p22.y=bighy;	//���½�����
		pi22.x = smallhx*downScale_qt;	//downScale_qt:�̶�ֵ
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
	//	cout<<"��ͼ����ʱ�䣺"<<t_bigline<<endl;
		Mat resultN(ip3, true);
	//	imshow("datu",resultN);


		dir4 +=  combine_name;
		dir5 = dir4 +"_big.jpg";
	
	
		imwrite(dir5,resultN);
		t_save =(double)(cvGetTickCount() - t_save)/(cvGetTickFrequency()*1000.*1000);
		cout<<"��ͼ���򱣴�ʱ�䣺"<<t_save<<endl;

		Mat img_matches;
			//ʹ�ú�ɫ����ƥ���������ԣ���ɫδƥ����
			drawMatches(img1,m_LeftKey,img2,m_RightKey,goodMatches,img_matches,
					Scalar::all(-1),CV_RGB(0,255,0),Mat(),2);//���һλ�Ǳ�־λ����ʾ��Ȧ�Ĵ�С��
			imshow("Match_surf",img_matches);

	
	
	}


	int main()
	{
		//******����SURF+BRISK����������������׼��ʱ�䣩
		
		//1:��Ӧ��ͼref��2:��ӦСͼuav
	
		int th=1000;
		/*int detectorFlag =1;
		int descriptorFlag =1;*/
		dir3 = "e:\\��Ӣ���Ա���\\0312SURF_BRISK\\1\\";
	
		dir1 = dir3 + "30002.jpg";	//	��ͼ���ο�ͼ��
		dir2 = dir3 + "2.jpg";// Сͼ
		dir_p1 = dir3 + "p1.jpg";
		dir_p2 = dir3 + "p2.jpg";
	
		Mat img1 =imread(dir1);	//��ͼ
		Mat img2 = imread(dir2);	//Сͼ
		
		dir1_down = dir3 + "1_4.jpg";
		dir2_down = dir3 + "2_4.jpg";
		
		dir = dir3 + "ResultFiles" ;//�㷨����ļ���
	
		
		dir_match_line =dir3 + "img_matches.jpg";	//��ʾ����ͼ�����ߵ�ƥ��
	
		lujing.push_back(dir1);
		lujing.push_back(dir2);
		lujing.push_back(dir1_down);
		lujing.push_back(dir2_down);
		
	
		fileName_txt = "";
		combine_name = "";
	
		 remove_directory(dir);
		 _mkdir(dir.c_str());
		
		
	 dir4 = dir  + "\\_";
		
		fileName_txt += dir4 ;//��������������ڲο�ͼ���ļ����µ�txt�ĵ���
		fileName_txt += double_to_String(scale);
		fileName_txt +="_";
	
	//	 t_combine = cvGetTickCount();
	
	
	
		Combine_Regiatration(dir2_down,dir1_down,detectorFlag,descriptorFlag,th/*,th2*/);		//ǰ����������·��
	//	Combine_RegiatrationMat(img2_down,img1_down,detectorFlag,descriptorFlag,th);			//ǰ����������Mat��ʽͼ��
	
	//	SURF_SURF(img2_down,img1_down,th);		//ֱ��ʹ��SURF-SURF�㷨��ʵ���ǽ�����֮���ͼ��
	//	SURF_BRISK(img2_down,img1_down,th,detector,descriptor_extractor);	//ֱ����SURF-BRISK�㷨��ʵ���ǽ�����ͼ��
	//	SurfFeatureDetector detector(th);
	//	BRISK  descriptor_extractor;
	//	SurfFeatureDetector descriptor_extractor1;
	//  SURF_BRISK2(img2,img1,th,detector,descriptor_extractor);	//����ӡ������ӵĶ�������׼�������棻ʵ����ԭͼ
	 // SURF_SURF2(img2,img1,th,detector,descriptor_extractor1);
	
		ofstream txt_out(fileName_txt.c_str(),ios::app);
		
			txt_out << t_sj << endl;		//���������ʱ��
			//QString ReferanceDetectorTime = QString::number(t_rj,'g',6);		//�ο������ʱ��
		//	txt_out << t_rj << endl;	//�ο������ʱ��
			//QString SceneDescriptorTime = QString::number(t_sm,'g',6);		//����������ʱ��
			txt_out << t_sm << endl;
			//QString referanceDescriptorTime = QString::number(t_rm,'g',6);	//�ο�������ʱ��
		//	txt_out << t_rm << endl;
			//QString DescriptorsMatchTime =QString::number(t_pt,'g',6);		//������ƥ���ʱ��
			txt_out << t_pt << endl;
			//QString ScenePoint = QString::number(pt_s,'g',6);						//������������
			txt_out << pt_s << endl;
			//QString ReferancePoint = QString::number(pt_r,'g',6);					//�ο���������
			txt_out << pt_r << endl;
			//QString MatchPairs = QString::number(pt_pairs,'g',6);					//ϵ��ɸѡ��һ�δ�ƥ���������
			txt_out << pt_pairs << endl;
			//QString MatchChoosePairs = QString::number(FeaturePairs,'g',6);//RANSACɸѡ����������
			txt_out << FeaturePairs << endl;
			//QString HTime = QString::number(t_H,'g',6);								//����H�����ʱ��
			txt_out << t_H << endl;
			//QString CombineTime =QString::number(t_combine,'g',6);			//����㷨��ʱ��
			//txt_out << t_combine << endl;	//t_combine��SiftTest��Ŀ�У�combine�����а�����ԭͼ�л���λҪ���ѵ�ʱ��
			
			txt_out<<t_down<<endl;
			txt_out<<t_ceshi<<endl;
			//QString JingDu = QString::number(rmse_combine,'g',6);				//����
		//	txt_out << rmse_combine << endl;
		//	txt_out<<t_save<<endl;
	
		waitKey(0);
	
	}