#include <iostream>
#include "opencv2/core/core.hpp"//��Ϊ���������Ѿ�������opencv��Ŀ¼�����԰��䵱���˱���Ŀ¼һ��
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp" //��opencv��ʹ��sift������Ҫ��ͷ�ļ�"opencv2/nonfree/nonfree.hpp"��
//ע������Ƿ���ѵģ�Sift�㷨��ר��Ȩ���ڸ��ױ��Ǵ�ѧ���������ҵ�����ʹ�ã������з��ա�
#include "windows.h"
#include "opencv2/legacy/legacy.hpp"
#include <fstream>
#include <ios>
#include <string>
#include <comutil.h>
#include <direct.h>  
#include <io.h>  
#include <windows.h>

using namespace std;
using namespace cv;


//IplImage* rotateImage1(IplImage* img,int degree);	//ͼ����ת
void rotateImage1(IplImage* img,int degree);	
void affineTransform(Mat Src);	//ͼ�����任
string Int_to_String(int n);

string dir,dir1,dir2;
string dir_rotate,dir_up,dir_down;

double degree = -10;	//��ת�Ƕ�
int main()
{
	 dir = "e:\\��Ӣ���Ա���\\0312SURF_BRISK\\1\\";
	 dir1 = dir+"1.jpg";
	 dir2 = dir+"rotate_-10.jpg";
	Mat src =imread(dir1);
	dir_rotate = dir + "rotate_" + Int_to_String(degree) + ".jpg";


	IplImage* img=&IplImage(src);
	IplImage* img1 = cvCloneImage(img);
	IplImage* img2 =cvCreateImage(cvSize(6000,6000),img1->depth,img1->nChannels);
	//affineTransform(src);
//	rotateImage(img1,img2,30);

	rotateImage1(img1,degree);

//	cvShowImage("ԭͼ",img1);
//	cvShowImage("rotate",img2);

	waitKey(0);

}


void rotateImage1(IplImage* img,int degree)
{  
	double angle = degree  * CV_PI / 180.; // ����    
	double a = sin(angle), b = cos(angle);   
	int width = img->width;    
	int height = img->height;    
	int width_rotate= int(height * fabs(a) + width * fabs(b));    
	int height_rotate=int(width * fabs(a) + height * fabs(b));    
	//��ת����map  
	// [ m0  m1  m2 ] ===>  [ A11  A12   b1 ]  
	// [ m3  m4  m5 ] ===>  [ A21  A22   b2 ]  
	float map[6];  
	CvMat map_matrix = cvMat(2, 3, CV_32F, map);    
	// ��ת����  
	CvPoint2D32f center = cvPoint2D32f(width / 2, height / 2);    
	cv2DRotationMatrix(center, degree, 1.0, &map_matrix);    
	map[2] += (width_rotate - width) / 2;    
	map[5] += (height_rotate - height) / 2;    
	IplImage* img_rotate = cvCreateImage(cvSize(width_rotate, height_rotate), 8, 3);   
	//��ͼ��������任  
	//CV_WARP_FILL_OUTLIERS - ����������ͼ������ء�  
	//�������������������ͼ��ı߽��⣬��ô���ǵ�ֵ�趨Ϊ fillval.  
	//CV_WARP_INVERSE_MAP - ָ�� map_matrix �����ͼ������ͼ��ķ��任��  
	cvWarpAffine( img,img_rotate, &map_matrix, CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS, cvScalarAll(0));    
	IplImage* dst = cvCreateImage(cvSize(img_rotate->width,img_rotate->height),img_rotate->depth,img_rotate->nChannels);
	dst = img_rotate;  
//	cvShowImage("rotate",dst);
//	cvSave("e:\\��Ӣ���Ա���\\0130������׼�������ͼ\\xz_rotate.jpg",dst);	//���ﱣ���ͼ��������

	Mat dst1;
	dst1 = cvarrToMat(dst);
	imshow("rotate",dst1);
	imwrite(dir_rotate,dst1);


}  


//����任
void affineTransform(Mat Src)
{
	Point2f srcTri[3];
	Point2f dstTri[3];
	Mat rot_mat( 2, 3, CV_32FC1 );
	Mat warp_mat( 2, 3, CV_32FC1 );
	Mat src, warp_dst, warp_rotate_dst;
	//����ͼ��
	//	src = imread( "e:\\��Ӣ���Ա���\\0130������׼�������ͼ\\3_4\\2.jpg", 1 );
	src = Src;
		warp_dst = Mat::zeros( src.rows, src.cols, src.type() );
	// ��3����ȷ��A����任
	srcTri[0] = Point2f( 0,0 );
	srcTri[1] = Point2f( src.cols - 1, 0 );
	srcTri[2] = Point2f( 0, src.rows - 1 );
	dstTri[0] = Point2f( src.cols*0.0, src.rows*0.33 );
	dstTri[1] = Point2f( src.cols*0.85, src.rows*0.25 );
	dstTri[2] = Point2f( src.cols*0.15, src.rows*0.7 );
	warp_mat = getAffineTransform( srcTri, dstTri );
	warpAffine( src, warp_dst, warp_mat, warp_dst.size() );
	/// ��ת����
	Point center = Point( warp_dst.cols/2, warp_dst.rows/2 );
	double angle = -10.0;
	double scale = 0.6;
	rot_mat = getRotationMatrix2D( center, angle, scale );
	warpAffine( warp_dst, warp_rotate_dst, rot_mat, warp_dst.size() );
	////OpenCV 1.0����ʽ
	//IplImage * img=cvLoadImage("baboon.jpg");
	//IplImage *img_rotate=cvCloneImage(img);
	//CvMat M =warp_mat;
	//cvWarpAffine(img,img_rotate, &M,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,cvScalarAll(0) );
	//cvShowImage("Wrap2",img_rotate);

	namedWindow( "Source", CV_WINDOW_AUTOSIZE );
	imshow( "Source", src );
	namedWindow( "Wrap", CV_WINDOW_AUTOSIZE );
	imshow( "Wrap", warp_dst );
	imwrite("e:\\��Ӣ���Ա���\\0130������׼�������ͼ\\2xz.jpg",warp_dst);
	namedWindow("Wrap+Rotate", CV_WINDOW_AUTOSIZE );
	imshow( "Wrap+Rotate", warp_rotate_dst );
}







