#include <iostream>
#include <conio.h>

#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

//----Some requisite parameters/constant-----
float pi = 3.14159265358979f;
int Nmax = 2000;
int Omax = 20000;
int Kmax = 20000;
int eta = 60;
float S = 0.4f;
float fh = (57.0f/180.0f)*pi;
float fv = (43.0f/180.0f)*pi;
int l = 60;
float epsilon = 0.04f;
float alpha_in = 0.8f;
int num_points = 0;
int eff_r = 0;
int out_pt = 0;
float off_angle_s = (20.0f/180.0f)*pi;
float off_angle_l = (160.0f/180.0f)*pi;
float z_off = 0.08f;

struct Vec_3D{
	float x,y,z;
};

struct Vec_3D_m{
	float x,y,z;
	int n;
};

struct Vec_2D{
	int x,y;
	float val;
};

class basic_nav{
private:
	Mat point_cloud,point_cloud_filt,trav_map,depth_map;
	float h_rob;
	float clear;
	float h_rob_eff;
	float h_max,z_min,z_max;
	float depth_max,depth_min,depth_range;
	float height_max,height_min,height_range;
	float hor_max,hor_min,hor_range;
	int rows,cols,eff_pts,map_d1,map_d2;
	int num_points;
	int eff_r;
	int out_pt;
		Vec_3D *P;										//List of plane filtered points
		Vec_3D *R;										//Normals corresponding to plane filtered points								
		Vec_3D *O;											//List of Outliers points
		Vec_3D_m *R_eff;
		Vec_3D *H;
		Vec_3D *Ob;

public:
	basic_nav(float ih_rob=2.365,float iclear=0.5f,float ih_max=0.0f,float iz_min=-0.5,float iz_max=-0.4f,int ipt_cloud_rows=480,int ipt_cloud_cols=640,int imap_d1=480,int imap_d2=640){
		h_rob = ih_rob;
		clear = iclear;
		h_max = ih_max;
		z_min = iz_min;
		z_max = iz_max;
		rows = ipt_cloud_rows;
		cols = ipt_cloud_cols;
		map_d1 = imap_d1;
		map_d2 = imap_d2;
		height_min=-3.26f,height_max=3.26f,height_range=5.0f;
		hor_min=-3.5f,hor_max=2.0f,hor_range=5.0f;
		depth_min=0.3f,depth_max=6.0f,depth_range=6.0f;
		h_rob_eff = h_rob+clear+height_min;
		eff_pts = 0;
		num_points = 0;
		eff_r = 0;
		out_pt = 0;
		point_cloud = Mat(ipt_cloud_rows,ipt_cloud_rows,CV_32FC3);
		point_cloud_filt = Mat(1,1,CV_32FC3);
		depth_map = Mat(ipt_cloud_rows,ipt_cloud_cols,CV_16UC1);
		trav_map = Mat(map_d1,map_d2,CV_8UC3,Scalar(128,128,128));
		P = new Vec_3D[Nmax+1000];										//List of plane filtered points
		R = new Vec_3D[Nmax+1000];										//Normals corresponding to plane filtered points								
		O = new Vec_3D[40*Omax];											//List of Outliers points
		R_eff = new Vec_3D_m[500];
		H = new Vec_3D[20*Omax];
		Ob = new Vec_3D[20*Omax];
	}

	void discard_tall(void){
		Vec3f v;
		for(int i=0;i<rows;i++){
			for(int j=0;j<cols;j++){
				v = point_cloud.at<Vec3f>(i,j);
				
				if(v[0]==0.0f && v[1]==0.0f && v[2]==0.0f)
					continue;
				if(v[1]<h_rob_eff){
					point_cloud_filt.push_back<Vec3f>(v);
					eff_pts++;
				}
			}
		}
	}

	void project_on_map(Vec3f v,uchar val[]){
		int j = int(((v[0]-hor_min)/hor_range)*float(map_d1));
		int i = int(((v[2]-depth_min)/depth_range)*float(map_d2));
		if(i>map_d1 || j>map_d2 || i<0||j<0);
		else{
			trav_map.at<cv::Vec3b>(map_d1-i,j)[0] = val[0];
			trav_map.at<cv::Vec3b>(map_d1-i,j)[1] = val[1];
			trav_map.at<cv::Vec3b>(map_d1-i,j)[2] = val[2];
		}
	}
		
	void project_ob_abv(void){
		Vec3f v;
		for(int i=0;i<eff_pts;i++){
			v = point_cloud_filt.at<Vec3f>(i,0);
			if(v[1]>h_max){
				//project_on_map(v,{);
			}
		}
	}
	
	void project_floor(void){
		Vec3f v;
		for(int i=0;i<eff_pts;i++){
			v = point_cloud_filt.at<Vec3f>(i,0);
			if(v[1]<z_max && v[2]>z_min){
				project_on_map(v,0);
			}
			else;
				//project_on_map(v,255);
		}
	}

	void project_floor_1(Vec_3D H[],int h_pt){
		Vec3f v;
		uchar val[3]={0,255,0};
		for(int i=0;i<h_pt;i++){
			v[0] = H[i].x;
			v[1] = H[i].y;
			v[2] = H[i].z;
			
			project_on_map(v,val);
		}
	}

	void project_obstacles(Vec_3D Ob[],int ob_pt){
		Vec3f v;
		uchar val[3] = {0,0,255};
		for(int i=0;i<ob_pt;i++){
			v[0] = Ob[i].x;
			v[1] = Ob[i].y;
			v[2] = Ob[i].z;
			project_on_map(v,val);
		}
	}

	Mat& acc_pt_cloud(void){
		return point_cloud;
	}

	Mat& acc_depth_map(void){
		return depth_map;
	}

	void FSPF(cv::Mat *depth_img,cv::Mat *point_cloud,int img_width,int img_height,Vec_3D P[],Vec_3D R[], Vec_3D O[],Vec_3D_m R_eff[]){

		int n = 0;
		int k = 0;	

		Vec_2D *d = new Vec_2D[l];
		Vec_3D *p = new Vec_3D[l];
		Vec_3D diffp1,diffp2;

		Vec_3D *Pbar = new Vec_3D[l];
		Vec_3D *Rbar = new Vec_3D[l];;

		Vec_3D r;
		float zbar;
		int wbar,hbar;
		do
		{
			k++;

			d[0].x = rand()%img_width;
			d[0].y = rand()%img_height;
			//d[0].val = depth_img.at<ushort>(d[0].x,d[0].y);

			//std::cout << d[0].x << " " << d[0].y << " " << d[0].val <<"\n";
		
			int rnd1 = rand();
			int rnd2 = rand();
			d[1].x = abs(d[0].x + (2*(rnd1%eta) - eta));
			if(d[1].x >= img_width)
				d[1].x = img_width-1;
			d[1].y = abs(d[0].y + (2*(rnd2%eta) - eta));
			if(d[1].y >= img_height)
				d[1].y = img_height-1;
			//d[1].val = depth_img.at<short>(d[1].x,d[1].y);
			if(d[0].x==d[1].x && d[0].y==d[1].y){
				k--;
				continue;
			}

			//std::cout << d[1].x << " " << d[1].y << " " << d[1].val << "\n";

			d[2].x = abs(d[0].x + (2*(rand()%eta) - eta));
			if(d[2].x >= img_width)
				d[2].x = img_width-1;
			d[2].y = abs(d[0].y + (2*(rand()%eta) - eta));
			if(d[2].y >= img_height)
				d[2].y = img_height-1;
			if(d[2].x==d[0].x && d[2].y==d[0].y){
				k--;
				continue;
			}
			//d[2].val = depth_img.at<short>(d[2].x,d[2].y);

			//std::cout << d[2].x << " " << d[2].y << " " << d[2].val << "\n";

			//recontruct p0,p1,p2 from d0,d1,d2
			p[0].x = point_cloud->at<cv::Vec3f>(d[0].y,d[0].x)[0];
			p[0].y = point_cloud->at<cv::Vec3f>(d[0].y,d[0].x)[1];
			p[0].z = point_cloud->at<cv::Vec3f>(d[0].y,d[0].x)[2];

			p[1].x = point_cloud->at<cv::Vec3f>(d[1].y,d[1].x)[0];
			p[1].y = point_cloud->at<cv::Vec3f>(d[1].y,d[1].x)[1];
			p[1].z = point_cloud->at<cv::Vec3f>(d[1].y,d[1].x)[2];

			p[2].x = point_cloud->at<cv::Vec3f>(d[2].y,d[2].x)[0];
			p[2].y = point_cloud->at<cv::Vec3f>(d[2].y,d[2].x)[1];
			p[2].z = point_cloud->at<cv::Vec3f>(d[2].y,d[2].x)[2];

			if(p[0].x ==0 && p[0].y ==0 && p[0].z ==0 || p[1].x ==0 && p[1].y ==0 && p[1].z ==0 || p[2].x ==0 && p[2].y ==0 && p[2].z ==0)
			{
				continue;
			}
			diffp1.x = p[1].x - p[0].x;
			diffp1.y = p[1].y - p[0].y;
			diffp1.z = p[1].z - p[0].z;

			diffp2.x = p[2].x - p[0].x;
			diffp2.y = p[2].y - p[0].y;
			diffp2.z = p[2].z - p[0].z;

			r.x = diffp1.y*diffp2.z - diffp1.z*diffp2.y;
			r.y = diffp1.z*diffp2.x - diffp1.x*diffp2.z;
			r.z = diffp1.x*diffp2.y - diffp1.y*diffp2.x;

			float dist = sqrt(r.x*r.x + r.y*r.y + r.z*r.z);
			if(dist > 0)
			{
				r.x /= dist;
				r.y /= dist;
				r.z /= dist;
			}
			else
				continue;

			zbar = float(p[0].z + p[1].z +p[2].z)/3.0f;

			wbar = int(float(img_width)*(S/zbar)*tan(fh));
			hbar = int(float(img_height)*(S/zbar)*tan(fv));

			int num_inliers = 0;
			float e = 0.0f;

			for(int j=3;j<l;j++)
			{
					
				int rand1 = (2*(rand()%(wbar/2))-wbar/2);
				int rand2 = (2*(rand()%(hbar/2))-hbar/2);
				d[j].x = abs(d[0].x + rand1);
				if(d[j].x >= img_width)
					d[j].x = img_width-1;

				d[j].y = abs(d[0].y + rand2);
				if(d[j].y >= img_height)
					d[j].y = img_height-1;

				//reconstruct 3D points

				p[j].x = point_cloud->at<cv::Vec3f>(d[j].y,d[j].x)[0];
				p[j].y = point_cloud->at<cv::Vec3f>(d[j].y,d[j].x)[1];
				p[j].z = point_cloud->at<cv::Vec3f>(d[j].y,d[j].x)[2];

				e = abs(r.x*(p[j].x-p[0].x) + r.y*(p[j].y - p[0].y) + r.z*(p[j].z - p[0].z));

				if(e < epsilon)
				{
					Pbar[num_inliers].x = p[j].x;
					Pbar[num_inliers].y = p[j].y;
					Pbar[num_inliers].z = p[j].z;

					Rbar[num_inliers].x = r.x;
					Rbar[num_inliers].y = r.y;
					Rbar[num_inliers].z = r.z;
					num_inliers++;
				} 
			}

			if(num_inliers > alpha_in*l)
			{
				P[num_points].x = p[0].x;
				P[num_points].y = p[0].y;
				P[num_points].z = p[0].z;

				P[1+num_points].x = p[1].x;
				P[1+num_points].y = p[1].y;
				P[1+num_points].z = p[1].z;

				P[2+num_points].x = p[2].x;
				P[2+num_points].y = p[2].y;
				P[2+num_points].z = p[2].z;

				for(int i=0;i<3;i++)
				{
					R[i+num_points].x = r.x;
					R[i+num_points].y = r.y;
					R[i+num_points].z = r.z;
				}
				num_points+=3;
				for(int i=0;i<num_inliers;i++)
				{
					P[i+num_points].x = Pbar[i].x;
					P[i+num_points].y = Pbar[i].y;
					P[i+num_points].z = Pbar[i].z;

					R[i+num_points].x = Rbar[i].x;
					R[i+num_points].y = Rbar[i].y;
					R[i+num_points].z = Rbar[i].z;
				}

				R_eff[eff_r].x = r.x;
				R_eff[eff_r].y = r.y;
				R_eff[eff_r].z = r.z;
				R_eff[eff_r].n = num_inliers+3;
				eff_r++;

				num_points += num_inliers;
			}
			else
			{
				O[out_pt].x = p[0].x;
				O[out_pt].y = p[0].y;
				O[out_pt].z = p[0].z;

				O[1+out_pt].x = p[1].x;
				O[1+out_pt].y = p[1].y;
				O[1+out_pt].z = p[1].z;

				O[2+out_pt].x = p[2].x;
				O[2+out_pt].y = p[2].y;
				O[2+out_pt].z = p[2].z;

				out_pt+=3;
				for(int i=0;i<num_inliers;i++)
				{
					O[i+out_pt].x = Pbar[i].x;
					O[i+out_pt].y = Pbar[i].y;
					O[i+out_pt].z = Pbar[i].z;
				}
				out_pt+=num_inliers;
			}
	
		}
		while(num_points<Nmax && k < Kmax && out_pt<20000);

		delete[] d,p,Pbar,Rbar;
	}

	Mat* whole_process_v1(Mat ipoint_cloud){
		discard_tall();
		project_ob_abv();
		project_floor();
		return &trav_map;
	}

	int search_for_hor_plane(Vec_3D P[],Vec_3D R[],Vec_3D O[],Vec_3D_m R_eff[],Vec_3D H[],Vec_3D Ob[],int *ob_pt){
		int h_pt=0;
		int ob_pt1 = 0;
		float h_sum=0,h_mean=0;
		float angle = 0.0f;
		Vec3f v;
		for(int i=0;i<num_points;i++){
			angle = acos(R[i].y);
			if((angle<off_angle_s || angle>off_angle_l) && P[i].y < z_max){
				H[h_pt].x=P[i].x,H[h_pt].y=P[i].y,H[h_pt].z=P[i].z;
				h_sum+=H[h_pt].y;
				h_pt++;
			}
		}
		h_mean = h_sum/h_pt++;
		
		/*for(int i=0;i<rows;i+=1){
			for(int j=0;j<cols;j+=1){
				v = point_cloud.at<Vec3f>(i,j);
				
				if(v[0]==0.0f && v[1]==0.0f && v[2]==0.0f)
					continue;
				if(abs(v[1]-h_mean)<z_off){
					H[h_pt].x = v[0];
					H[h_pt].y = v[1];
					H[h_pt].z = v[2];
					h_pt++;
				}
				else{
					Ob[ob_pt1].x = v[0];
					Ob[ob_pt1].y = v[1];
					Ob[ob_pt1].z = v[2];
					ob_pt1++;
				}
			}
		}*/

		for(int i=0;i<out_pt;i++){
			v[0] = O[i].x;
			v[1] = O[i].y;
			v[2] = O[i].z;
			if(v[0]==0.0f && v[1]==0.0f && v[2]==0.0f)
				continue;
			if(abs(v[1]-h_mean)<z_off){
				H[h_pt].x = v[0];
				H[h_pt].y = v[1];
				H[h_pt].z = v[2];
				h_pt++;
			}
			else{
				Ob[ob_pt1].x = v[0];
				Ob[ob_pt1].y = v[1];
				Ob[ob_pt1].z = v[2];
				ob_pt1++;
			}
		}

		*ob_pt = ob_pt1;
		return h_pt;
	}

	Mat& whole_process_v2(void){

		num_points = 0;
		eff_r = 0;
		out_pt = 0;
		
		int f_pt=0;
		int ob_pt = 0;
		
		trav_map = Mat(map_d1,map_d2,CV_8UC3,Scalar(128,128,128));
		FSPF(&depth_map,&point_cloud,cols,rows,P,R,O,R_eff);

		f_pt = search_for_hor_plane(P,R,O,R_eff,H,Ob,&ob_pt);
		project_floor_1(H,f_pt);
		project_obstacles(Ob,ob_pt);

		//delete[] P,R,O,R_eff,H,Ob;
		return trav_map;
	}
};

void parmam_tuning(void){
	VideoCapture dumm(CV_CAP_OPENNI);
	if(!dumm.isOpened()){
		cout << "cannot open device(kinect). Press any key";
		_getch();
		return;
	}

	dumm.grab();
}

int main(void){
	VideoCapture cap(CV_CAP_OPENNI);
	if(!cap.isOpened()){
		cout << "cannot open device(kinect). Press any key";
		_getch();
		return 0;
	}

	basic_nav nav1(2.365f);
	Mat RGB,map;

	while(cap.grab()){
		double time = double(cv::getTickCount());
		cap.retrieve(nav1.acc_pt_cloud(),CV_CAP_OPENNI_POINT_CLOUD_MAP);
		cap.retrieve(nav1.acc_depth_map(),CV_CAP_OPENNI_DEPTH_MAP);
		cap.retrieve(RGB,CV_CAP_OPENNI_BGR_IMAGE);

		imshow("RGB",RGB);
		if(waitKey(1)==32)
			break;
		double tim = double(cv::getTickCount());
			map = nav1.whole_process_v2();
			tim = double(cv::getTickCount())-tim;
			cout << "\n" << tim/cv::getTickFrequency();
			//cv::erode(;
			imshow("MAP",map);
			//cout << map;
			waitKey(10);
			time = double(cv::getTickCount())-time;
			cout << "time full" <<time/cv::getTickFrequency();
	}
	return 0;
}



