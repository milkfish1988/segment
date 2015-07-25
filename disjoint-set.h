/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/
#include <list>
#include <iostream>
#include <iomanip>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;

#ifndef DISJOINT_SET
#define DISJOINT_SET

// disjoint-set forests using union-by-rank and path compression (sort of).

typedef struct {
  int rank;
  int p;
  int size;
  int minx;
  int miny;
  int maxx;
  int maxy;
} uni_elt;

typedef struct  
{
	int x;
	int y;
	int i;
} uni_pos;
typedef list<uni_pos> LISTPOS;

class universe {
public:
  universe(int elements, int height, int width);
  ~universe();
  int find(int x);  
  void join(int x, int y);
  int size(int x) const { return elts[x].size; }
  int num_sets() const { return num; }
  void UpdateJoinedBox(int x, int y, int minx, int miny, int maxx ,int maxy);
  void CalcJoinedBox(int ax, int ay, int bx, int by, int &minx, int &miny, int &maxx, int &maxy, int height, int width, int &sizea, int &sizeb);
  void showcomplist(int height, int width, image<rgb> *input_edge);
  void showJoinedcomplist(int a, int height, int width, image<rgb> *input_edge);
  void Find8NeighborComp(int x, int y, vector<int> &neighbor_comp, int height, int width, image<rgb> *input_edge);
  void GetRegion(int height, int width, image<rgb> *input_edge, image<rgb> *input_im);
  void GetRegion2(int height, int width, image<rgb> *input_edge, image<rgb> *input_im);
  void GetRegion3(int height, int width, image<rgb> *input_edge, image<rgb> *input_im, Mat gradient_label, Mat src_image, string ifile_name);
  void SaveWindowImage(int r, int window_minx, int window_miny, int window_maxx, int window_maxy, Mat src_image, string ifile_name, string dir,
	  int &cnt_window, int &cnt_r, int &cnt_s, Vec3b *lp, int x, int y);

private:
  uni_elt *elts;
  LISTPOS *complists;
  int num;
};

universe::universe(int elements, int height, int width) {
  elts = new uni_elt[elements];
  complists = new LISTPOS[elements];
  num = elements;
  int i = 0;
  uni_pos u_pos;
  for (int y = 0; y < height; y++) {
	  for (int x = 0; x < width; x++) {
		  elts[i].rank = 0;
		  elts[i].size = 1;
		  elts[i].p = i;
		  elts[i].minx = x;
		  elts[i].miny = y;
		  elts[i].maxx = x;
		  elts[i].maxy = y;
		  u_pos.x = x;
		  u_pos.y = y;
		  u_pos.i = i;
		  complists[i].push_back(u_pos);
		  i++;
	  }
  }
}
  
universe::~universe() {
  delete [] elts;
}

int universe::find(int x) {
  int y = x;
  while (y != elts[y].p)
	  y = elts[y].p;
  elts[x].p = y;
  return y;
}

void universe::join(int x, int y) {
  LISTPOS::iterator it;
  if (elts[x].rank > elts[y].rank) {
    elts[y].p = x;
    elts[x].size += elts[y].size;
	it = complists[x].end();
	complists[x].splice(it, complists[y]);	//move all element in list y to the end of list x
  } else {
    elts[x].p = y;
    elts[y].size += elts[x].size;
	it = complists[y].end();
	complists[y].splice(it, complists[x]);	//move all element in list x to the end of list y
    if (elts[x].rank == elts[y].rank)
      elts[y].rank++;
  }
  num--;
}

void universe::UpdateJoinedBox(int x, int y, int minx, int miny, int maxx ,int maxy)
{
	//if (elts[x].rank > elts[y].rank) {
		elts[x].minx = minx;
		elts[x].miny = miny;
		elts[x].maxx = maxx;
		elts[x].maxy = maxy;
	//} else {
		elts[y].minx = minx;
		elts[y].miny = miny;
		elts[y].maxx = maxx;
		elts[y].maxy = maxy;
	//}
}

void universe::CalcJoinedBox(int ax, int ay, int bx, int by, int &minx, int &miny, int &maxx, int &maxy, 
	int height, int width, int &sizea, int &sizeb)
{
	int min_x_a, min_y_a, max_x_a, max_y_a;
	int min_x_b, min_y_b, max_x_b, max_y_b;

	int a = ay * width + ax;
	int b = by * width + bx;

	a = find(a);
	b = find(b);

	sizea = elts[a].size;
	sizeb = elts[b].size;

	min_x_a = elts[a].minx;
	min_y_a = elts[a].miny;
	max_x_a = elts[a].maxx;
	max_y_a = elts[a].maxy;

	min_x_b = elts[b].minx;
	min_y_b = elts[b].miny;
	max_x_b = elts[b].maxx;
	max_y_b = elts[b].maxy;
	
	if (min_x_a < min_x_b)
	{
		minx = min_x_a;
	} 
	else
	{
		minx = min_x_b;
	}
	if (min_y_a < min_y_b)
	{
		miny = min_y_a;
	} 
	else
	{
		miny = min_y_b;
	}
	if (max_x_a > max_x_b)
	{
		maxx = max_x_a;
	} 
	else
	{
		maxx = max_x_b;
	}
	if (max_y_a > max_y_b)
	{
		maxy = max_y_a;
	} 
	else
	{
		maxy = max_y_b;
	}
	//cout << max_x_a <<" " << max_x_b <<" " << maxx << endl;
	//cout << min_x_a <<" " << min_x_b <<" " << minx << endl;
	//cout << max_y_a <<" " << max_y_b <<" " << maxy << endl;
	//cout << min_y_a <<" " << min_y_b <<" " << miny << endl;
}

void universe::showcomplist(int height, int width, image<rgb> *input_edge)
{
	int sum = 0;
	int i = 0;
	bool edge_flag;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			//int y = 119;
			//int x = 79;
			i = y * width + x;
			i = find(i);
			sum = sum + complists[i].size();
			if (complists[i].size() > 0)
			{
				if (imRef(input_edge, x, y).r == 255 && imRef(input_edge, x, y).g == 255 && imRef(input_edge, x, y).b == 255)
				{
					edge_flag = true;
				}
				else
				{
					edge_flag = false;
				}
				if (edge_flag == false)
				{
					float ratio;
					ratio = (float(elts[i].maxy) - float(elts[i].miny)) / (float(elts[i].maxx)- float(elts[i].minx));
					std::cout<<" "<< elts[i].maxy <<" "<< elts[i].miny <<" "<< elts[i].maxx <<" "<< elts[i].minx <<" "<< complists[i].size() <<" " << ratio << endl;
				
					LISTPOS::iterator it;
					int *comp_flag;
					comp_flag = new int[height*width];
					for (it = complists[i].begin(); it != complists[i].end(); it++)
					{
						comp_flag[it->i] = 1;
					}

					//convert segment result from vector to mat
					Mat src_show(height, width, CV_8UC3);
					Vec3b *pp;
					for (int y = 0; y < height; y++) {
						pp = src_show.ptr<Vec3b>(y);
						for (int x = 0; x < width; x++) {
							if (comp_flag[y*width+x] == 1)
							{
								pp[x][0] = 255;
								pp[x][1] = 255;
								pp[x][2] = 255;
							}
							if (y*width+x == i)
							{
								pp[x][0] = 0;
								pp[x][1] = 0;
								pp[x][2] = 255;
							}
						}
					}
					Point p1;
					Point p2;
					p1.x = elts[i].minx;
					p1.y = elts[i].miny;
					p2.x = elts[i].maxx;
					p2.y = elts[i].maxy;
					rectangle(src_show, p1, p2,CV_RGB(0,255,150));
					namedWindow("debug show", WINDOW_AUTOSIZE);
					imshow("debug show", src_show);
					waitKey(0);
					destroyWindow("debug show");
					src_show.release();
				}

			}

		}
	}
	std::cout << sum << endl;
	std::cout << height << width << endl;

}

void universe::showJoinedcomplist(int a, int height, int width, image<rgb> *input_edge)
{
	int sum = 0;
	int i = 0;
	bool edge_flag;
	//for (int y = 0; y < height; y++) {
		//for (int x = 0; x < width; x++) {
			//int y = 119;
			//int x = 79;
			//i = y * width + x;
			i = find(a);
			float ratio;
			ratio = (float(elts[i].maxy) - float(elts[i].miny)) / (float(elts[i].maxx)- float(elts[i].minx));
			std::cout<<" "<< elts[i].maxy <<" "<< elts[i].miny <<" "<< elts[i].maxx <<" "<< elts[i].minx <<" "<< complists[i].size() <<" " << ratio << endl;
			LISTPOS::iterator it;
			int *comp_flag;
			comp_flag = new int[height*width];
			for (it = complists[i].begin(); it != complists[i].end(); it++)
			{
				comp_flag[it->i] = 1;
			}

			//convert segment result from vector to mat
			Mat src_show(height, width, CV_8UC3);
			Vec3b *pp;
			for (int y = 0; y < height; y++) {
				pp = src_show.ptr<Vec3b>(y);
				for (int x = 0; x < width; x++) {
					if (comp_flag[y*width+x] == 1)
					{
						pp[x][0] = 255;
						pp[x][1] = 255;
						pp[x][2] = 255;
					}
					if (y*width+x == i)
					{
						pp[x][0] = 0;
						pp[x][1] = 0;
						pp[x][2] = 255;
					}
				}
			}
			Point p1;
			Point p2;
			p1.x = elts[i].minx;
			p1.y = elts[i].miny;
			p2.x = elts[i].maxx;
			p2.y = elts[i].maxy;
			if (complists[i].size() > 1)
			{
				rectangle(src_show, p1, p2,CV_RGB(0,255,150));
			}
			namedWindow("debug show", WINDOW_AUTOSIZE);
			imshow("debug show", src_show);
			waitKey(0);
			destroyWindow("debug show");
			src_show.release();

		//}
	//}
}

// give one pixel, find the components of 8 non edge neighbor pixels
void universe::Find8NeighborComp(int x, int y, vector<int> &neighbor_comp, int height, int width, image<rgb> *input_edge)
{
	int i = 0;
	if ((y-1) >= 0 && (y-1) < height && (x-1) >= 0 && (x-1) < width)
	{
		i = (y-1) * width + x-1;
		if (imRef(input_edge, x-1, y-1).r != 255 ||
			imRef(input_edge, x-1, y-1).g != 255 || 
			imRef(input_edge, x-1, y-1).b != 255)
		{
			neighbor_comp.push_back(find(i));
		}
	}

	if ((y-1) >= 0 && (y-1) < height && x >= 0 && x < width)
	{
		i = (y-1) * width + x;
		if (imRef(input_edge, x, y-1).r != 255 ||
			imRef(input_edge, x, y-1).g != 255 || 
			imRef(input_edge, x, y-1).b != 255)
		{
			neighbor_comp.push_back(find(i));
		}
	}

	if ((y-1) >= 0 && (y-1) < height && (x+1) >= 0 && (x+1) < width)
	{
		i = (y-1) * width + x+1;
		if (imRef(input_edge, x+1, y-1).r != 255 ||
			imRef(input_edge, x+1, y-1).g != 255 || 
			imRef(input_edge, x+1, y-1).b != 255)
		{
			neighbor_comp.push_back(find(i));
		}
	}

	if (y >= 0 && y < height && (x-1) >= 0 && (x-1) < width)
	{
		i = y * width + x-1;
		if (imRef(input_edge, x-1, y).r != 255 ||
			imRef(input_edge, x-1, y).g != 255 || 
			imRef(input_edge, x-1, y).b != 255)
		{
			neighbor_comp.push_back(find(i));
		}
	}

	if (y >= 0 && y < height && (x+1) >= 0 && (x+1) < width)
	{
		i = y * width + x+1;
		if (imRef(input_edge, x+1, y).r != 255 ||
			imRef(input_edge, x+1, y).g != 255 || 
			imRef(input_edge, x+1, y).b != 255)
		{
			neighbor_comp.push_back(find(i));
		}
	}

	if ((y+1) >= 0 && (y+1) < height && (x-1) >= 0 && (x-1) < width)
	{
		i = (y+1) * width + x-1;
		if (imRef(input_edge, x-1, y+1).r != 255 ||
			imRef(input_edge, x-1, y+1).g != 255 || 
			imRef(input_edge, x-1, y+1).b != 255)
		{
			neighbor_comp.push_back(find(i));
		}
	}

	if ((y+1) >= 0 && (y+1) < height && x >= 0 && x < width)
	{
		i = (y+1) * width + x;
		if (imRef(input_edge, x, y+1).r != 255 ||
			imRef(input_edge, x, y+1).g != 255 || 
			imRef(input_edge, x, y+1).b != 255)
		{
			neighbor_comp.push_back(find(i));
		}
	}

	if ((y+1) >= 0 && (y+1) < height && (x+1) >= 0 && (x+1) < width)
	{
		i = (y+1) * width + x+1;
		if (imRef(input_edge, x+1, y+1).r != 255 ||
			imRef(input_edge, x+1, y+1).g != 255 || 
			imRef(input_edge, x+1, y+1).b != 255)
		{
			neighbor_comp.push_back(find(i));
		}
	}
}

// get region from feature extraction (neighbor components)
void universe::GetRegion(int height, int width, image<rgb> *input_edge, image<rgb> *input_im)
{
	int i = 0;
	LISTPOS::iterator it;
	vector<int> neighbor_comp;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			if (imRef(input_edge, x, y).r == 255 &&
				imRef(input_edge, x, y).g == 255 && 
				imRef(input_edge, x, y).b == 255)
			{
				Find8NeighborComp(x, y, neighbor_comp, height, width, input_edge);

				// remove duplicate elements to find unique neighbor components 
				if(neighbor_comp.size() > 1)
				{
					sort(neighbor_comp.begin(),neighbor_comp.end());
					vector<int>::iterator pos;
					pos = unique(neighbor_comp.begin(), neighbor_comp.end());
					neighbor_comp.erase(pos, neighbor_comp.end());
				}
				
				Mat comp_flag(height, width, CV_8UC3, Scalar(0,0,0));
				cout << neighbor_comp.size() << endl;

				// mark the pixel red
				comp_flag.at<Vec3b>(y,x)[0] = 0;
				comp_flag.at<Vec3b>(y,x)[1] = 0;
				comp_flag.at<Vec3b>(y,x)[2] = 255;

				// show all the neighbor components of this pixel
				for (vector<int>::iterator iter = neighbor_comp.begin(); iter != neighbor_comp.end(); ++iter)
				{
					int randr, randg, randb;
					randr = (uchar)rand();
					randg = (uchar)rand();
					randb = (uchar)rand();
					for (it = complists[*iter].begin(); it != complists[*iter].end(); it++)
					{
						comp_flag.at<Vec3b>(it->y, it->x)[0] = imRef(input_im, it->x, it->y).b;
						comp_flag.at<Vec3b>(it->y, it->x)[1] = imRef(input_im, it->x, it->y).g;
						comp_flag.at<Vec3b>(it->y, it->x)[2] = imRef(input_im, it->x, it->y).r;
					}
				}
				
				namedWindow("neighbor comp show", WINDOW_AUTOSIZE);
				imshow("neighbor comp show", comp_flag);
				waitKey(0);
				destroyWindow("neighbor comp show");
				neighbor_comp.clear();
			}
		}
	}
}

// show region from feature extraction (far neighbor components)
void universe::GetRegion2(int height, int width, image<rgb> *input_edge, image<rgb> *input_im)
{
	int i = 0;
	LISTPOS::iterator it;
	vector<int> neighbor_comps;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			if (imRef(input_edge, x, y).r == 255 &&
				imRef(input_edge, x, y).g == 255 && 
				imRef(input_edge, x, y).b == 255)
			{
				// find comps of 8 neighbors
				Find8NeighborComp(x, y, neighbor_comps, height, width, input_edge);

				// remove duplicate elements to find unique neighbor components 
				if(neighbor_comps.size() > 1)
				{
					sort(neighbor_comps.begin(),neighbor_comps.end());
					vector<int>::iterator pos;
					pos = unique(neighbor_comps.begin(), neighbor_comps.end());
					neighbor_comps.erase(pos, neighbor_comps.end());
				}

				Mat comp_show(height, width, CV_8UC3, Scalar(0,0,0));
				cout << neighbor_comps.size() << endl;

				// mark the pixel red
				comp_show.at<Vec3b>(y,x)[0] = 0;
				comp_show.at<Vec3b>(y,x)[1] = 0;
				comp_show.at<Vec3b>(y,x)[2] = 255;

				// show all the neighbor components of this pixel
				for (vector<int>::iterator iter = neighbor_comps.begin(); iter != neighbor_comps.end(); ++iter)
				{
					int randr, randg, randb;
					randr = (uchar)rand();
					randg = (uchar)rand();
					randb = (uchar)rand();
					for (it = complists[*iter].begin(); it != complists[*iter].end(); it++)
					{
						comp_show.at<Vec3b>(it->y, it->x)[0] = imRef(input_im, it->x, it->y).b;
						comp_show.at<Vec3b>(it->y, it->x)[1] = imRef(input_im, it->x, it->y).g;
						comp_show.at<Vec3b>(it->y, it->x)[2] = imRef(input_im, it->x, it->y).r;
					}
				}

				namedWindow("neighbor comp show", WINDOW_AUTOSIZE);
				imshow("neighbor comp show", comp_show);
				waitKey(0);

				// find all comp of 8 neighbor of all contours
				vector<int> far_neighbor_comps;
				for (vector<int>::iterator iter = neighbor_comps.begin(); iter != neighbor_comps.end(); ++iter)
				{
					Mat comp_flag(height, width, CV_8UC1, Scalar(0));
					vector<vector<Point>> contours;
					for (it = complists[*iter].begin(); it != complists[*iter].end(); it++)
					{
						comp_flag.at<uchar>(it->y, it->x) = 255;
					}
					findContours(comp_flag, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
					drawContours(comp_flag, contours, -1, Scalar(255));
					if (contours.size() > 0 && contours[0].size() > 0)
					{
						for (vector<Point>:: iterator it_p = contours[0].begin(); it_p != contours[0].end(); ++it_p)
						{
							Find8NeighborComp(it_p->x, it_p->y, far_neighbor_comps, height, width, input_edge);
						}
					}
				}

				neighbor_comps.insert(neighbor_comps.end(), far_neighbor_comps.begin(), far_neighbor_comps.end());

				// remove duplicate elements to find unique neighbor components 
				if(neighbor_comps.size() > 1)
				{
					sort(neighbor_comps.begin(),neighbor_comps.end());
					vector<int>::iterator pos;
					pos = unique(neighbor_comps.begin(), neighbor_comps.end());
					neighbor_comps.erase(pos, neighbor_comps.end());
				}

				// show all the neighbor components of this pixel
				for (vector<int>::iterator iter = neighbor_comps.begin(); iter != neighbor_comps.end(); ++iter)
				{
					int randr, randg, randb;
					randr = (uchar)rand();
					randg = (uchar)rand();
					randb = (uchar)rand();
					for (it = complists[*iter].begin(); it != complists[*iter].end(); it++)
					{
						comp_show.at<Vec3b>(it->y, it->x)[0] = imRef(input_im, it->x, it->y).b;
						comp_show.at<Vec3b>(it->y, it->x)[1] = imRef(input_im, it->x, it->y).g;
						comp_show.at<Vec3b>(it->y, it->x)[2] = imRef(input_im, it->x, it->y).r;
					}
				}

				namedWindow("neighbor comp show", WINDOW_AUTOSIZE);
				imshow("neighbor comp show", comp_show);
				waitKey(0);
				destroyWindow("neighbor comp show");

				neighbor_comps.clear();
				far_neighbor_comps.clear();
			}
		}
	}
}

void universe::SaveWindowImage(int r, int window_minx, int window_miny, int window_maxx, int window_maxy, Mat src_image, string ifile_name, string dir,
	int &cnt_window, int &cnt_r, int &cnt_s, Vec3b *lp, int x, int y)
{
	// save window for feature extraction
	Mat cropedwindow = src_image(Rect(window_minx, window_miny, window_maxx-window_minx, window_maxy-window_miny));
	//cropedwindow.convertTo(cropedwindow, CV_16UC3);
	//Mat window_image(2*r, 2*r, CV_16UC3, Scalar(499,500,501));
	Mat window_image(2*r, 2*r, CV_8UC3, Scalar(0,0,0));
	int x_tmp, y_tmp;	// left-up point of croped window in r*r
	int delt_x = window_maxx-window_minx;
	int delt_y = window_maxy-window_miny;
	if (delt_x != 2*r || delt_y != 2*r)
	{
		// if window contain out border pixels, do not save
		return;
		if (window_minx == 0)
		{
			x_tmp = 2*r - delt_x;
		} 
		else
		{
			x_tmp = 0;
		}
		if (window_miny == 0)
		{
			y_tmp = 2*r - delt_y;
		} 
		else
		{
			y_tmp = 0;
		}
	}
	else
	{
		x_tmp = 0;
		y_tmp = 0;
	}
	Mat roi(window_image, Rect(x_tmp, y_tmp, delt_x, delt_y));
	cropedwindow.copyTo(roi);
	//namedWindow("window", WINDOW_AUTOSIZE);
	//imshow("window", window_image);
	//waitKey(0);
	//destroyWindow("window");
	// save window image
	string window_file_name;
	string str_window_minx, str_window_miny, str_delt_x, str_delt_y, str_x, str_y;
	stringstream stream;
	stream << setw(3) << setfill('0') << window_minx; stream >> str_window_minx; stream.clear();
	stream << setw(3) << setfill('0')  << window_miny; stream >> str_window_miny; stream.clear();
	stream << setw(3) << setfill('0')  << delt_x; stream >> str_delt_x; stream.clear();
	stream << setw(3) << setfill('0')  << delt_y; stream >> str_delt_y; stream.clear();
	stream << setw(3) << setfill('0')  << x; stream >> str_x; stream.clear();
	stream << setw(3) << setfill('0')  << y; stream >> str_y; stream.clear();
	window_file_name = ifile_name + "_" + 
					   str_x + "_" + 
					   str_y + "_" + 
					   str_window_minx + "_" + 
					   str_window_miny + "_" + 
					   str_delt_x + "_" + 
					   str_delt_y;
	if (lp[x][2] == 255)
	{
		window_file_name = window_file_name + "_r.png";
		cnt_r++;
	} 
	else if (lp[x][1] == 255)
	{
		window_file_name = window_file_name + "_s.png";
		cnt_s++;
	}
	cnt_window++;
	imwrite(dir + window_file_name, window_image);
	window_file_name.clear();
}

// get region from feature extraction (far neighbor components)
void universe::GetRegion3(int height, int width, image<rgb> *input_edge, image<rgb> *input_im, Mat gradient_label,
	Mat src_image, string ifile_name)
{
	int i = 0;
	LISTPOS::iterator it;
	vector<int> neighbor_comps;
	Vec3b *lp;
	int strd = 3;
	// count window_fix number
	int cnt_window0, cnt_r0, cnt_s0;
	cnt_window0 = 0; cnt_r0 = 0; cnt_s0 = 0;
	// count window_inside number
	int cnt_window1, cnt_r1, cnt_s1;
	cnt_window1 = 0; cnt_r1 = 0; cnt_s1 = 0;
	// count window_outside number
	int cnt_window2, cnt_r2, cnt_s2;
	cnt_window2 = 0; cnt_r2 = 0; cnt_s2 = 0;
	string dir;
	Point p1, p2;
	//Mat simple_show(height, width, CV_8UC3, Scalar(0,0,0));
	//Vec3b *lp_show;
	for (int y = 0; y < height; y = y + strd) {
		lp = gradient_label.ptr<Vec3b>(y);
		//lp_show = simple_show.ptr<Vec3b>(y);
		for (int x = 0; x < width; x = x + strd) {
			if (lp[x][2] == 255 || lp[x][1] == 255)	
			{
				//if (lp[x][2] == 255)
				//{
				//	lp_show[x][2] = 255;
				//}
				//else if (lp[x][1] == 255)
				//{
				//	lp_show[x][1] = 255;
				//}

				// extract fixed size windows
				int r = 18;
				int window_minx, window_miny, window_maxx, window_maxy;
				if (x - r > 0)
				{
					window_minx = x - r;
				} 
				else
				{
					window_minx = 0;
				}
				if (y - r > 0)
				{
					window_miny = y - r;
				} 
				else
				{
					window_miny = 0;
				}
				if (x + r < width)
				{
					window_maxx = x + r;
				} 
				else
				{
					window_maxx = width;
				}
				if (y + r < height)
				{
					window_maxy = y + r;
				} 
				else
				{
					window_maxy = height;
				}
				// show window
				p1.x = window_minx;
				p1.y = window_miny;
				p2.x = window_maxx;
				p2.y = window_maxy;
				dir = "D:/zw/segment/windows_fixed/";
				SaveWindowImage(r, window_minx, window_miny, window_maxx, window_maxy, src_image, ifile_name, dir,
					cnt_window0, cnt_r0, cnt_s0, lp, x, y);

	//			// find comps of 8 neighbors
	//			Find8NeighborComp(x, y, neighbor_comps, height, width, input_edge);

	//			// remove duplicate elements to find unique neighbor components 
	//			if(neighbor_comps.size() > 1)
	//			{
	//				sort(neighbor_comps.begin(),neighbor_comps.end());
	//				vector<int>::iterator pos;
	//				pos = unique(neighbor_comps.begin(), neighbor_comps.end());
	//				neighbor_comps.erase(pos, neighbor_comps.end());
	//			}

	//			// show first level neighbor components
	//			Mat comp_show(height, width, CV_8UC3, Scalar(0,0,0));
	//			//cout << neighbor_comps.size() << endl;
	//			// mark the pixel red
	//			//comp_show.at<Vec3b>(y,x)[0] = 0;
	//			//comp_show.at<Vec3b>(y,x)[1] = 0;
	//			//comp_show.at<Vec3b>(y,x)[2] = 255;
	//			// show all the neighbor components of this pixel
	//			vector<int> minxs, minys, maxxs, maxys;
	//			for (vector<int>::iterator iter = neighbor_comps.begin(); iter != neighbor_comps.end(); ++iter)
	//			{
	//				minxs.push_back(elts[*iter].minx);
	//				minys.push_back(elts[*iter].miny);
	//				maxxs.push_back(elts[*iter].maxx);
	//				maxys.push_back(elts[*iter].maxy);
	//				//for (it = complists[*iter].begin(); it != complists[*iter].end(); it++)
	//				//{
	//				//	comp_show.at<Vec3b>(it->y, it->x)[0] = imRef(input_im, it->x, it->y).b;
	//				//	comp_show.at<Vec3b>(it->y, it->x)[1] = imRef(input_im, it->x, it->y).g;
	//				//	comp_show.at<Vec3b>(it->y, it->x)[2] = imRef(input_im, it->x, it->y).r;
	//				//}
	//			}
	//			int neighbor_minx = x;
	//			int neighbor_miny = y;
	//			int neighbor_maxx = x;
	//			int neighbor_maxy = y;
	//			if (minxs.size() > 0 && minys.size() > 0 && maxxs.size() > 0 && maxys.size() > 0)
	//			{
	//				neighbor_minx = *min_element(minxs.begin(), minxs.end());
	//				neighbor_miny = *min_element(minys.begin(), minys.end());
	//				neighbor_maxx = *max_element(maxxs.begin(), maxxs.end());
	//				neighbor_maxy = *max_element(maxys.begin(), maxys.end());
	//			}
	//			p1.x = neighbor_minx;
	//			p1.y = neighbor_miny;
	//			p2.x = neighbor_maxx;
	//			p2.y = neighbor_maxy;
	//			//rectangle(comp_show, p1, p2,CV_RGB(0,255,150));
	//			// calc window size
	//			vector<int> rs;
	//			rs.push_back(abs(neighbor_minx - x)); rs.push_back(abs(neighbor_miny - y));
	//			rs.push_back(abs(neighbor_maxx - x)); rs.push_back(abs(neighbor_maxy - y));
	//			r = *max_element(rs.begin(), rs.end());
	//			if (r < 22)
	//			{
	//				r = 22;
	//			}
	//			if (x - r > 0)
	//			{
	//				window_minx = x - r;
	//			} 
	//			else
	//			{
	//				window_minx = 0;
	//			}
	//			if (y - r > 0)
	//			{
	//				window_miny = y - r;
	//			} 
	//			else
	//			{
	//				window_miny = 0;
	//			}
	//			if (x + r < width)
	//			{
	//				window_maxx = x + r;
	//			} 
	//			else
	//			{
	//				window_maxx = width;
	//			}
	//			if (y + r < height)
	//			{
	//				window_maxy = y + r;
	//			} 
	//			else
	//			{
	//				window_maxy = height;
	//			}
	//			// show window
	//			p1.x = window_minx;
	//			p1.y = window_miny;
	//			p2.x = window_maxx;
	//			p2.y = window_maxy;
	//			//rectangle(comp_show, p1, p2,CV_RGB(255,0,150));
	//			minxs.clear(); minys.clear(); maxxs.clear(); maxys.clear();
	//			rs.clear();
	//			//namedWindow("neighbor comp show", WINDOW_AUTOSIZE);
	//			//imshow("neighbor comp show", comp_show);
	//			//waitKey(0);

	//			dir = "D:/zw/segment/windows_inside_new/";
	//			SaveWindowImage(r, window_minx, window_miny, window_maxx, window_maxy, src_image, ifile_name, dir,
	//				cnt_window1, cnt_r1, cnt_s1, lp, x, y);

	//			// ---- find all comp of 8 neighbor of all contours ----
	//			vector<int> far_neighbor_comps;
	//			for (vector<int>::iterator iter = neighbor_comps.begin(); iter != neighbor_comps.end(); ++iter)
	//			{
	//				Mat comp_flag(height, width, CV_8UC1, Scalar(0));
	//				vector<vector<Point>> contours;
	//				for (it = complists[*iter].begin(); it != complists[*iter].end(); it++)
	//				{
	//					comp_flag.at<uchar>(it->y, it->x) = 255;
	//				}
	//				findContours(comp_flag, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	//				//drawContours(comp_flag, contours, -1, Scalar(255));
	//				if (contours.size() > 0 && contours[0].size() > 0)
	//				{
	//					for (vector<Point>:: iterator it_p = contours[0].begin(); it_p != contours[0].end(); ++it_p)
	//					{
	//						Find8NeighborComp(it_p->x, it_p->y, far_neighbor_comps, height, width, input_edge);
	//					}
	//				}
	//			}

	//			neighbor_comps.insert(neighbor_comps.end(), far_neighbor_comps.begin(), far_neighbor_comps.end());

	//			// remove duplicate elements to find unique neighbor components 
	//			if(neighbor_comps.size() > 1)
	//			{
	//				sort(neighbor_comps.begin(),neighbor_comps.end());
	//				vector<int>::iterator pos;
	//				pos = unique(neighbor_comps.begin(), neighbor_comps.end());
	//				neighbor_comps.erase(pos, neighbor_comps.end());
	//			}

	//			// show all the neighbor components of this pixel
	//			for (vector<int>::iterator iter = neighbor_comps.begin(); iter != neighbor_comps.end(); ++iter)
	//			{
	//				minxs.push_back(elts[*iter].minx);
	//				minys.push_back(elts[*iter].miny);
	//				maxxs.push_back(elts[*iter].maxx);
	//				maxys.push_back(elts[*iter].maxy);
	//				//for (it = complists[*iter].begin(); it != complists[*iter].end(); it++)
	//				//{
	//				//	comp_show.at<Vec3b>(it->y, it->x)[0] = imRef(input_im, it->x, it->y).b;
	//				//	comp_show.at<Vec3b>(it->y, it->x)[1] = imRef(input_im, it->x, it->y).g;
	//				//	comp_show.at<Vec3b>(it->y, it->x)[2] = imRef(input_im, it->x, it->y).r;
	//				//}
	//			}
	//			neighbor_minx = x;
	//			neighbor_miny = y;
	//			neighbor_maxx = x;
	//			neighbor_maxy = y;
	//			if (minxs.size() > 0 && minys.size() > 0 && maxxs.size() > 0 && maxys.size() > 0)
	//			{
	//				neighbor_minx = *min_element(minxs.begin(), minxs.end());
	//				neighbor_miny = *min_element(minys.begin(), minys.end());
	//				neighbor_maxx = *max_element(maxxs.begin(), maxxs.end());
	//				neighbor_maxy = *max_element(maxys.begin(), maxys.end());
	//			}
	//			p1.x = neighbor_minx;
	//			p1.y = neighbor_miny;
	//			p2.x = neighbor_maxx;
	//			p2.y = neighbor_maxy;
	//			//rectangle(comp_show, p1, p2,CV_RGB(0,255,150));
	//			// calc window size
	//			rs.push_back(abs(neighbor_minx - x)); rs.push_back(abs(neighbor_miny - y));
	//			rs.push_back(abs(neighbor_maxx - x)); rs.push_back(abs(neighbor_maxy - y));
	//			r = *max_element(rs.begin(), rs.end());
	//			if (r < 34)
	//			{
	//				r = 34;
	//			}
	//			if (x - r > 0)
	//			{
	//				window_minx = x -r;
	//			} 
	//			else
	//			{
	//				window_minx = 0;
	//			}
	//			if (y - r > 0)
	//			{
	//				window_miny = y - r;
	//			} 
	//			else
	//			{
	//				window_miny = 0;
	//			}
	//			if (x + r < width)
	//			{
	//				window_maxx = x + r;
	//			} 
	//			else
	//			{
	//				window_maxx = width;
	//			}
	//			if (y + r < height)
	//			{
	//				window_maxy = y + r;
	//			} 
	//			else
	//			{
	//				window_maxy = height;
	//			}
	//			// show window
	//			p1.x = window_minx;
	//			p1.y = window_miny;
	//			p2.x = window_maxx;
	//			p2.y = window_maxy;
	//			//rectangle(comp_show, p1, p2,CV_RGB(255,0,150));
	//			minxs.clear(); minys.clear(); maxxs.clear(); maxys.clear();
	//			rs.clear();
	//			//namedWindow("neighbor comp show", WINDOW_AUTOSIZE);
	//			//imshow("neighbor comp show", comp_show);
	//			//waitKey(0);

	//			dir = "D:/zw/segment/windows_outside_new/";
	//			SaveWindowImage(r, window_minx, window_miny, window_maxx, window_maxy, src_image, ifile_name, dir,
	//				cnt_window2, cnt_r2, cnt_s2, lp, x, y);
	//			// clear comps and far comps records
	//			neighbor_comps.clear();
	//			far_neighbor_comps.clear();
			}
		}
	}
	//namedWindow("simple show", WINDOW_AUTOSIZE);
	//imshow("simple show", simple_show);
	//waitKey(0);
	cout << "window num: " << cnt_window0 << " r: " << cnt_r0 << " s: " << cnt_s0 << endl;
	cout << "window num: " << cnt_window1 << " r: " << cnt_r1 << " s: " << cnt_s1 << endl;
	cout << "window num: " << cnt_window2 << " r: " << cnt_r2 << " s: " << cnt_s2 << endl;
}

#endif