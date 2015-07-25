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

#include <cstdio>
#include <cstdlib>
#include "image.h"
#include "misc.h"
#include "pnmfile.h"
#include "segment-image.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <time.h>
#include "opencv2/contrib/contrib.hpp"
#include <string>
using namespace cv;
using namespace std;

int main(int argc, char **argv) {
  //Mat img_test;
  //img_test = imread("D:/zw/segment/red.png", CV_LOAD_IMAGE_COLOR);
  //cout << img_test.at<Vec3b>(1,1) << endl;
  //Mat im2(200, 300, CV_8UC3, Scalar(255,0,0));
  //cout << im2.at<Vec3b>(1,1)[0] << endl;
  //cout << im2.at<Vec3b>(1,1)[1] << endl;
  //cout << im2.at<Vec3b>(1,1)[2] << endl;
  //namedWindow("im2", WINDOW_AUTOSIZE);
  //imshow("im2", im2);
  //waitKey(0);
  //imwrite("red2.png", im2);

  cv::Directory dir;
  string dir_iiw_HKU_label = "D:/zw/segment/iiw-HKU-label/";
  string exten1 = "*_label.png";
  bool addPath1 = false;
  vector<string> filenames = dir.GetListFiles(dir_iiw_HKU_label, exten1, addPath1);
  cout << "total label number:" << filenames.size() << endl;

  // graph-based segment parameters
  float sigma = 0.5;
  float k = 50;
  int min_size = 10;
  // canny edge parameters
  int lowThreshold = 10;
  int ratio = 3;
  int kernel_size = 3;

  // open a txt file to record
  ofstream logfile("window_log.txt", ios::out);
  if (!logfile)
  {
	  cout << "cannot open log file" << endl;
  }

  for (int i = 0; i < filenames.size(); i++)
  {
	  string ifile_name, xmlfile_name, lfile_name, tmp_name;
	  tmp_name = filenames[i];
	  ifile_name = tmp_name.replace(tmp_name.end()-10, tmp_name.end()-4, "");
	  tmp_name = filenames[i];
	  xmlfile_name = tmp_name.replace(tmp_name.end()-10, tmp_name.end(), ".xml");
	  lfile_name = filenames[i];
	  cout << i << "/" << filenames.size() << "loading: " << ifile_name << "  " << xmlfile_name << "  " << lfile_name <<endl;
	  logfile << i << "/" << filenames.size() << "loading: " << ifile_name << "  " << xmlfile_name << "  " << lfile_name <<endl;

	  Mat img, src_gray, src_edge, dst;

	  img = imread(dir_iiw_HKU_label + ifile_name, CV_LOAD_IMAGE_COLOR);
	  if (! img.data)
	  {
		  cout << "Could not open or find the image!"<< ifile_name << endl;
		  waitKey(0);
		  return -1;
	  }
	  //namedWindow("orginal image", WINDOW_AUTOSIZE);
	  //imshow("orginal image", img);

	  Mat limage;
	  limage = imread(dir_iiw_HKU_label + lfile_name, CV_LOAD_IMAGE_COLOR);
	  if (!limage.data)
	  {
		  cout << "could not open or find the label image" << ifile_name << endl;
		  waitKey(0);
		  return -1;
	  }
	  //namedWindow("label", WINDOW_AUTOSIZE);
	  //imshow("label", limage);

	  int height = img.rows;
	  int width = img.cols;

	  cvtColor(img, src_gray, CV_BGR2GRAY);
	  blur(src_gray, src_edge, Size(3,3));
	  Canny(src_edge, src_edge, lowThreshold, lowThreshold*ratio, kernel_size);
	  Mat src_edge_rgb;
	  cvtColor(src_edge, src_edge_rgb, CV_GRAY2BGR);
	  //namedWindow("canny edge", WINDOW_AUTOSIZE);
	  //imshow("canny edge", src_edge_rgb);

	  Vec3b *p2;
	  for (int y = 0; y < height; y++)
	  {
		  p2 = src_edge_rgb.ptr<Vec3b>(y);
		  for (int x = 0; x < width; x++)
		  {
			  if (p2[x][0]!=0 && p2[x][0]!=255)
			  {
				  cout << p2[x][0] << endl;
			  }
			  if (p2[x][1]!=0 && p2[x][1]!=255)
			  {
				  cout << p2[x][1] << endl;
			  }
			  if (p2[x][2]!=0 && p2[x][2]!=255)
			  {
				  cout << p2[x][2] << endl;
			  }
			  p2[x][0] = p2[x][0] / 255;
			  p2[x][0] = p2[x][1] / 255;
			  p2[x][0] = p2[x][2] / 255;
		  }
	  }
	  Mat gradient_label;
	  src_edge_rgb.copyTo(gradient_label);
	  multiply(src_edge_rgb, limage, gradient_label);
	  //namedWindow("gradient_label", WINDOW_AUTOSIZE);
	  //imshow("gradient_label", gradient_label);
	  //waitKey(0);

	  //convert orginal image from mat to vector
	  image<rgb> *input_im = new image<rgb>(width,height);
	  Mat2Vector(img, input_im);

	  //convert edge image from mat to vector
	  image<rgb> *input_edge = new image<rgb>(width,height);
	  Mat2Vector(src_edge, input_edge);

 	  clock_t  clockBegin, clockEnd;    
	  clockBegin = clock();

	  printf("processing\n");
	  int num_ccs;
	  image<rgb> *seg = segment_image(input_im, sigma, k, min_size, &num_ccs, input_edge, gradient_label, img,
		  ifile_name.replace(ifile_name.end()-4, ifile_name.end(), ""));

	  clockEnd = clock();    
	  printf("%d\n", clockEnd - clockBegin);

	  //convert segment result from vector to mat
	  height = seg->height();
	  width = seg->width();
	  Mat src_seg(height, width, CV_8UC3);
	  Vec3b *pp;
	  for (int y = 0; y < height; y++) {
		  pp = src_seg.ptr<Vec3b>(y);
		  for (int x = 0; x < width; x++) {
			  pp[x][0] = imRef(seg, x, y).r;
			  pp[x][1] = imRef(seg, x, y).g;
			  pp[x][2] = imRef(seg, x, y).b;
		  }
	  }
	  //namedWindow("segment result", WINDOW_AUTOSIZE);
	  //imshow("segment result", src_seg);
	  //waitKey(0);

	  //savePPM(seg, "tree_rst.ppm");
	  //imwrite("canny_edge.png", src_edge);
	  imwrite("segment_res.png", src_seg);

	  //printf("got %d components\n", num_ccs);
  }
  logfile.close();
  printf("done! uff...thats hard work.\n");
  waitKey(0);

  return 0;
}

