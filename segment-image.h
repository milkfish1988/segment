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

#ifndef SEGMENT_IMAGE
#define SEGMENT_IMAGE

#include <cstdlib>
#include "image.h"
#include "misc.h"
#include "filter.h"
#include "segment-graph.h"
#include <stdlib.h>
#include <iostream>

// random color
rgb random_rgb(){ 
  rgb c;
  double r;
  
  c.r = (uchar)rand();
  c.g = (uchar)rand();
  c.b = (uchar)rand();

  return c;
}

// dissimilarity measure between pixels
static inline float diff(image<float> *r, image<float> *g, image<float> *b,
			 int x1, int y1, int x2, int y2) {
  return sqrt(square(imRef(r, x1, y1)-imRef(r, x2, y2)) +
	      square(imRef(g, x1, y1)-imRef(g, x2, y2)) +
	      square(imRef(b, x1, y1)-imRef(b, x2, y2)));
}

// convert mat to vector
void Mat2Vector(Mat &mat_image, image<rgb> *vec_image)
{
	int height = mat_image.rows;
	int width = mat_image.cols;
	rgb tmp;
	if (mat_image.channels() == 1)
	{
		uchar *p;
		for (int y = 0; y < height; y++) {
			p = mat_image.ptr<uchar>(y);
			for (int x = 0; x < width; x++) {
				tmp.r = p[x];
				tmp.g = p[x];
				tmp.b = p[x];
				imRef(vec_image, x, y) = tmp;
			}
		}
	} 
	else if (mat_image.channels() == 3)
	{
		Vec3b *p;
		for (int y = 0; y < height; y++) {
			p = mat_image.ptr<Vec3b>(y);
			for (int x = 0; x < width; x++) {
				tmp.r = p[x][2];
				tmp.g = p[x][1];
				tmp.b = p[x][0];
				imRef(vec_image, x, y) = tmp;
			}
		}
	}
}

/*
 * Segment an image
 *
 * Returns a color image representing the segmentation.
 *
 * im: image to segment.
 * sigma: to smooth the image.
 * c: constant for threshold function.
 * min_size: minimum component size (enforced by post-processing stage).
 * num_ccs: number of connected components in the segmentation.
 */
image<rgb> *segment_image(image<rgb> *im, float sigma, float c, int min_size,
			  int *num_ccs, image<rgb> *input_edge, Mat gradient_label, Mat src_image, string ifile_name) {
  int width = im->width();
  int height = im->height();

  image<float> *r = new image<float>(width, height);
  image<float> *g = new image<float>(width, height);
  image<float> *b = new image<float>(width, height);

  // smooth each color channel  
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      imRef(r, x, y) = imRef(im, x, y).r;
      imRef(g, x, y) = imRef(im, x, y).g;
      imRef(b, x, y) = imRef(im, x, y).b;
    }
  }
  image<float> *smooth_r = smooth(r, sigma);
  image<float> *smooth_g = smooth(g, sigma);
  image<float> *smooth_b = smooth(b, sigma);
  delete r;
  delete g;
  delete b;
 
  // build graph
  edge *edges = new edge[width*height*4];
  int num = 0;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      if (x < width-1) {
	edges[num].a = y * width + x;
	edges[num].b = y * width + (x+1);
	edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x+1, y);
	edges[num].ax = x;
	edges[num].ay = y;
	edges[num].bx = x+1;
	edges[num].by = y;
	edges[num].a_edge_val = imRef(input_edge, x, y);
	edges[num].b_edge_val = imRef(input_edge, x+1, y);
	num++;
      }

      if (y < height-1) {
	edges[num].a = y * width + x;
	edges[num].b = (y+1) * width + x;
	edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x, y+1);
	edges[num].ax = x;
	edges[num].ay = y;
	edges[num].bx = x;
	edges[num].by = y+1;
	edges[num].a_edge_val = imRef(input_edge, x, y);
	edges[num].b_edge_val = imRef(input_edge, x, y+1);
	num++;
      }

      if ((x < width-1) && (y < height-1)) {
	edges[num].a = y * width + x;
	edges[num].b = (y+1) * width + (x+1);
	edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x+1, y+1);
	edges[num].ax = x;
	edges[num].ay = y;
	edges[num].bx = x+1;
	edges[num].by = y+1;
	edges[num].a_edge_val = imRef(input_edge, x, y);
	edges[num].b_edge_val = imRef(input_edge, x+1, y+1);
	num++;
      }

      if ((x < width-1) && (y > 0)) {
	edges[num].a = y * width + x;
	edges[num].b = (y-1) * width + (x+1);
	edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x+1, y-1);
	edges[num].ax = x;
	edges[num].ay = y;
	edges[num].bx = x+1;
	edges[num].by = y-1;
	edges[num].a_edge_val = imRef(input_edge, x, y);
	edges[num].b_edge_val = imRef(input_edge, x+1, y-1);
	num++;
      }
    }
  }
  delete smooth_r;
  delete smooth_g;
  delete smooth_b;

  // segment
  universe *u = segment_graph(width*height, num, edges, c, input_edge, height, width);

  int **comp_idx = new int *[height];
  FindAllFather(u, height, width, comp_idx);

  bool flag_ratio;
  bool flag_accountratio;
  // post process small components
  for (int i = 0; i < num; i++) {
	  //std::cout << i << "/" << num << std::endl;
	  int a = u->find(edges[i].a);
	  int b = u->find(edges[i].b);
	  edge *pedge = &edges[i];

	  // check the ratio if join comp a and comp b
	  int join_minx, join_miny, join_maxx, join_maxy;
	  int sizea, sizeb;
	  u->CalcJoinedBox(pedge->ax, pedge->ay, pedge->bx, pedge->by, join_minx, join_miny, join_maxx, join_maxy, height, width, sizea, sizeb);
	  float ratio;
	  ratio = (float(join_maxy) - float(join_miny)) / (float(join_maxx)- float(join_minx));
	  float accountratio;
	  accountratio = (sizea + sizeb) / ((float(join_maxy) - float(join_miny)) * (float(join_maxx)- float(join_minx)));
	  if ( ratio > 1.2 || ratio < 0.83)
	  {
		  flag_ratio = false;
	  } 
	  else
	  {
		  flag_ratio = true;
	  }
	  if (accountratio > 0.5)
	  {
		  flag_accountratio = true;
	  } 
	  else
	  {
		  flag_accountratio = false;
	  }

	  // check whether vertice are edge pixels
	  int flag = 
		  pedge->a_edge_val.r == 0 &&
		  pedge->a_edge_val.g == 0 &&
		  pedge->a_edge_val.b == 0 &&
		  pedge->b_edge_val.r == 0 &&
		  pedge->b_edge_val.g == 0 &&
		  pedge->b_edge_val.b == 0;

	  if ( ((a != b) && ((u->size(a) < min_size) || (u->size(b) < min_size))) && 
		  flag == 1 && flag_ratio == true && flag_accountratio == true )
	  {
		  u->UpdateJoinedBox(a, b, join_minx, join_miny, join_maxx, join_maxy);
		  u->join(a, b);
		  a = u->find(a);
	  }	  
  }
  delete [] edges;
  *num_ccs = u->num_sets();

  image<rgb> *output = new image<rgb>(width, height);

  // pick random colors for each component
  rgb *colors = new rgb[width*height];
  for (int i = 0; i < width*height; i++)
    colors[i] = random_rgb();
  
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int comp = u->find(y * width + x);
      imRef(output, x, y) = colors[comp];
    }
  }

  for (int y = 0; y < height; y++) {
	  for (int x = 0; x < width; x++) {
		  if(
			  imRef(input_edge, x, y).r == 255 &&
			  imRef(input_edge, x, y).g == 255 &&
			  imRef(input_edge, x, y).b == 255 )
		  {
			  imRef(output, x, y).r = 255;
			  imRef(output, x, y).g = 255;
			  imRef(output, x, y).b = 255;
		  }
		  
	  }
  }  

  u->GetRegion3(height, width, input_edge, im, gradient_label, src_image, ifile_name);

  delete [] colors;  
  delete u;

  return output;
}

#endif
