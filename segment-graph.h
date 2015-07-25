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

#ifndef SEGMENT_GRAPH
#define SEGMENT_GRAPH

#include <algorithm>
#include <cmath>
#include "disjoint-set.h"
#include <iostream>
#include <time.h>

// threshold function
#define THRESHOLD(size, c) (c/size)

typedef struct {
  float w;
  int a, b;
  int ax, ay, bx, by;
  rgb a_edge_val, b_edge_val;
} edge;

typedef struct {
	int min_x;
	int min_y;
	int max_x;
	int max_y;
} comp_box;

bool operator<(const edge &a, const edge &b) {
  return a.w < b.w;
}

// record each pixel belongs to which component
void FindAllFather(universe *u, int height, int width, int **comp_idx)
{
	for (int i = 0; i < height; ++i)
	{
		comp_idx[i] = new int[width];
	}
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			int comp = u->find(y * width + x);
			comp_idx[y][x] = comp;
		}
	}
}

/* given one component (corresponding father's idx), 
 * search the min max x y
 * in order to control component's aspect ratio during joining
 */
void FindComponentBoundingBox(universe *u, int height, int width, 
	int comp, int & min_x, int & max_x, int & min_y, int & max_y)
{
	//min_x = 1000;
	//min_y = 1000;
	//max_x = -1;
	//max_y = -1;
	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			int comp_tmp = u->find(y * width + x);
			if(comp_tmp == comp)
			{
				if ( y < min_y ) {min_y = y;}
				if ( y > max_y ) {max_y = y;}
				if ( x < min_x ) {min_x = x;}
				if ( x > max_x ) {max_x = x;}
			}
		}
	}
}

// calc the aspect ratio after join comp a and comp b
//comp_box CalcJoinedBox(comp_box **comp_boxes, int ax, int ay, int bx, int by)
//{
//	comp_box comp_box_joined;
//	int min_x_a, min_y_a, max_x_a, max_y_a;
//	int min_x_b, min_y_b, max_x_b, max_y_b;
//	int min_x, min_y, max_x, max_y;
//
//	min_x_a = comp_boxes[ay][ax].min_x;
//	min_y_a = comp_boxes[ay][ax].min_y;
//	max_x_a = comp_boxes[ay][ax].max_x;
//	max_y_a = comp_boxes[ay][ax].max_y;
//
//	min_x_b = comp_boxes[by][bx].min_x;
//	min_y_b = comp_boxes[by][bx].min_y;
//	max_x_b = comp_boxes[by][bx].max_x;
//	max_y_b = comp_boxes[by][bx].max_y;
//
//	if (min_x_a < min_x_b)
//	{
//		min_x = min_x_a;
//	} 
//	else
//	{
//		min_x = min_x_b;
//	}
//	if (min_y_a < min_y_b)
//	{
//		min_y = min_y_a;
//	} 
//	else
//	{
//		min_y = min_y_b;
//	}
//	if (max_x_a > max_x_b)
//	{
//		max_x = max_x_a;
//	} 
//	else
//	{
//		max_x = max_x_b;
//	}
//	if (max_y_a > max_y_b)
//	{
//		max_y = max_y_a;
//	} 
//	else
//	{
//		max_y = max_y_b;
//	}
//	//if ( max_x == min_x )
//	//{
//	//	std::cout << "x 0 !!!" << std::endl;
//	//}
//	comp_box_joined.min_x = min_x;
//	comp_box_joined.min_y = min_y;
//	comp_box_joined.max_x = max_x;
//	comp_box_joined.max_y = max_y;
//
//	return comp_box_joined;
//}

//void CalcAllCompBoxes(universe *u, int height, int width, comp_box **comp_boxes)
//{
//	for (int i = 0; i < height; ++i)
//	{
//		comp_boxes[i] = new comp_box[width];
//	}
//	for (int y = 0; y < height; y++) {
//		for (int x = 0; x < width; x++) {
//			int min_x, min_y, max_x, max_y;
//			min_x = 10000; min_y = 10000;
//			max_x = -1; max_y = -1;
//			int comp = u->find(y * width + x);
//			FindComponentBoundingBox(u, height, width, comp, min_x, max_x, min_y, max_y);
//			comp_boxes[y][x].max_x = max_x;
//			comp_boxes[y][x].max_y = max_y;
//			comp_boxes[y][x].min_x = min_x;
//			comp_boxes[y][x].min_y = min_y;
//		}
//	}
//}

void UpdateCompBoxes(universe *u, int comp_a, int comp_b, comp_box comp_box_joined, comp_box **comp_boxes, int height, int width)
{
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			int comp = u->find(y * width + x);
			if (comp == comp_a || comp == comp_b)
			{
				comp_boxes[y][x] = comp_box_joined;
			}
		}
	}
}

/*
 * Segment a graph
 *
 * Returns a disjoint-set forest representing the segmentation.
 *
 * num_vertices: number of vertices in graph.
 * num_edges: number of edges in graph
 * edges: array of edges.
 * c: constant for treshold function.
 */
universe *segment_graph(int num_vertices, int num_edges, edge *edges, 
			float c, image<rgb> *input_edge, int height, int width) { 
  // sort edges by weight
  std::sort(edges, edges + num_edges);

  // make a disjoint-set forest
  universe *u = new universe(num_vertices, height, width);

  // init thresholds
  float *threshold = new float[num_vertices];
  for (int i = 0; i < num_vertices; i++)
    threshold[i] = THRESHOLD(1,c);

  //clock_t  clockBegin, clockEnd;    
  //clockBegin = clock();

  bool flag_ratio;
  bool flag_accountratio;
  // for each edge, in non-decreasing weight order...
  for (int i = 0; i < num_edges; i++) {
	//std::cout << i << "/" << num_edges << std::endl;
    edge *pedge = &edges[i];
    
    // components conected by this edge
    int a = u->find(pedge->a);
    int b = u->find(pedge->b);

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

    if (a != b) {
        if ((pedge->w <= threshold[a]) &&
	       (pedge->w <= threshold[b]) && flag == 1 && flag_ratio == true && flag_accountratio == true) {
		  //u->showJoinedcomplist(a, height, width, input_edge);
		  //u->showJoinedcomplist(b, height, width, input_edge);
		  u->UpdateJoinedBox(a, b, join_minx, join_miny, join_maxx, join_maxy);
		  u->join(a, b);
		  a = u->find(a);
		  threshold[a] = pedge->w + THRESHOLD(u->size(a), c);
		  //u->showJoinedcomplist(a, height, width, input_edge);
		  //u->showJoinedcomplist(b, height, width, input_edge);
		}
    }

  }
  //u->showcomplist(height, width, input_edge);
  //clockEnd = clock();    
  //printf("loop time : %d\n", clockEnd - clockBegin);
  // free up
  delete threshold;
  return u;
}

#endif
