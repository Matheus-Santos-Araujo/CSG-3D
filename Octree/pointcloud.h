#pragma once
#ifndef __POINTCLOUD__H__
#define __POINTCLOUD__H__

#include "vectorops.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

typedef struct {
	point3* vertices;
	vec3* norm; 
	int size;
} PointCloud;

_inline PointCloud* pointcloudsave(point3* vertices, vec3* norm, int size) {
	PointCloud* point_cloud = NULL;
	point_cloud = (PointCloud*)malloc(sizeof(PointCloud));
	point_cloud->vertices = vertices;
	point_cloud->norm = norm;
	point_cloud->size = size;
	return point_cloud;
}

_inline void pointcloudfree(PointCloud** point_cloud) {
	free((*point_cloud)->vertices);
	free((*point_cloud)->norm);
	free((*point_cloud));
	(*point_cloud) = NULL;
}
#endif