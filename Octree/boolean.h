#pragma once
#ifndef __BOOLEAN_H__
#define __BOOLEAN_H__

#include "vectorops.h"
#include "pointcloud.h"
#include "geometry.h"
#include "BSP.h"
#include <stdio.h>
#include <stdlib.h>

// Utilities
PointCloud* bsp2pointcloud(Tree tree, int density);

_inline static int bspcontains(Tree tree, point3* point) {
	point3 p;
	matrix_prodp3(p, tree->inv_transformations, *point);
	if (tree->geometry != NULL) {
		return tree->geometry->contains_function(tree->geometry->args, (point3*)&p);
	}
	switch (tree->op) {
	case Union:
		return bspcontains(tree->left, &p) || bspcontains(tree->right, &p);
	case Intersection:
		return bspcontains(tree->left, &p) && bspcontains(tree->right, &p);
	case Difference:
		return bspcontains(tree->left, &p) && !bspcontains(tree->right, &p);
	}
}

// Union
_inline static PointCloud* opunion(matrix transformations, matrix norm_transformations, Tree left, Tree right, int density) {
	point3* vertices = NULL;
	vec3* norm = NULL;
	PointCloud* a = bsp2pointcloud(left, density);
	PointCloud* b = bsp2pointcloud(right, density);
	int size = a->size + b->size;
	vertices = (point3*)malloc(size * sizeof(point3));
	norm = (vec3*)malloc(size * sizeof(vec3));
	size = 0;
	int i;
	for (i = 0; i < a->size; i++) {
		if (!bspcontains(right, a->vertices + i)) {
			matrix_prodp3(vertices[size], transformations, a->vertices[i]);
			matrix_prodv3(norm[size], norm_transformations, a->norm[i]);
			size++;
		}
	}
	for (i = 0; i < b->size; i++) {
		if (!bspcontains(left, b->vertices + i)) {
			matrix_prodp3(vertices[size], transformations, b->vertices[i]);
			matrix_prodv3(norm[size], norm_transformations, b->norm[i]);
			size++;
		}
	}
	pointcloudfree(&a);
	pointcloudfree(&b);
	return pointcloudsave(vertices, norm, size);
}

// Inter
_inline static PointCloud* opintersection(matrix transformations, matrix norm_transformations, Tree left, Tree right, int density) {
	point3* vertices = NULL;
	vec3* norm = NULL;
	PointCloud* a = bsp2pointcloud(left, density);
	PointCloud* b = bsp2pointcloud(right, density);
	int size = a->size + b->size;
	vertices = (point3*)malloc(size * sizeof(point3));
	norm = (vec3*)malloc(size * sizeof(vec3));
	size = 0;
	int i;
	for (i = 0; i < a->size; i++) {
		if (bspcontains(right, a->vertices + i)) {
			matrix_prodp3(vertices[size], transformations, a->vertices[i]);
			matrix_prodv3(norm[size], norm_transformations, a->norm[i]);
			size++;
		}
	}
	for (i = 0; i < b->size; i++) {
		if (bspcontains(left, b->vertices + i)) {
			matrix_prodp3(vertices[size], transformations, b->vertices[i]);
			matrix_prodv3(norm[size], norm_transformations, b->norm[i]);
			size++;
		}
	}
	pointcloudfree(&a);
	pointcloudfree(&b);
	return pointcloudsave(vertices, norm, size);
}

// Diff
_inline static PointCloud* opdifference(matrix transformations, matrix norm_transformations, Tree left, Tree right, int density) {
	point3* vertices = NULL;
	vec3* norm = NULL;
	PointCloud* a = bsp2pointcloud(left, density);
	PointCloud* b = bsp2pointcloud(right, density);
	int size = a->size + b->size;
	vertices = (point3*)malloc(size * sizeof(point3));
	norm = (vec3*)malloc(size * sizeof(vec3));
	size = 0;
	int i;
	for (i = 0; i < a->size; i++) {
		if (!bspcontains(right, a->vertices + i)) {
			matrix_prodp3(vertices[size], transformations, a->vertices[i]);
			matrix_prodv3(norm[size], norm_transformations, a->norm[i]);
			size++;
		}
	}
	for (i = 0; i < b->size; i++) {
		if (bspcontains(left, b->vertices + i)) {
			matrix_prodp3(vertices[size], transformations, b->vertices[i]);
			coord3_set(
				norm[size],
				-((b->norm[i])[0]),
				-((b->norm[i])[1]),
				-((b->norm[i])[2])
			);
			matrix_prodv3(norm[size], norm_transformations, norm[size]);
			size++;
		}
	}
	pointcloudfree(&a);
	pointcloudfree(&b);
	return pointcloudsave(vertices, norm, size);
}

// Point Cloud
PointCloud* bsp2pointcloud(Tree tree, int density) {
	if (tree->geometry != NULL) {
		return tree->geometry->point_cloud_converter(density, tree->transformations, tree->norm_transformations, tree->geometry->x_scale, tree->geometry->y_scale, tree->geometry->z_scale, tree->geometry->args);
	}
	switch (tree->op) {
	case Union:
		return opunion(tree->transformations, tree->norm_transformations, tree->left, tree->right, density);
	case Intersection:
		return opintersection(tree->transformations, tree->norm_transformations, tree->left, tree->right, density);
	case Difference:
		return opdifference(tree->transformations, tree->norm_transformations, tree->left, tree->right, density);
	default:
		printf("Erro BSP2PointCloud \n");

	}
}
#endif