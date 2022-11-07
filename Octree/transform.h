#pragma once
#ifndef __TRANSFORM_H__
#define __TRANSFORM_H__

#include "vectorops.h"
#include "pointcloud.h"
#include "geometry.h"
#include "BSP.h"
#include <stdio.h>
#include <stdlib.h>

_inline static void transformation(Tree tree, matrix transformation, matrix inv_transformation, matrix norm_transformation) {
	matrix_prod(tree->transformations, tree->transformations, transformation);
	matrix_prod(tree->inv_transformations, inv_transformation, tree->inv_transformations);
	matrix_prod(tree->norm_transformations, tree->norm_transformations, norm_transformation);
}

_inline void treetrans(Tree tree, double x, double y, double z) {
	matrix translation, inv_translation;
	matrixtrans(translation, x, y, z);
	matrixtrans(inv_translation, -x, -y, -z);
	transformation(tree, translation, inv_translation, translation);
}

_inline static void rescale_node(Tree tree, double x, double y, double z) {
	if (NULL != tree->geometry) {
		tree->geometry->x_scale *= x;
		tree->geometry->y_scale *= y;
		tree->geometry->z_scale *= z;
	}
	else {
		rescale_node(tree->left, x, y, z);
		rescale_node(tree->right, x, y, z);
	}
}

_inline void treescale(Tree tree, double x, double y, double z) {
	matrix homothetie, inv_homothetie, norm_homothetie;
	matrixscale(homothetie, x, y, z);
	matrixscale(inv_homothetie, 1. / x, 1. / y, 1. / z);
	matrixscale(norm_homothetie, y / z, z / x, x / y);
	transformation(tree, homothetie, inv_homothetie, norm_homothetie);
	rescale_node(tree, x, y, z);
}

_inline void treerot(Tree tree, double x, double y, double z) {
	matrix rotation, inv_rotation;
	matrixrotx(rotation, x);
	matrixrotx(inv_rotation, -x);
	transformation(tree, rotation, inv_rotation, rotation);
	matrixroty(rotation, y);
	matrixroty(inv_rotation, -y);
	transformation(tree, rotation, inv_rotation, rotation);
	matrixrotz(rotation, z);
	matrixrotz(inv_rotation, -z);
	transformation(tree, rotation, inv_rotation, rotation);
}
#endif