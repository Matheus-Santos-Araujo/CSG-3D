#pragma once
#ifndef __TRANSFORMS_H__
#define __TRANSFORMS_H__

#include "vectorops.h"
#include "pointcloud.h"
#include "geometry.h"
#include "BSP.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

// Geometric
_inline static void tree_transform(Tree tree, mat4 transformation, mat4 inv_transformation, mat4 norm_transformation) {
	mat4_product_mat4(tree->transformations, tree->transformations, transformation);
	mat4_product_mat4(tree->inv_transformations, inv_transformation, tree->inv_transformations);
	mat4_product_mat4(tree->norm_transformations, tree->norm_transformations, norm_transformation);
}

_inline void tree_translation(Tree tree, double x, double y, double z) {
	mat4 translation, inv_translation;
	mat4_translation(translation, x, y, z);
	mat4_translation(inv_translation, -x, -y, -z);
	tree_transform(tree, translation, inv_translation, translation);
}

_inline static void rescale_node(Tree tree, double x, double y, double z) {
	if (NULL != tree->shape) {
		tree->shape->x_scale *= x;
		tree->shape->y_scale *= y;
		tree->shape->z_scale *= z;
	}
	else {
		rescale_node(tree->left, x, y, z);
		rescale_node(tree->right, x, y, z);
	}
}

_inline void tree_scale(Tree tree, double x, double y, double z) {
	mat4 homothetie, inv_homothetie, norm_homothetie;
	mat4_homothety(homothetie, x, y, z);
	mat4_homothety(inv_homothetie, 1. / x, 1. / y, 1. / z);
	mat4_homothety(norm_homothetie, y / z, z / x, x / y);
	tree_transform(tree, homothetie, inv_homothetie, norm_homothetie);
	rescale_node(tree, x, y, z);
}

_inline void tree_rotation(Tree tree, double x, double y, double z) {
	mat4 rotation, inv_rotation;
	mat4_rotation_x(rotation, x);
	mat4_rotation_x(inv_rotation, -x);
	tree_transform(tree, rotation, inv_rotation, rotation);
	mat4_rotation_y(rotation, y);
	mat4_rotation_y(inv_rotation, -y);
	tree_transform(tree, rotation, inv_rotation, rotation);
	mat4_rotation_z(rotation, z);
	mat4_rotation_z(inv_rotation, -z);
	tree_transform(tree, rotation, inv_rotation, rotation);
}

#endif