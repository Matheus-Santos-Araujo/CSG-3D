#pragma once
#ifndef __BSP_H__
#define __BSP_H__

#include "vectorops.h"
#include "pointcloud.h"
#include "geometry.h"
#include <stdio.h>
#include <stdlib.h>

typedef enum {
	Union, 
	Intersection, 
	Difference,
	Identity,
	NumberOperator
} Operator;

typedef struct Node {
	Operator op;
	Geometry* geometry; 
	matrix transformations; 
	matrix inv_transformations;
	matrix norm_transformations; 
	struct Node* left; 
	struct Node* right; 
} *Tree;

_inline static Tree bspsave(Geometry* geometry, int op, Tree left, Tree right) {
	Tree t = NULL;
	t = (Tree) malloc(sizeof(struct Node));
	t->geometry = geometry;
	t->left = left;
	t->right = right;
	t->op = (Operator) op;
	matrix_set_identity(t->transformations);
	matrix_set_identity(t->inv_transformations);
	matrix_set_identity(t->norm_transformations);
	return t;
}

_inline void bspfree(Tree* tree) {
	if ((*tree)->geometry != NULL) {
		free((&((*tree)->geometry)));
		((*tree)->geometry) = NULL;
	}
	else {
		bspfree(&((*tree)->left));
		bspfree(&((*tree)->right));
	}
	free((*tree));
	(*tree) = NULL;
}
#endif