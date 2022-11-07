#pragma once
#ifndef __INPUT_H__
#define __INPUT_H__

#include "BSP.h"
#include "boolean.h"
#include "transform.h"
#include "vectorops.h"
#include "geometry.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int _count;

_inline static Tree solid(char* strgeometry, char* args) {
	static double cx, cy, cz, cw;
	static double tx, ty, tz;
	static double rx, ry, rz;
	static double sx, sy, sz;
	if (strncmp(strgeometry, "sphere", 30) == 0) {
		int read = sscanf(args, "(%lf,%lf,%lf) (%lf,%lf,%lf) (%lf,%lf,%lf)", &tx, &ty, &tz, &rx, &ry, &rz, &sx, &sy, &sz);

		Tree tree = bspsave(geometrysave(Sphere, containssphere, pointcloudsphere, NULL), 0, NULL, NULL);
		treetrans(tree, tx, ty, tz);
		treerot(tree, rx, ry, rz);
		treescale(tree, sx, sy, sz);
		return tree;
	}
	if (strncmp(strgeometry, "cube", 30) == 0) {
		int read = sscanf(args, "(%lf,%lf,%lf) (%lf,%lf,%lf) (%lf,%lf,%lf)", &tx, &ty, &tz, &rx, &ry, &rz, &sx, &sy, &sz);

		Tree tree = bspsave(geometrysave(Cube, containscube, pointcloudcube, NULL), 0, NULL, NULL);
		treetrans(tree, tx, ty, tz);
		treerot(tree, rx, ry, rz);
		treescale(tree, sx, sy, sz);
		return tree;
	}
	if (strncmp(strgeometry, "cylinder", 30) == 0) {
		int read = sscanf(args, "(%lf,%lf,%lf) (%lf,%lf,%lf) (%lf,%lf,%lf)", &tx, &ty, &tz, &rx, &ry, &rz, &sx, &sy, &sz);

		Tree tree = bspsave(geometrysave(Cylinder, containscylinder, pointcloudcylinder, NULL), 0, NULL, NULL);
		treetrans(tree, tx, ty, tz);
		treerot(tree, rx, ry, rz);
		treescale(tree, sx, sy, sz);
		return tree;
	}
	if (strncmp(strgeometry, "cone", 30) == 0) {
		int read = sscanf(args, "(%lf,%lf,%lf) (%lf,%lf,%lf) (%lf,%lf,%lf)", &tx, &ty, &tz, &rx, &ry, &rz, &sx, &sy, &sz);

		Tree tree = bspsave(geometrysave(Cone, containscone, pointcloudcone, NULL), 0, NULL, NULL);
		treetrans(tree, tx, ty, tz);
		treerot(tree, rx, ry, rz);
		treescale(tree, sx, sy, sz);
		return tree;
	}
	printf("token inválido \n");
}

_inline Tree branch(FILE* file) {
	Tree tree = NULL, left = NULL, right = NULL;
	char* line = NULL;
	char node[30];
	line = (char*)malloc(312 * sizeof(char));
	fgets(line, 312, file);
	int l = ++_count;
	sscanf(line, "%" "30" "s", node);
	line += 1 + strlen(node);
	if (strncmp(node, "+", 30) == 0) {
		left = branch(file);
		right = branch(file);
		tree = bspsave(NULL, Union, left, right);
	}
	else if (strncmp(node, "*", 30) == 0) {
		left = branch(file);
		right = branch(file);
		tree = bspsave(NULL,Intersection, left, right);
	}
	else if (strncmp(node, "-", 30) == 0) {
		left = branch(file);
		right = branch(file);
		tree = bspsave(NULL, Difference, left, right);
	}
	else {
		return solid(node, line);
	}
	return tree;
}

_inline Tree readtree(FILE* file) {
	_count = 0;
	return branch(file);
}
#endif