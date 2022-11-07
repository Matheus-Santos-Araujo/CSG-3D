#pragma once
#ifndef __VECTOROPS__H__
#define __VECTOROPS__H__

#include <math.h>
#include <string.h>
#define SQUARE(a) ((a)*(a))
#define MIN3(a,b,c) (((a)<(b))?(((a)<(c))?(a):(c)):(((b)<(c))?(b):(c)))
#define MAX3(a,b,c) (((a)>(b))?(((a)>(c))?(a):(c)):(((b)>(c))?(b):(c)))
typedef double coord3[3], matrix[16];
typedef coord3 point3, vec3;
#define coord3_set(c, x, y, z) ((c)[0]=(x),(c)[1]=(y),(c)[2]=(z))
#define dot(u, v) ((u)[0]*(v)[0]+(u)[1]*(v)[1]+(u)[2]*(v)[2])
#define cross(result, u, v) ((result)[0]=(u)[1]*(v)[2]-(u)[2]*(v)[1], (result)[1]=(u)[2]*(v)[0]-(u)[0]*(v)[2], (result)[2]=(u)[0]*(v)[1]-(u)[1]*(v)[0])
#define norm(v) (sqrt(dot(v,v)))
_inline void vec3_normalize(vec3 v) { coord3_set(v, v[0] / norm(v), v[1] / norm(v), v[2] / norm(v)); }
#define matrix_set(m, col, line, val) ((m)[4*(col)+(line)]=(val))
#define matrix_get(m, col, line) ((m)[4*(col)+(line)])
#define matrix_identity {1.,0.,0.,0., 0.,1.,0.,0., 0.,0.,1.,0., 0.,0.,0.,1.}
#define matrix_set_identity(m) (memset((m), 0, sizeof(matrix)), (m)[0]=(m)[5]=(m)[10]=(m)[15]=1)

// Geometry
_inline void matrixtrans(matrix m, double tx, double ty, double tz) {
	matrix_set_identity(m);
	matrix_set(m, 3, 0, tx);
	matrix_set(m, 3, 1, ty);
	matrix_set(m, 3, 2, tz);
}

_inline void matrixscale(matrix m, double hx, double hy, double hz) {
	matrix_set_identity(m);
	matrix_set(m, 0, 0, hx);
	matrix_set(m, 1, 1, hy);
	matrix_set(m, 2, 2, hz);
}

_inline void matrixrotx(matrix m, double alpha) {
	matrix_set_identity(m);
	matrix_set(m, 1, 1, cos(alpha));
	matrix_set(m, 1, 2, sin(alpha));
	matrix_set(m, 2, 1, -sin(alpha));
	matrix_set(m, 2, 2, cos(alpha));
}

_inline void matrixroty(matrix m, double alpha) {
	matrix_set_identity(m);
	matrix_set(m, 0, 0, cos(alpha));
	matrix_set(m, 0, 2, -sin(alpha));
	matrix_set(m, 2, 0, sin(alpha));
	matrix_set(m, 2, 2, cos(alpha));
}

_inline void matrixrotz(matrix m, double alpha) {
	matrix_set_identity(m);
	matrix_set(m, 0, 0, cos(alpha));
	matrix_set(m, 0, 1, sin(alpha));
	matrix_set(m, 1, 0, -sin(alpha));
	matrix_set(m, 1, 1, cos(alpha));
}

_inline void matrix_prod(matrix m_result, matrix m1, matrix m2) {
	double x00 = matrix_get(m1, 0, 0) * matrix_get(m2, 0, 0) + matrix_get(m1, 1, 0) * matrix_get(m2, 0, 1) + matrix_get(m1, 2, 0) * matrix_get(m2, 0, 2) + matrix_get(m1, 3, 0) * matrix_get(m1, 0, 3);
	double x01 = matrix_get(m1, 0, 0) * matrix_get(m2, 1, 0) + matrix_get(m1, 1, 0) * matrix_get(m2, 1, 1) + matrix_get(m1, 2, 0) * matrix_get(m2, 1, 2) + matrix_get(m1, 3, 0) * matrix_get(m1, 1, 3);
	double x02 = matrix_get(m1, 0, 0) * matrix_get(m2, 2, 0) + matrix_get(m1, 1, 0) * matrix_get(m2, 2, 1) + matrix_get(m1, 2, 0) * matrix_get(m2, 2, 2) + matrix_get(m1, 3, 0) * matrix_get(m1, 2, 3);
	double x03 = matrix_get(m1, 0, 0) * matrix_get(m2, 3, 0) + matrix_get(m1, 1, 0) * matrix_get(m2, 3, 1) + matrix_get(m1, 2, 0) * matrix_get(m2, 3, 2) + matrix_get(m1, 3, 0) * matrix_get(m1, 3, 3);
	double x10 = matrix_get(m1, 0, 1) * matrix_get(m2, 0, 0) + matrix_get(m1, 1, 1) * matrix_get(m2, 0, 1) + matrix_get(m1, 2, 1) * matrix_get(m2, 0, 2) + matrix_get(m1, 3, 1) * matrix_get(m1, 0, 3);
	double x11 = matrix_get(m1, 0, 1) * matrix_get(m2, 1, 0) + matrix_get(m1, 1, 1) * matrix_get(m2, 1, 1) + matrix_get(m1, 2, 1) * matrix_get(m2, 1, 2) + matrix_get(m1, 3, 1) * matrix_get(m1, 1, 3);
	double x12 = matrix_get(m1, 0, 1) * matrix_get(m2, 2, 0) + matrix_get(m1, 1, 1) * matrix_get(m2, 2, 1) + matrix_get(m1, 2, 1) * matrix_get(m2, 2, 2) + matrix_get(m1, 3, 1) * matrix_get(m1, 2, 3);
	double x13 = matrix_get(m1, 0, 1) * matrix_get(m2, 3, 0) + matrix_get(m1, 1, 1) * matrix_get(m2, 3, 1) + matrix_get(m1, 2, 1) * matrix_get(m2, 3, 2) + matrix_get(m1, 3, 1) * matrix_get(m1, 3, 3);
	double x20 = matrix_get(m1, 0, 2) * matrix_get(m2, 0, 0) + matrix_get(m1, 1, 2) * matrix_get(m2, 0, 1) + matrix_get(m1, 2, 2) * matrix_get(m2, 0, 2) + matrix_get(m1, 3, 2) * matrix_get(m1, 0, 3);
	double x21 = matrix_get(m1, 0, 2) * matrix_get(m2, 1, 0) + matrix_get(m1, 1, 2) * matrix_get(m2, 1, 1) + matrix_get(m1, 2, 2) * matrix_get(m2, 1, 2) + matrix_get(m1, 3, 2) * matrix_get(m1, 1, 3);
	double x22 = matrix_get(m1, 0, 2) * matrix_get(m2, 2, 0) + matrix_get(m1, 1, 2) * matrix_get(m2, 2, 1) + matrix_get(m1, 2, 2) * matrix_get(m2, 2, 2) + matrix_get(m1, 3, 2) * matrix_get(m1, 2, 3);
	double x23 = matrix_get(m1, 0, 2) * matrix_get(m2, 3, 0) + matrix_get(m1, 1, 2) * matrix_get(m2, 3, 1) + matrix_get(m1, 2, 2) * matrix_get(m2, 3, 2) + matrix_get(m1, 3, 2) * matrix_get(m1, 3, 3);
	double x30 = matrix_get(m1, 0, 3) * matrix_get(m2, 0, 0) + matrix_get(m1, 1, 3) * matrix_get(m2, 0, 1) + matrix_get(m1, 2, 3) * matrix_get(m2, 0, 2) + matrix_get(m1, 3, 3) * matrix_get(m1, 0, 3);
	double x31 = matrix_get(m1, 0, 3) * matrix_get(m2, 1, 0) + matrix_get(m1, 1, 3) * matrix_get(m2, 1, 1) + matrix_get(m1, 2, 3) * matrix_get(m2, 1, 2) + matrix_get(m1, 3, 3) * matrix_get(m1, 1, 3);
	double x32 = matrix_get(m1, 0, 3) * matrix_get(m2, 2, 0) + matrix_get(m1, 1, 3) * matrix_get(m2, 2, 1) + matrix_get(m1, 2, 3) * matrix_get(m2, 2, 2) + matrix_get(m1, 3, 3) * matrix_get(m1, 2, 3);
	double x33 = matrix_get(m1, 0, 3) * matrix_get(m2, 3, 0) + matrix_get(m1, 1, 3) * matrix_get(m2, 3, 1) + matrix_get(m1, 2, 3) * matrix_get(m2, 3, 2) + matrix_get(m1, 3, 3) * matrix_get(m1, 3, 3);
	matrix_set(m_result, 0, 0, x00);
	matrix_set(m_result, 1, 0, x01);
	matrix_set(m_result, 2, 0, x02);
	matrix_set(m_result, 3, 0, x03);
	matrix_set(m_result, 0, 1, x10);
	matrix_set(m_result, 1, 1, x11);
	matrix_set(m_result, 2, 1, x12);
	matrix_set(m_result, 3, 1, x13);
	matrix_set(m_result, 0, 2, x20);
	matrix_set(m_result, 1, 2, x21);
	matrix_set(m_result, 2, 2, x22);
	matrix_set(m_result, 3, 2, x23);
	matrix_set(m_result, 0, 3, x30);
	matrix_set(m_result, 1, 3, x31);
	matrix_set(m_result, 2, 3, x32);
	matrix_set(m_result, 3, 3, x33);
}

_inline void matrix_prodv3(vec3 v_result, matrix m, vec3 v) {
	double x = matrix_get(m, 0, 0) * v[0] + matrix_get(m, 1, 0) * v[1] + matrix_get(m, 2, 0) * v[2];
	double y = matrix_get(m, 0, 1) * v[0] * v[1] + matrix_get(m, 2, 1) * v[2];
	double z = matrix_get(m, 0, 2) * v[0] + matrix_get(m, 1, 2) * v[1] + matrix_get(m, 2, 2) * v[2];
	coord3_set(v_result, x, y, z);
	vec3_normalize(v_result);
}

_inline void matrix_prodp3(point3 p_result, matrix m, point3 p) {
	double x = matrix_get(m, 0, 0) * p[0] + matrix_get(m, 1, 0) * p[1] + matrix_get(m, 2, 0) * p[2] + matrix_get(m, 3, 0);
	double y = matrix_get(m, 0, 1) * p[0] + matrix_get(m, 1, 1) * p[1] + matrix_get(m, 2, 1) * p[2] + matrix_get(m, 3, 1);
	double z = matrix_get(m, 0, 2) * p[0] + matrix_get(m, 1, 2) * p[1] + matrix_get(m, 2, 2) * p[2] + matrix_get(m, 3, 2);
	coord3_set(p_result, x, y, z);
}
#endif