#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

#include "vectorops.h"
#include "pointcloud.h"
#define PI 3.1415926535897932384626433832795
#define SQUARE(a) ((a)*(a))

typedef enum {
	Sphere, 
	Cube, 
	Cylinder, 
	Cone, 
	NumberGeometryType 
} GeometryType;

typedef struct {
	GeometryType type; 
	double x_scale; 
	double y_scale; 
	double z_scale;
	double* args; 
	int (*contains_function)(double*, point3*); 
	PointCloud* (*point_cloud_converter)(int, matrix, matrix, double, double, double, double*); 
} Geometry;

// Utilities
double rand(double min, double max) { return (max - min) * ((double)rand() / (double)RAND_MAX) + min; }

static void minmax(double* a, double* b, double* c) {
	double tmp;
	if (*a < *b) {
		tmp = *a;
		*a = *b;
		*b = tmp;
	}
	if (*a < *c) {
		tmp = *a;
		*a = *c;
		*c = tmp;
	}
	if (*b < *c) {
		tmp = *b;
		*b = *c;
		*c = tmp;
	}
}

static Geometry* geometrysave(GeometryType type, int(*contains_function)(double*, point3*), PointCloud* (*point_cloud_converter)(int, matrix, matrix, double, double, double, double*), double* args) {
	Geometry* geometry = NULL;
	geometry = (Geometry*)malloc(sizeof(Geometry));
	geometry->type = type;
	geometry->contains_function = contains_function;
	geometry->point_cloud_converter = point_cloud_converter;
	geometry->args = args;
	geometry->x_scale = geometry->y_scale = geometry->z_scale = 1;
	return geometry;
}

// Sphere
static int containssphere(double* args, point3* point) {
	return SQUARE(((*point)[0])) + SQUARE(((*point)[1])) + SQUARE(((*point)[2])) <= 1.;
}

static PointCloud* pointcloudsphere(int d, matrix transformations, matrix norm_transformations, double x_scale, double y_scale, double z_scale, double* args) {
	double a = x_scale, b = y_scale, c = z_scale;
	minmax(&a, &b, &c);
	double sqr_c = SQUARE(c);
	int size;
	if (a == c) {
		size = d * 4 * PI * sqr_c;
	}
	else {
		double sqrt_sasc = sqrt(SQUARE(a) - sqr_c);
		size = d * 2 * PI * (sqr_c + ((b * sqr_c) / (sqrt_sasc)) + b * sqrt_sasc);
	}
	point3* vertices = NULL;
	vec3* norm = NULL;
	vertices = (point3*)malloc(size * sizeof(point3));
	norm = (vec3*)malloc(size * sizeof(vec3));
	int i;
	point3* v = vertices;
	vec3* n = norm;
	double half_pi = PI / 2;
	for (i = 0; i < size; i++) {
		double alpha = rand(0, 2 * PI);
		double phi = rand(0, PI) + rand(0, PI) / 2;
		double sin_phi = sin(phi);
		coord3_set((*n), cos(alpha) * sin_phi, sin(alpha) * sin_phi, cos(phi));
		coord3_set((*v), cos(alpha) * sin_phi, sin(alpha) * sin_phi, cos(phi));
		matrix_prodp3((*v), transformations, (*v));
		matrix_prodv3((*n), norm_transformations, (*n));
		n++;
		v++;
	}
	return pointcloudsave(vertices, norm, size);
}

// Cube
static int containscube(double* args, point3* point) {
	double x = ((*point)[0]);
	double y = ((*point)[1]);
	double z = ((*point)[2]);
	if (x < 0.)
		x = -x;
	if (y < 0.)
		y = -y;
	if (z < 0.)
		z = -z;
	return MAX3(x, y, z) <= 1.;
}

static PointCloud* pointcloudcube(int d, matrix transformations, matrix norm_transformations, double x_scale, double y_scale, double z_scale, double* args) {
	point3* vertices = NULL;
	vec3* norm = NULL;
	int xface = 4 * d * y_scale * z_scale;
	int yface = 4 * d * x_scale * z_scale;
	int zface = 4 * d * x_scale * y_scale;
	int size = 2 * (xface + yface + zface);
	vertices = (point3*)malloc(size * sizeof(point3));
	norm = (vec3*)malloc(size * sizeof(vec3));
	double x, y, z;
	int i;
	point3* v = vertices;
	vec3* n = norm;
	for (i = 0; i < zface; i++) {
		for (z = -1; z <= 1; z += 2) {
			x = rand(-1, 1);
			y = rand(-1, 1);
			coord3_set((*n), 0, 0, z);
			coord3_set((*v), x, y, z);
			matrix_prodp3((*v), transformations, (*v));
			matrix_prodv3((*n), norm_transformations, (*n));
			n++;
			v++;
		}
	}
	for (i = 0; i < yface; i++) {
		for (y = -1; y <= 1; y += 2) {
			x = rand(-1, 1);
			z = rand(-1, 1);
			coord3_set((*n), 0, y, 0);
			coord3_set((*v), x, y, z);
			matrix_prodp3((*v), transformations, (*v));
			matrix_prodv3((*n), norm_transformations, (*n));
			n++;
			v++;
		}
	}
	for (i = 0; i < xface; i++) {
		for (x = -1; x <= 1; x += 2) {
			y = rand(-1, 1);
			z = rand(-1, 1);
			coord3_set((*n), x, 0, 0);
			coord3_set((*v), x, y, z);
			matrix_prodp3((*v), transformations, (*v));
			matrix_prodv3((*n), norm_transformations, (*n));
			n++;
			v++;
		}
	}
	return pointcloudsave(vertices, norm, size);
}

// Cylinder
static int containscylinder(double* args, point3* point) {
	double z = ((*point)[2]);
	if (z < 0.)
		z = -z;
	return (z <= 1.) && (SQUARE(((*point)[0])) + SQUARE(((*point)[1])) <= 1.);
}

static PointCloud* pointcloudcylinder(int d, matrix transformations, matrix norm_transformations, double x_scale, double y_scale, double z_scale, double* args) {
	int face = d * PI * x_scale * y_scale;
	int side = d * 2 * z_scale * PI * sqrt(2 * (SQUARE(x_scale) + SQUARE(y_scale)));
	int size = side + 2 * face;
	point3* vertices = NULL;
	vec3* norm = NULL;
	vertices = (point3*)malloc(size * sizeof(point3));
	norm = (vec3*)malloc(size * sizeof(vec3));
	double z;
	int i;
	point3* v = vertices;
	vec3* n = norm;
	for (i = 0; i < side; i++) {
		z = rand(-1, 1);
		double alpha = rand(0, 2 * PI);
		coord3_set((*n), cos(alpha), sin(alpha), 0);
		coord3_set((*v), cos(alpha), sin(alpha), z);
		matrix_prodp3((*v), transformations, (*v));
		matrix_prodv3((*n), norm_transformations, (*n));
		n++;
		v++;
	}
	for (z = -1; z <= 1; z += 2) {
		i = 0;
		while (i < face) {
			double x = rand(-1, 1);
			double y = rand(-1, 1);
			if (x * x + y * y > 1)
				continue;
			coord3_set((*n), 0, 0, z);
			coord3_set((*v), x, y, z);
			matrix_prodp3((*v), transformations, (*v));
			matrix_prodv3((*n), norm_transformations, (*n));
			n++;
			v++;
			i++;
		}
	}
	return pointcloudsave(vertices, norm, size);
}

// Cone
static int containscone(double* args, point3* point) {
	double z = ((*point)[2]);
	if (z < 0.)
		z = -z;
	double rz = 1 - ((*point)[2]);
	return (z <= 1.) && (SQUARE(((*point)[0])) + SQUARE(((*point)[1])) <= SQUARE(rz) / 4.);
}

static PointCloud* pointcloudcone(int d, matrix transformations, matrix norm_transformations, double x_scale, double y_scale, double z_scale, double* args) {
	double r = x_scale * y_scale;
	double b = d * PI * r;
	int face = b;
	int side = b * sqrt(1 + (4. * SQUARE(z_scale)) / r);
	int size = side + face;
	point3* vertices = NULL;
	vec3* norm = NULL;
	vertices = (point3*)malloc(size * sizeof(point3));
	norm = (vec3*)malloc(size * sizeof(vec3));
	int i;
	point3* v = vertices;
	vec3* n = norm;
	for (i = 0; i < side; i++) {
		double z = 2 * (1 - sqrt(rand(0, 1))) - 1;
		double alpha = rand(0, 2 * PI);
		double rz = (1 - z) / 2;
		double cos_alpha = cos(alpha);
		double sin_alpha = sin(alpha);
		coord3_set((*n), cos_alpha, sin_alpha, 1);
		coord3_set((*v), rz * cos_alpha, rz * sin_alpha, z);
		matrix_prodp3((*v), transformations, (*v));
		matrix_prodv3((*n), norm_transformations, (*n));
		n++;
		v++;
	}
	i = 0;
	while (i < face) {
		double x = rand(-1, 1);
		double y = rand(-1, 1);
		if (x * x + y * y > 1)
			continue;
		coord3_set((*n), 0, 0, -1);
		coord3_set((*v), x, y, -1);
		matrix_prodp3((*v), transformations, (*v));
		matrix_prodv3((*n), norm_transformations, (*n));
		n++;
		v++;
		i++;
	}
	return pointcloudsave(vertices, norm, size);
}
#endif