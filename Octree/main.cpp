#include "BSP.h"
#include "input.h"
#include "pointcloud.h"
#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/freeglut.h>
#include <time.h>
#include <iostream>
using namespace std;

int density = 0;
FILE* f;
Tree scene;
PointCloud* pointsscene = NULL;

void draw(const PointCloud* point_cloud) {
	if (point_cloud == NULL) { return; }
	point3* v = point_cloud->vertices;
	vec3* n = point_cloud->norm;
	point3* max = point_cloud->vertices + point_cloud->size;
	glPointSize(2);
	glBegin(GL_POINTS);
	glEnable(GL_COLOR_MATERIAL);
	GLfloat color[] = { 1, 0.2, 0.2, 1 };
	GLfloat specular[] = { 0.6, 0.6, 0.6, 1 };
	GLfloat shininess = 30;
	while (v < max) {
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color);
		glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
		glMaterialf(GL_FRONT, GL_SHININESS, shininess);
		glNormal3dv(*n);
		glVertex3dv(*v);
		n++;
		v++;
	}
	glDisable(GL_COLOR_MATERIAL);
	glEnd();
}

void keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case 'A':
	case 'a':
		density = 100000;
		srand(time(NULL));
		f = fopen("../exemplos/exemp1.txt", "r");
		scene = readtree(f);
		fclose(f);
		pointsscene = bsp2pointcloud(scene, density);
		break;
	case 'B':
	case 'b':
		density = 100000;
		srand(time(NULL));
		f = fopen("../exemplos/exemp2.txt", "r");
		scene = readtree(f);
		fclose(f);
		pointsscene = bsp2pointcloud(scene, density);
		break;
	case 'C':
	case 'c':
		density = 100000;
		srand(time(NULL));
		f = fopen("../exemplos/exemp3.txt", "r");
		scene = readtree(f);
		fclose(f);
		pointsscene = bsp2pointcloud(scene, density);
		break;
	case 'D':
	case 'd':
		density = 100000;
		srand(time(NULL));
		f = fopen("../exemplos/exemp4.txt", "r");
		scene = readtree(f);
		fclose(f);
		pointsscene = bsp2pointcloud(scene, density);
		break;
	case 'T':
	case 't':
		density = 100000;
		srand(time(NULL));
		f = fopen("../exemplos/tema.txt", "r");
		scene = readtree(f);
		fclose(f);
		pointsscene = bsp2pointcloud(scene, density);
		break;
		break;
	}
	glutPostRedisplay();
}

void display() {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	double w = glutGet(GLUT_WINDOW_WIDTH);
	double h = glutGet(GLUT_WINDOW_HEIGHT);
	gluPerspective(60.0, w / h, 0.1, 1000.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(
		-2, 5, 5,
		0, 0, 0,
		0, 0, 1
	);
	glClearColor(0.22, 0.22, 0.22, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	draw(pointsscene);
	glutSwapBuffers();
}

void init() {
	GLfloat black[] = { 0.2, 0.2, 0.2, 1 };
	GLfloat white[] = { 0.75, 0.75, 0.75, 1 };
	GLfloat direction[] = { 0, 0, 1, 0 };
	glLightfv(GL_LIGHT0, GL_AMBIENT, black);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
	glLightfv(GL_LIGHT0, GL_SPECULAR, white);
	glLightfv(GL_LIGHT0, GL_POSITION, direction);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_NORMALIZE);
}

void reshape(int w, int h) {
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (GLfloat)w / (GLfloat)h, 4.0f, 500.0);
	glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char* argv[]) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(768, 512);
	glutCreateWindow("CSG");
	init();
	glutDisplayFunc(display);
	glutKeyboardFunc(keyboard);
	glutReshapeFunc(reshape);
	glutMainLoop();
	pointcloudfree(&pointsscene);
	bspfree(&scene);
	system("pause");
	return 0;
}