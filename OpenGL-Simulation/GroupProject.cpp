// Phantom.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <windows.h>
#include <iostream>
#include <string.h>
#include <GL/glui.h>
#include <time.h>
#include <time.h>
#include <conio.h>
#include <assert.h>
#include "Inverse_Kinematics.h"
#include <stdio.h>

using namespace std;
#ifdef __APPLE_CC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#include <GL/GLU.h>
#include <math.h>
#include <GL/glui.h>
#endif
#define RADPERDEG 0.017453
GLUquadricObj* myReusableQuadric = 0;
// Joint variables
float baseRot = 0.0;
float theta_1 = 60.0;
float theta_2 = 30.0;
float theta_3 = 60.0;
float theta_4 = 30.0;
float theta_5 = 60.0;
float theta_6 = 30.0;
float baseframe[3] = { 0.0,0.0,0.0 }, ini_pos[3];
float temp,temp1,temp2;
int wh = 0,st=0,dest=0;
float frame1[3] = { 0.0,4.0,0.0 };
float frame3[3] = { 0.0,0.0,4.0 };
float frame4[3] = { 0.0,0.0,3.5 };
float frame6[3] = { 0.0,0.0,2.0 };
float framecam[3] = { 0.0,0.0,2.0 };
/*
struct joint {
	float t1, t2, t3, t4, t5, t6;
};*/

int  main_window;
/*struct transform {
	float r11, r22, r33, r12, r13, r21, r23, r31, r32;
	float px, py, pz;
};*/
transform latest_matrix, old_matrix, final_matrix;
joint ini,fin,tem,tem1;

float tf = 150, tf1=100; float ti = 0,curr_t=0,di_t=0;
double starttime;

//Goal
float goal[3] = {0,0,0},goal1[3];


// Return a true a false value if true robot is going to tip if false it is not
int torque(joint angles)
{
	float t2 = angles.t2;
	float t3 = angles.t3;
	/*
	//density = 0.034 lbs / in ^ 3 * (25.4 / 1000) ^ 3 * 2.2 lbs / kg = 2.94488189
	float density = 2.94488189;

	//area density = density * width * thickness ....width = 0.04m and thickness = 0.00635m;
	float d = density*0.04*0.00635;
	float g = 9.8; //gravity = -9.8m/s^2



	float L2 = 0.25; //meter
	float m2 = L2 *d;
	float t2 = angles.t2;
	float T2 = L2*m2*g*0.5*cos(t2);

	//cout << T2 << "\n";

	float L3 = 0.2; //meter
	float m3 = L3*d + 5;
	float t3 = angles.t3;
	float T3 = m3*g*(L2*cos(t2) + 0.5*L3*cos(t3));

	//cout << T3 << "\n";

	float L4 = 0.2; //meter
	float m4 = 0.5 + 5; //  camera weight
	float T4 = (L2*cos(t2) + L3*cos(t3))*(m4*g);

	//cout << T4 << "\n";

	float T = T2 + T3 + T4;
	//cout << T << "\n";
	*/
	// Set a tip variable = 0
	int tip = 0;

	// Tipping torque
	//float Torque_tip = 35;

	//if (T >= Torque_tip) {

		// If the torque causes the robot to tip return 1
		//tip = 1;
	//}
	// the angles were choosen from the program ran on calculating the angles which causes the robot to tip
	if (t2<29) {
		if (t3 < 59) {
			tip = 1;
		}
	}

	return tip;
}

float trajectory_mapping(float t, float tf, float ti, float theta_i, float theta_f) {
	float a0 = theta_i;
	float a2 = (3 * (theta_f - theta_i)) / pow(tf, 2);
	float a3 = -(2 * (theta_f - theta_i)) / pow(tf, 3);
	float theta;

	theta = a0 + a2*pow(t, 2) + a3*pow(t, 3);
	return theta;
}

void Arrow(GLdouble x1, GLdouble y1, GLdouble z1, GLdouble x2, GLdouble y2, GLdouble z2, GLdouble D)
{
	double x = x2 - x1;
	double y = y2 - y1;
	double z = z2 - z1;
	double L = sqrt(x*x + y*y + z*z);

	GLUquadricObj *quadObj;

	glPushMatrix();

	glTranslated(x1, y1, z1);

	if ((x != 0.) || (y != 0.)) {
		glRotated(atan2(y, x) / RADPERDEG, 0., 0., 1.);
		glRotated(atan2(sqrt(x*x + y*y), z) / RADPERDEG, 0., 1., 0.);
	}
	else if (z<0) {
		glRotated(180, 1., 0., 0.);
	}

	glTranslatef(0, 0, L - 4 * D);

	quadObj = gluNewQuadric();
	gluQuadricDrawStyle(quadObj, GLU_LINE);
	gluQuadricNormals(quadObj, GLU_SMOOTH);
	gluCylinder(quadObj, 2 * D, 0.0, 4 * D, 32, 1);
	gluDeleteQuadric(quadObj);

	quadObj = gluNewQuadric();
	gluQuadricDrawStyle(quadObj, GLU_LINE);
	gluQuadricNormals(quadObj, GLU_SMOOTH);
	gluDisk(quadObj, 0.0, 2 * D, 32, 1);
	gluDeleteQuadric(quadObj);

	glTranslatef(0, 0, -L + 4 * D);
	
	quadObj = gluNewQuadric();
	gluQuadricDrawStyle(quadObj, GLU_LINE);
	gluQuadricNormals(quadObj, GLU_SMOOTH);
	gluCylinder(quadObj, D, D, L - 4 * D, 32, 1);
	gluDeleteQuadric(quadObj);

	quadObj = gluNewQuadric();
	gluQuadricDrawStyle(quadObj, GLU_LINE);
	gluQuadricNormals(quadObj, GLU_SMOOTH);
	gluDisk(quadObj, 0.0, D, 32, 1);
	gluDeleteQuadric(quadObj);

	glPopMatrix();

}

//Cylinder/*************************////
//  A Reusable gluQuadric object:
//GLUquadricObj* myReusableQuadric = 0;
void drawSlantCylinder(double height, double radiusBase, double radiusTop, int slices, int stacks)
{
	if (!myReusableQuadric) {
		myReusableQuadric = gluNewQuadric();
		// Should (but don't) check if pointer is still null --- to catch memory allocation errors.
		gluQuadricNormals(myReusableQuadric, GL_TRUE);
	}
	// Draw the cylinder.
	gluCylinder(myReusableQuadric, radiusBase, radiusTop, height, slices, stacks);
}
void drawCylinder(double height, double radius, int slices, int stacks) {
	drawSlantCylinder(height, radius, radius, slices, stacks);
}
void drawSlantCylinderWithCaps(double height, double radiusBase, double radiusTop, int slices, int stacks)
{
	// First draw the cylinder
	drawSlantCylinder(height, radiusBase, radiusTop, slices, stacks);

	// Draw the top disk cap
	glPushMatrix();
	glTranslated(0.0, 0.0, height);
	gluDisk(myReusableQuadric, 0.0, radiusTop, slices, stacks);
	glPopMatrix();

	// Draw the bottom disk cap
	glPushMatrix();
	glRotated(180.0, 1.0, 0.0, 0.0);
	gluDisk(myReusableQuadric, 0.0, radiusBase, slices, stacks);
	glPopMatrix();

}
void drawCylinderWithCaps(double height, double radius, int slices, int stacks) {
	drawSlantCylinderWithCaps(height, radius, radius, slices, stacks);
}

void drawFlexibleCylinderWithCaps(double height, double radiusBase, double radiusTop, int slices, int stacks) {
	drawSlantCylinderWithCaps(height, radiusBase, radiusTop, slices, stacks);
}



float xy_aspect;
int   last_x, last_y;
float rotationX = 0.0, rotationY = 0.0;

/** These are the live variables passed into GLUI ***/
int   wireframe = 0;
int   obj_type = 1;
int   segments = 8;
int   segments2 = 8;
std::string text = "Hello World!";
int   light0_enabled = 1;
int   light1_enabled = 0;
float light0_intensity = 1.0;
float light1_intensity = 1.0;
int   counter = 0;
float scale = 1.0;

/** Pointers to the windows and some of the controls we'll create **/
GLUI *cmd_line_glui = 0, *glui;
GLUI_Checkbox    *checkbox;
GLUI_Spinner     *spinner1, *light0_spinner, *light1_spinner, *spinner2, *spinner3, *spinner4, *spinner5, *spinner6, *spinner7, *spinner8;
GLUI_RadioGroup  *radio;
GLUI_EditText    *edittext1, *edittext2, *edittext3, *edittext4, *edittext5, *edittext6, *edittext7, *edittext8, *edittext9, *edittext10, *edittext11, *edittext12;
GLUI_EditText    *edittext13, *edittext14, *edittext15;

GLUI_CommandLine *cmd_line;
GLUI_Panel       *obj_panel;
GLUI_Button      *open_console_btn;

/********** User IDs for callbacks ********/
#define OPEN_CONSOLE_ID      100
#define CMD_HIST_RESET_ID    101
#define CMD_CLOSE_ID         102
#define LIGHT0_ENABLED_ID    200
#define LIGHT1_ENABLED_ID    201
#define LIGHT0_INTENSITY_ID  250
#define LIGHT1_INTENSITY_ID  251

/********** Miscellaneous global variables **********/

GLfloat light0_ambient[] = { 0.1f, 0.1f, 0.3f, 1.0f };
GLfloat light0_diffuse[] = { .6f, .6f, 1.0f, 1.0f };
GLfloat light0_position[] = { .5f, .5f, 1.0f, 0.0f };

GLfloat light1_ambient[] = { 0.1f, 0.1f, 0.3f, 1.0f };
GLfloat light1_diffuse[] = { .9f, .6f, 0.0f, 1.0f };
GLfloat light1_position[] = { -1.0f, -1.0f, 1.0f, 0.0f };

/**************************************** control_cb() *******************/
/* GLUI control callback                                                 */

void control_cb(int control)
{
	if (control == LIGHT0_ENABLED_ID) {
		if (light0_enabled) {
			glEnable(GL_LIGHT0);
			light0_spinner->enable();
		}
		else {
			glDisable(GL_LIGHT0);
			light0_spinner->disable();
		}
	}
	else if (control == LIGHT1_ENABLED_ID) {
		if (light1_enabled) {
			glEnable(GL_LIGHT1);
			light1_spinner->enable();
		}
		else {
			glDisable(GL_LIGHT1);
			light1_spinner->disable();
		}
	}
	else if (control == LIGHT0_INTENSITY_ID) {
		float v[] = { light0_diffuse[0],  light0_diffuse[1],
			light0_diffuse[2],  light0_diffuse[3] };

		v[0] *= light0_intensity;
		v[1] *= light0_intensity;
		v[2] *= light0_intensity;

		glLightfv(GL_LIGHT0, GL_DIFFUSE, v);
	}
	else if (control == LIGHT1_INTENSITY_ID) {
		float v[] = { light1_diffuse[0],  light1_diffuse[1],
			light1_diffuse[2],  light1_diffuse[3] };

		v[0] *= light1_intensity;
		v[1] *= light1_intensity;
		v[2] *= light1_intensity;

		glLightfv(GL_LIGHT1, GL_DIFFUSE, v);
	}
}

/**************************************** pointer_cb() *******************/
/* GLUI control pointer callback                                         */
/* You can also use a function that takes a GLUI_Control pointer  as its */
/* argument.  This can simplify things sometimes, and reduce the clutter */
/* of global variables by giving you the control pointer directly.       */
/* For instance here we didn't need an additional global ID for the      */
/* cmd_line because we can just compare pointers directly.               */

void pointer_cb(GLUI_Control* control)
{
	if (control->get_id() == OPEN_CONSOLE_ID) {
		/****** Make command line window ******/
		cmd_line_glui = GLUI_Master.create_glui("Enter command:",
			0, 50, 500);

		cmd_line = new GLUI_CommandLine(
			cmd_line_glui, "Command (try 'exit'):", NULL, -1, pointer_cb);
		cmd_line->set_w(400);  /** Widen 'command line' control **/

		GLUI_Panel *panel = new GLUI_Panel(cmd_line_glui, "", GLUI_PANEL_NONE);
		new GLUI_Button(panel, "Clear History", CMD_HIST_RESET_ID, pointer_cb);
		new GLUI_Column(panel, false);
		new GLUI_Button(panel, "Close", CMD_CLOSE_ID, pointer_cb);

		cmd_line_glui->set_main_gfx_window(main_window);

		control->disable();
	}
	else if (control->get_id() == CMD_CLOSE_ID) {
		open_console_btn->enable();
		control->glui->close();
	}
	else if (control == cmd_line) {
		/*** User typed text into the 'command line' window ***/
		printf("Command (%d): %s\n", counter, cmd_line->get_text());
		std::string text = cmd_line->get_text();
		if (text == "exit" || text == "quit")
			exit(0);
	}
	else if (control->get_id() == CMD_HIST_RESET_ID) {
		cmd_line->reset_history();
	}

}

/**************************************** myGlutKeyboard() **********/

void myGlutKeyboard(unsigned char Key, int x, int y)
{
	switch (Key)
	{
		// A few keys here to test the sync_live capability.
	case 'o':
		// Cycle through object types
		++obj_type %= 3;
		GLUI_Master.sync_live_all();
		break;
	case 'w':
		// Toggle wireframe mode
		wireframe = !wireframe;
		GLUI_Master.sync_live_all();
		break;
	case 27:
	case 'q':
		exit(0);
		break;
	};

	glutPostRedisplay();
}


/***************************************** myGlutMenu() ***********/

void myGlutMenu(int value)
{
	myGlutKeyboard(value, 0, 0);
}


/***************************************** myGlutIdle() ***********/

void myGlutIdle(void)
{
	/* According to the GLUT specification, the current window is
	undefined during an idle callback.  So we need to explicitly change
	it if necessary */
	if (glutGetWindow() != main_window)
		glutSetWindow(main_window);


	glutPostRedisplay();

	/****************************************************************/
	/*            This demonstrates GLUI::sync_live()               */
	/*   We change the value of a variable that is 'live' to some   */
	/*   control.  We then call sync_live, and the control          */
	/*   associated with that variable is automatically updated     */
	/*   with the new value.  This frees the programmer from having */
	/*   to always remember which variables are used by controls -  */
	/*   simply change whatever variables are necessary, then sync  */
	/*   the live ones all at once with a single call to sync_live  */
	/****************************************************************/

	counter++;

	glui->sync_live();

}

/***************************************** myGlutMouse() **********/

void myGlutMouse(int button, int button_state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON && button_state == GLUT_DOWN) {
		last_x = x;
		last_y = y;
	}
}


/***************************************** myGlutMotion() **********/

void myGlutMotion(int x, int y)
{
	rotationX += (float)(y - last_y);
	rotationY += (float)(x - last_x);

	last_x = x;
	last_y = y;

	glutPostRedisplay();
}

/**************************************** myGlutReshape() *************/

void myGlutReshape(int x, int y)
{
	xy_aspect = (float)x / (float)y;
	glViewport(0, 0, x, y);

	glutPostRedisplay();
}
// Initialization routine.
void setup(void)
{
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	glEnable(GL_DEPTH_TEST); // Enable depth testing.

							 // Turn on OpenGL lighting.
	glEnable(GL_LIGHTING);

	// Light property vectors.
	float lightAmb[] = { 0.0, 0.0, 0.0, 1.0 };
	float lightDifAndSpec[] = { 1.0, 1.0, 1.0, 1.0 };
	float lightPos[] = { 0.0, 1.0, 0.0, 0.0 }; // Overhead directional light source (e.g., sun).
	float globAmb[] = { 0.2, 0.2, 0.2, 1.0 };

	// Light properties.
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmb);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDifAndSpec);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightDifAndSpec);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

	glEnable(GL_LIGHT0); // Enable particular light source.
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, globAmb); // Global ambient light.

													 // Material property vectors.
	float matSpec[] = { 1.0, 1.0, 1.0, 1.0 };
	float matShine[] = { 50.0 };

	// Material properties.
	glMaterialfv(GL_FRONT, GL_SPECULAR, matSpec);
	glMaterialfv(GL_FRONT, GL_SHININESS, matShine);

	// Enable color material mode.
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

	// Cull back faces.
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
}

/***************************************** myGlutDisplay() *****************/

void myGlutDisplay(void)
{
	int i = 0;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();
	gluLookAt(0.0, 5.0, 30.0, 0.0, 10.0, 0.0, 0.0, 1.0, 0.0);
	//gluLookAt(0.0, 5.0, 20.0,
	//	0.0, 5.0, 10.0,
	//	0.0, 1.0, 0.0);

	// Draw checkered floor.
	glShadeModel(GL_FLAT); // Flat shading to get the checkered pattern.
	for (float z = 100.0; z > -100.0; z -= 5.0)
	{
		glBegin(GL_TRIANGLE_STRIP);
		for (float x = -100.0; x < 100.0; x += 5.0)
		{
			if (i % 2) glColor3f(0.0, 0.5, 0.5);
			else glColor3f(1.0, 1.0, 1.0);
			glNormal3f(0.0, 1.0, 0.0);
			glVertex3f(x, 0.0, z - 5.0);
			glVertex3f(x, 0.0, z);
			i++;
		}
		glEnd();
		i++;
	}
	for (float y1 = 100.0; y1 > -100.0; y1 -= 5.0)
	{
		glBegin(GL_TRIANGLE_STRIP);
		for (float x1 = -100.0; x1 < 100.0; x1 += 5.0)
		{
			if (i % 2) glColor3f(0.8, 0.5, 0.5);
			else glColor3f(1.0, 1.0, 1.0);
			glNormal3f(0.0, 0.0, 1.0);
			glVertex3f(x1, y1-5.0, 0.0);
			glVertex3f(x1, y1, 0.0);
			i++;
		}
		glEnd();
		i++;
	}
	glPushMatrix();
	glTranslatef(0, 0, -50.0);
	glBegin(GL_QUADS);
	/* Back Wall */
	glColor3f(1.0f, 0.0f, 0.0f);
	glNormal3f(0, 0, 1);
	glVertex3f(-200, 0, 0);
	glVertex3f(200, 0, 0);
	glVertex3f(200, 200, 0);
	glVertex3f(-200, 200, 0);
	glEnd();
	glPopMatrix();

	glPushMatrix();
	glTranslatef(-30, 0, 0.0);
	glBegin(GL_QUADS);
	/* side Wall */
	glColor3f(0.0f, 1.0f, 0.0f);
	glNormal3f(-1, 0, 0);
	glVertex3f(0, 200, -200);
	glVertex3f(0, 200, 200);
	glVertex3f(0, -200, 200);
	glVertex3f(0, -200, -200);
	glEnd();
	glPopMatrix();



	////////
	//World co-ordinates
	//x-axis
	glPopMatrix();
	glPushMatrix();
	glScalef(1.0, 0.0, 1.0);
	glColor3f(0.0, 0.5, 0.8);
	Arrow(0, 0, 0, 5, 0, 0, 0.2);
	glPopMatrix();
	//y-axis
	glPopMatrix();
	glPushMatrix();
	//glScalef(0.0, 0.0, 1.0);
	glColor3f(0.0, 0.5, 0.8);
	Arrow(0, 0, 0, 0, 5, 0, 0.2);
	glPopMatrix();
	//z-axis
	glPopMatrix();
	glPushMatrix();
	glScalef(1.0, 0.0, 1.0);
	glColor3f(0.0, 0.5, 0.8);
	Arrow(0, 0, 0, 0, 0, 5, 0.2);
	glPopMatrix();

	////
	glShadeModel(GL_SMOOTH); // Restore smooth shading.
	glDisable(GL_LIGHTING); // Turn off lighting as shadows are not lit.
	glDisable(GL_DEPTH_TEST); // Turn off depth testing so that shadows are drawn over the
							  // floor without comparing distance from the viewer.
							  //Draw circle markers
	float x_pos[5] = { -7.0, 7.0, 10.0, -10.0 , 0.0 };
	float y_pos[5] = { 6.0, 5.0, 6.0, 5.0, 1.0 };
	float z_pos[5] = { 18, 18, 6, 6, 12 };

	glEnable(GL_DEPTH_TEST); // Restore depth testing.
	glEnable(GL_LIGHTING); // Restore lighting.

	

	//Check the new position of goal frame within the robot's workspace

	float dist = sqrt(pow(ini_pos[0] - goal[0], 2) + pow(ini_pos[2] - goal[2], 2));
	if (dist > 8) {
		dest = 1;
		baseRot=0;
		memcpy(&latest_matrix,&final_matrix,sizeof(transform));
		if (goal1[0]!=goal[0]) {
			baseRot = 0;
		}
		if (goal1[2] != goal[2]) {
			baseRot = 90;
		}
		//baseRot = br;
	}
	else if(dist>0&dist<8)
	{
		dest = 0;
		//memcpy(&latest_matrix, &final_matrix, sizeof(transform));
		
			latest_matrix.px = goal[0]*cos(baseRot*RADPERDEG)-goal[2]*sin(baseRot*RADPERDEG) - baseframe[0]*cos(baseRot*RADPERDEG)+baseframe[2]*sin(baseRot*RADPERDEG);
			latest_matrix.py = goal[1] - baseframe[1];
			latest_matrix.pz = goal[0] * sin(baseRot*RADPERDEG) + goal[2] * cos(baseRot*RADPERDEG) - baseframe[0] * sin(baseRot*RADPERDEG) - baseframe[2] * cos(baseRot*RADPERDEG);
			memcpy(&final_matrix, &latest_matrix,  sizeof(transform));
}
	else {
		//memcpy(&latest_matrix, &final_matrix, sizeof(transform));
	}
	//cout << baseRot;
	if (di_t == tf1) {
		di_t = 0;
		dest = 0;
		ini_pos[0] = baseframe[0];
		ini_pos[1] = baseframe[1];
		ini_pos[2] = baseframe[2];
		goal1[0] = goal[0];
		goal1[1] = goal[1];
		goal1[2] = goal[2];
		latest_matrix.px = goal[0] * cos(baseRot*RADPERDEG) - goal[2] * sin(baseRot*RADPERDEG) - baseframe[0] * cos(baseRot*RADPERDEG) + baseframe[2] * sin(baseRot*RADPERDEG);
		latest_matrix.py = goal[1] - baseframe[1];
		latest_matrix.pz = goal[0] * sin(baseRot*RADPERDEG) + goal[2] * cos(baseRot*RADPERDEG) - baseframe[0] * sin(baseRot*RADPERDEG) - baseframe[2] * cos(baseRot*RADPERDEG);
		memcpy(&final_matrix, &latest_matrix, sizeof(transform));
	}
	else{
		if (dest == 1) {
			baseframe[0] = trajectory_mapping(di_t, tf, ti, ini_pos[0], goal[0]);
			baseframe[2] = trajectory_mapping(di_t, tf, ti, ini_pos[2], goal[2]);
			di_t += 0.5;
		}
	}

	tem1 = InvKin(latest_matrix);
	tem = InvKin(final_matrix);
	//cout << tem.t1;
		fin.t1 = max(tem1.t1,0);
		fin.t2 = max(tem1.t2,20);
		fin.t3 = max(tem1.t3,0);
		fin.t4 = max(tem1.t4,0);
		fin.t5 = max(tem1.t5,34);
		fin.t6 = max(tem1.t6,0);
	
	
	// Trajectory planning
	if (!((ini.t1== fin.t1)& (ini.t2 == fin.t2)& (ini.t3 == fin.t3)& (ini.t4 == fin.t4)& (ini.t5 == fin.t5)&(ini.t6 == fin.t6))) {
		st = 1;
	}
	//cout<<st;
	if (curr_t == tf) {
		curr_t = 0;
		st = 0;
		ini.t1 = fin.t1;
		ini.t2 = fin.t2;
		ini.t3 = fin.t3;
		ini.t4 = fin.t4;
		ini.t5 = fin.t5;
		ini.t6 = fin.t6;
		//fin = InvKin(latest_matrix);
		//latest_matrix = ForwardKin(fin);
	}
	else {
		if (st == 1) {
			theta_1 = trajectory_mapping(curr_t, tf, ti, ini.t1, fin.t1);
			theta_2 = trajectory_mapping(curr_t, tf, ti, ini.t2, fin.t2);
			theta_3 = trajectory_mapping(curr_t, tf, ti, ini.t3, fin.t3);
			theta_4 = trajectory_mapping(curr_t, tf, ti, ini.t4, fin.t4);
			theta_5 = trajectory_mapping(curr_t, tf, ti, ini.t5, fin.t5);
			theta_6 = trajectory_mapping(curr_t, tf, ti, ini.t6, fin.t6);
			curr_t += 0.5;
		}
	}
	
	if (temp != baseframe[0] || temp1 !=baseRot || temp2 != baseframe[2]) {
		temp = baseframe[0];
		temp2 = baseframe[2];
		temp1 = baseRot;
		wh++;
	}

	if (wh == 18) {
		wh = 0;
	}
	//Check whether the robot will tip or not
	joint new_joints;
	new_joints.t1 = theta_1;
	new_joints.t2 = theta_2;
	new_joints.t3 = theta_3;
	new_joints.t4 = theta_4;
	new_joints.t5 = theta_5;
	new_joints.t6 = theta_6;
	float tipping = torque(new_joints);
	// color of base
	float basecolor[3];
	if (tipping == 1) {
		basecolor[0] = 0.8; basecolor[1]=0.3; basecolor[2]=0.3;
	}else{
		basecolor[0] = 0.8; basecolor[1] = 0.6; basecolor[2] = 0.1;
	}
	//Goal Point
	glPushMatrix();
	glColor3f(0.6,0.6,0.9);
	glTranslated(goal[0],goal[1],goal[2]);
	glutSolidCube(0.5);
	glPopMatrix();
	//Wheels 1
	glPushMatrix();
	glColor3f(0.8, 0.3, 0.1);
	glTranslated(baseframe[0], baseframe[1], baseframe[2]);
	glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(0, 0, 2.7);
	//glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslatef(0.0,0.8,0.0);
	glRotatef(20*wh,0.0,0.0,1.0);
	glutSolidSphere(0.8, 7, 10);
	glPopMatrix();
	//front wheel
	glPushMatrix();
	glColor3f(0.8, 0.3, 0.1);
	glTranslated(baseframe[0], baseframe[1], baseframe[2]);
	glRotatef(baseRot, 0.0, 1.0, 0.0);
	glRotatef(180, 0.0, 1.0, 0.0);
	glTranslated(0, 0, 2.7);
	//glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslatef(0.0, 0.8, 0.0);
	glRotatef(20 * wh, 0.0, 0.0, 1.0);
	glutSolidSphere(0.8, 7, 10);
	glPopMatrix();



	//Wheels 2
	glPushMatrix();
	glColor3f(0.8, 0.3, 0.1);
	glTranslated(baseframe[0], baseframe[1], baseframe[2]);
	glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(2, 0, 2.7);
	//glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslatef(0.0, 0.8, 0.0);
	glRotatef(20 * wh, 0.0, 0.0, 1.0);
	glutSolidSphere(0.8, 7, 10);
	glPopMatrix();

	glPushMatrix();
	glColor3f(0.8, 0.3, 0.1);
	glTranslated(baseframe[0], baseframe[1], baseframe[2]);
	glRotatef(baseRot, 0.0, 1.0, 0.0);
	glRotatef(180, 0.0, 1.0, 0.0);
	glTranslated(2, 0, 2.7);
	//glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslatef(0.0, 0.8, 0.0);
	glRotatef(20 * wh, 0.0, 0.0, 1.0);
	glutSolidSphere(0.8, 7, 10);
	glPopMatrix();
	
	//Base
	glPushMatrix();
	glColor3f(basecolor[0], basecolor[1], basecolor[2]);
	//glColor3f(0.9, 0.9, 0.9);
	//glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(baseframe[0], baseframe[1], baseframe[2]);
	glRotatef(baseRot, 0.0, 1.0, 0.0);
	glutSolidSphere(3, 20, 10);
	//glutSolidCube(1);
	glPopMatrix();

	// 1
	
	glPushMatrix();
	//glColor3f(0.3, 0.8, 0.1);
	glColor3f(0.7, 0.7, 0.7);
	//glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(baseframe[0], baseframe[1], baseframe[2]);
	glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(frame1[0], frame1[1], frame1[2]);
	glRotatef(theta_1, 0.0, 1.0, 0.0);
	glutSolidSphere(2, 32, 32);
	glPopMatrix();
	// 2

	glPushMatrix();
	//glColor3f(0.8, 0.3, 0.1);
	glColor3f(0.5, 0.5, 0.5);
	//glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(baseframe[0], baseframe[1], baseframe[2]);
	glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(frame1[0], frame1[1], frame1[2]);
	glRotatef(theta_1, 0.0, 1.0, 0.0);
	glRotatef(theta_2, -1.0, 0.0, 0.0);
	drawCylinderWithCaps(4, 0.5, 32, 32);
	glPopMatrix();

	// 3
	
	//Joint
	glPushMatrix();
	//glColor3f(0.8, 0.6, 0.1);
	glColor3f(0.2, 0.2, 0.2);
	//glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(baseframe[0], baseframe[1], baseframe[2]);
	glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(frame1[0], frame1[1], frame1[2]);
	glRotatef(theta_1, 0.0, 1.0, 0.0);
	glRotatef(theta_2, -1.0, 0.0, 0.0);
	glTranslatef(frame3[0], frame3[1], frame3[2]);
	glutSolidSphere(0.5, 32, 32);
	glPopMatrix();
	//link

	glPushMatrix();
	//glColor3f(0.8, 0.3, 0.1);
	glColor3f(0.5, 0.5, 0.5);
	//glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(baseframe[0], baseframe[1], baseframe[2]);
	glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(frame1[0], frame1[1], frame1[2]);
	glRotatef(theta_1, 0.0, 1.0, 0.0);
	glRotatef(theta_2, -1.0, 0.0, 0.0);
	glTranslatef(frame3[0], frame3[1], frame3[2]);
	glRotatef(theta_3, 1.0, 0.0, 0.0);
	drawCylinderWithCaps(3, 0.5, 32, 32);
	glPopMatrix();
	// 4	
	
	glPushMatrix();
	//glColor3f(0.8, 0.6, 0.1);
	glColor3f(0.7, 0.7, 0.7);
	//glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(baseframe[0], baseframe[1], baseframe[2]);
	glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(frame1[0], frame1[1], frame1[2]);	
	glRotatef(theta_1, 0.0, 1.0, 0.0);
	glRotatef(theta_2, -1.0, 0.0, 0.0);
	glTranslatef(frame3[0], frame3[1], frame3[2]);
	glRotatef(theta_3, 1.0, 0.0, 0.0);
	glTranslatef(frame4[0], frame4[1], frame4[2]);
	glRotatef(theta_4, 0.0, 0.0, 1.0);
	//glutSolidSphere(1, 32, 32);
	glutWireCube(1.5);
	glPopMatrix();

	//Joint
	glPushMatrix();
	//glColor3f(0.8, 0.3, 0.1);
	glColor3f(0.9, 0.9, 0.9);
	//glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(baseframe[0], baseframe[1], baseframe[2]);
	glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(frame1[0], frame1[1], frame1[2]);
	glRotatef(theta_1, 0.0, 1.0, 0.0);
	glRotatef(theta_2, -1.0, 0.0, 0.0);
	glTranslatef(frame3[0], frame3[1], frame3[2]);
	glRotatef(theta_3, 1.0, 0.0, 0.0);
	glTranslatef(frame4[0], frame4[1], frame4[2]);
	glRotatef(theta_4, 0.0, 0.0, 1.0);
	glRotatef(-90.0, 0.0, 1.0, 0.0);
	glTranslatef(0.0, 0.0, -1.5 / 2);
	drawCylinderWithCaps(1.5, 0.2, 32, 32);
	glPopMatrix();

	// 5

	//upper pen
	glPushMatrix();
	//glColor3f(0.8, 0.3, 0.1);
	glColor3f(0.9, 0.9, 0.9);
	//glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(baseframe[0], baseframe[1], baseframe[2]);
	glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(frame1[0], frame1[1], frame1[2]);
	glRotatef(theta_1, 0.0, 1.0, 0.0);
	glRotatef(theta_2, -1.0, 0.0, 0.0);
	glTranslatef(frame3[0], frame3[1], frame3[2]);
	glRotatef(theta_3, 1.0, 0.0, 0.0);
	glTranslatef(frame4[0], frame4[1], frame4[2]);
	glRotatef(theta_4, 0.0, 0.0, 1.0);
	glRotatef(theta_5, -1.0, 0.0, 0.0);
	drawFlexibleCylinderWithCaps(2, 0.2, 0.3, 32, 32);
	glPopMatrix();
	//lower pen
	/*
	glPushMatrix();
	//glColor3f(0.8, 0.3, 0.1);
	glColor3f(0.9, 0.9, 0.9);
	glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(baseframe[0], baseframe[1], baseframe[2]);
	glTranslated(frame1[0], frame1[1], frame1[2]);
	glRotatef(theta_1, 0.0, 1.0, 0.0);
	glRotatef(theta_2, -1.0, 0.0, 0.0);
	glTranslatef(frame3[0], frame3[1], frame3[2]);
	glRotatef(theta_3, 1.0, 0.0, 0.0);
	glTranslatef(frame4[0], frame4[1], frame4[2]);
	glRotatef(theta_4, 0.0, 0.0, 1.0);
	glRotatef(theta_5, -1.0, 0.0, 0.0);
	glTranslatef(0.0, 0.0, -3.0);
	drawFlexibleCylinderWithCaps(3, 0.0, 0.2, 32, 32);
	glPopMatrix();*/


	// 6 
	/*
	glPushMatrix();
	glColor3f(0.8, 0.6, 0.1);
	//glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(baseframe[0], baseframe[1], baseframe[2]);
	glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(frame1[0], frame1[1], frame1[2]);
	glRotatef(theta_1, 0.0, 1.0, 0.0);
	glRotatef(theta_2, -1.0, 0.0, 0.0);
	glTranslatef(frame3[0], frame3[1], frame3[2]);
	glRotatef(theta_3, 1.0, 0.0, 0.0);
	glTranslatef(frame4[0], frame4[1], frame4[2]);
	glRotatef(theta_4, 0.0, 0.0, 1.0);
	glRotatef(theta_5, -1.0, 0.0, 0.0);
	glTranslatef(frame6[0], frame6[1], frame6[2]);
	glRotatef(theta_6, 0.0, 0.0, 1.0);
	drawFlexibleCylinderWithCaps(2, 0.3, 0.5, 32, 32);
	glPopMatrix();*/

	glPushMatrix();
	glColor3f(0.8, 0.6, 0.1);
	//glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(baseframe[0], baseframe[1], baseframe[2]);
	glRotatef(baseRot, 0.0, 1.0, 0.0);
	glTranslated(frame1[0], frame1[1], frame1[2]);
	glRotatef(theta_1, 0.0, 1.0, 0.0);
	glRotatef(theta_2, -1.0, 0.0, 0.0);
	glTranslatef(frame3[0], frame3[1], frame3[2]);
	glRotatef(theta_3, 1.0, 0.0, 0.0);
	glTranslatef(frame4[0], frame4[1], frame4[2]);
	glRotatef(theta_4, 0.0, 0.0, 1.0);
	glRotatef(theta_5, -1.0, 0.0, 0.0);
	glTranslatef(frame6[0], frame6[1], frame6[2]);
	glRotatef(theta_6, 0.0, 0.0, 1.0);
	//glTranslatef(framecam[0], framecam[1], framecam[2]);
	glutSolidCube(1.0);
	glPopMatrix();
	
	//Get 
	if (curr_t == tf) {

		//fin = InvKin(latest_matrix);
		latest_matrix = ForwardKin(new_joints);
	}

	final_matrix = ForwardKin(new_joints);
	glFlush();
}

void init() {
	//glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-5.0, 5.0, -5.0, 5.0, 5.0, 100.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

}
/**************************************** main() ********************/

int main(int argc, char* argv[])
{
	/****************************************/
	/*   Initialize GLUT and create window  */
	/****************************************/

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
	glutInitWindowPosition(80, 80);
	glutInitWindowSize(1500, 800);
	main_window = glutCreateWindow("Phantom Haptic Device");
	setup();
		ini.t1 = 60; ini.t2 = 30; ini.t3 = 60; ini.t4 = 30; ini.t5 = 60; ini.t6 = 30;
	//fin.t1 = 0; fin.t2 = 60; fin.t3 = 30; fin.t4 = 50; fin.t5 = 90; fin.t6 = 30;
		latest_matrix.px = 0.66; latest_matrix.py = 2.14; latest_matrix.pz = -11.39 ; 
		latest_matrix.r11 = 0.385; latest_matrix.r12 = 0.421; latest_matrix.r13 = -0.821;
		latest_matrix.r21 = -0.906; latest_matrix.r22 = 0.0044; latest_matrix.r23 = -0.9223;
		latest_matrix.r31 = -0.174; latest_matrix.r32 = 0.907; latest_matrix.r33 = 0.383 ;
		ini_pos[0] = baseframe[0];
		ini_pos[1] = baseframe[1];
		ini_pos[2] = baseframe[2];
	fin = InvKin(latest_matrix);
	glutDisplayFunc(myGlutDisplay);
	init();
	//glutReshapeFunc(myGlutReshape);
	glutKeyboardFunc(myGlutKeyboard);
	glutMotionFunc(myGlutMotion);
	glutMouseFunc(myGlutMouse);


	/****************************************/
	/*         Here's the GLUI code         */
	/****************************************/

	printf("GLUI version: %3.2f\n", GLUI_Master.get_version());

	glui = GLUI_Master.create_glui("GLUI", 0, 400, 50); /* name, flags,
														x, and y */
	new GLUI_StaticText(glui, "GLUI Phantom Omni");
	obj_panel = new GLUI_Panel(glui, "Joint Angles");

	/***** Control for the object type *****/

	GLUI_Panel *type_panel = new GLUI_Panel(obj_panel, "Degrees");

	spinner1 =
		new GLUI_Spinner(obj_panel, "Joint 1:", &theta_1);
	spinner1->set_int_limits(0, 360);
	spinner1->set_alignment(GLUI_ALIGN_RIGHT);

	spinner2 =
		new GLUI_Spinner(obj_panel, "Joint 2:", &theta_2);
	spinner2->set_float_limits(20.0f, 90.0f);
	spinner2->set_alignment(GLUI_ALIGN_RIGHT);

	spinner3 =
		new GLUI_Spinner(obj_panel, "Joint 3:", &theta_3);
	spinner3->set_float_limits(0.0f, 90.0f);
	spinner3->set_alignment(GLUI_ALIGN_RIGHT);

	spinner4 =
		new GLUI_Spinner(obj_panel, "Joint 4:", &theta_4);
	spinner4->set_float_limits(0.0f, 270.0f);
	spinner4->set_alignment(GLUI_ALIGN_RIGHT);

	spinner5 =
		new GLUI_Spinner(obj_panel, "Joint 5:", &theta_5);
	spinner5->set_float_limits(34.0f, 134.0f);
	spinner5->set_alignment(GLUI_ALIGN_RIGHT);

	spinner6 =
		new GLUI_Spinner(obj_panel, "Joint 6:", &theta_6);
	spinner6->set_float_limits(0.0f, 280.0f);
	spinner6->set_alignment(GLUI_ALIGN_RIGHT);


	new GLUI_Separator(obj_panel);

	spinner7 =
		new GLUI_Spinner(obj_panel, "rotation:", &baseframe[0]);
	spinner7->set_float_limits(-27.0f, 27.0f);
	spinner7->set_alignment(GLUI_ALIGN_RIGHT);

	spinner7 =
		new GLUI_Spinner(obj_panel, "rotation:", &baseframe[2]);
	spinner7->set_float_limits(-27.0f, 27.0f);
	spinner7->set_alignment(GLUI_ALIGN_RIGHT);

	spinner8 =
		new GLUI_Spinner(obj_panel, "baseRotate:", &baseRot);
	spinner8->set_float_limits(-360.0f, 360.0f);
	spinner8->set_alignment(GLUI_ALIGN_RIGHT);
/*
	edittext1 = new GLUI_EditText(obj_panel, "Px:", &latest_matrix.px);
	edittext1->set_w(150);

	edittext2 = new GLUI_EditText(obj_panel, "Py:", &latest_matrix.py);
	edittext2->set_w(150);

	edittext3 = new GLUI_EditText(obj_panel, "Pz:", &latest_matrix.pz);
	edittext3->set_w(150);*/
	new GLUI_Separator(obj_panel);
	edittext4 = new GLUI_EditText(obj_panel, "r11:", &latest_matrix.r11);
	edittext4->set_w(150);

	edittext5 = new GLUI_EditText(obj_panel, "r12:", &latest_matrix.r12);
	edittext5->set_w(150);

	edittext6 = new GLUI_EditText(obj_panel, "r13:", &latest_matrix.r13);
	edittext6->set_w(150);

	edittext7 = new GLUI_EditText(obj_panel, "r21:", &latest_matrix.r21);
	edittext7->set_w(150);

	edittext8 = new GLUI_EditText(obj_panel, "r22:", &latest_matrix.r22);
	edittext8->set_w(150);

	edittext9 = new GLUI_EditText(obj_panel, "r23:", &latest_matrix.r23);
	edittext9->set_w(150);

	edittext10 = new GLUI_EditText(obj_panel, "r31:", &latest_matrix.r31);
	edittext10->set_w(150);

	edittext11 = new GLUI_EditText(obj_panel, "r32:", &latest_matrix.r32);
	edittext11->set_w(150);

	edittext12 = new GLUI_EditText(obj_panel, "r33:", &latest_matrix.r33);
	edittext12->set_w(150);
	new GLUI_Separator(obj_panel);
	edittext13 = new GLUI_EditText(obj_panel, "goalx:", &goal[0]);
	edittext13->set_w(150);
	edittext13->set_float_limits(-26.0f, 26.0f);

	edittext14 = new GLUI_EditText(obj_panel, "goaly:", &goal[1]);
	edittext14->set_w(150);
	edittext14->set_float_limits(0.0f, 20.0f);

	edittext15 = new GLUI_EditText(obj_panel, "goalz:", &goal[2]);
	edittext15->set_w(150);
	edittext15->set_float_limits(-26.0f, 26.0f);
	/****** A 'quit' button *****/

	new GLUI_Button(glui, "Quit", 0, (GLUI_Update_CB)exit);

	/**** Link windows to GLUI, and register idle callback ******/

	glui->set_main_gfx_window(main_window);

	/* We register the idle callback with GLUI, not with GLUT */
	GLUI_Master.set_glutIdleFunc(myGlutIdle);

	/**** Regular GLUT main loop ****/
	glutMainLoop();

	return EXIT_SUCCESS;
}

