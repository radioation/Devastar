
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <iostream>
#include <vector>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#define BUFFER_SIZE 64
#define BAUDRATE B38400            
#define SERIAL_DEVICE "/dev/ttyACM0"

#define SHOW_IMAGE
#define SHOW_CALC
#define SHOW_3D

#ifdef SHOW_3D
#include <GL/freeglut.h>


std::vector<cv::Vec3f> camera3dPts;
std::vector<cv::Point3f> worldPoints3d;
float quat3D[4];
float glCameraX = 0.0f;
float glCameraY = 0.0f;
float glCameraZ = 1.5f;
bool mouseDown = false;
float prevMouseX = 0.0;
float prevMouseY = 0.0;
int glWidth = 512;
int glHeight = 512;
float hit3dX = -1.0;
float hit3dY = -1.0;
float dot3f(float* a, float* b) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/////////////////////////////////////////////////////////////////////////////

void cross3f(float* a, float* b, float* retvect) {
	retvect[0] = a[1] * b[2] - a[2] * b[1];
	retvect[1] = a[2] * b[0] - a[0] * b[2];
	retvect[2] = a[0] * b[1] - a[1] * b[0];
}

/////////////////////////////////////////////////////////////////////////////

void mat4fXmat4f(float* a, float* b, float* retmat) {

	retmat[0] = a[0] * b[0] + a[1] * b[4] + a[2] * b[8] + a[3] * b[12];
	retmat[1] = a[0] * b[1] + a[1] * b[5] + a[2] * b[9] + a[3] * b[13];
	retmat[2] = a[0] * b[2] + a[1] * b[6] + a[2] * b[10] + a[3] * b[14];
	retmat[3] = a[0] * b[3] + a[1] * b[7] + a[2] * b[11] + a[3] * b[15];

	retmat[4] = a[4] * b[0] + a[5] * b[4] + a[6] * b[8] + a[7] * b[12];
	retmat[5] = a[4] * b[1] + a[5] * b[5] + a[6] * b[9] + a[7] * b[13];
	retmat[6] = a[4] * b[2] + a[5] * b[6] + a[6] * b[10] + a[7] * b[14];
	retmat[7] = a[4] * b[3] + a[5] * b[7] + a[6] * b[11] + a[7] * b[15];

	retmat[8] = a[8] * b[0] + a[9] * b[4] + a[10] * b[8] + a[11] * b[12];
	retmat[9] = a[8] * b[1] + a[9] * b[5] + a[10] * b[9] + a[11] * b[13];
	retmat[10] = a[8] * b[2] + a[9] * b[6] + a[10] * b[10] + a[11] * b[14];
	retmat[11] = a[8] * b[3] + a[9] * b[7] + a[10] * b[11] + a[11] * b[15];

	retmat[12] = a[12] * b[0] + a[13] * b[4] + a[14] * b[8] + a[15] * b[12];
	retmat[13] = a[12] * b[1] + a[13] * b[5] + a[14] * b[9] + a[15] * b[13];
	retmat[14] = a[12] * b[2] + a[13] * b[6] + a[14] * b[10] + a[15] * b[14];
	retmat[15] = a[12] * b[3] + a[13] * b[7] + a[14] * b[11] + a[15] * b[15];

}


void display3d() {
	// prep scene
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clear screen and depth buffers

	/*
	   glBegin(GL_POLYGON);
	   glColor3f(1, 0, 0); glVertex3f(-0.6, -0.75, 0.5);
	   glColor3f(0, 1, 0); glVertex3f(0.6, -0.75, 0);
	   glColor3f(0, 0, 1); glVertex3f(0, 0.75, 0);
	   glEnd();
	   */

	////// BEGIN VIEW TRANSFORM  /////////////////////////////////////////////////////////////
	//--- First get Camera position/rotation with LookAt (This is the View part of ModelView)
	float F[3];
	F[0] = 0.0 - glCameraX;
	F[1] = 0.0 - glCameraY;
	F[2] = 0.0 - glCameraZ;
	float fLen = sqrt((F[0] * F[0]) + (F[1] * F[1]) + (F[2] * F[2]));

	float f[3] = { F[0] / fLen, F[1] / fLen, F[2] / fLen };
	float UP[3] = { 0, 1, 0 };
	float upLen = sqrt((UP[0] * UP[0]) + (UP[1] * UP[1]) + (UP[2] * UP[2]));
	float up[3] = { UP[0] / upLen, UP[1] / upLen, UP[2] / upLen };

	float s[3]; // = {f[1]*up[2]-f[2]*up[1],  f[2]*up[0] - f[0]*up[2], f[0]*up[1] - f[1]*up[0] }; // s = f x up
	cross3f(f, up, s);
	float u[3];// = {s[1]*f[2]-s[2]*f[1],  s[2]*f[0] - s[0]*f[2], s[0]*f[1] - s[1]*f[0] };; // u = s x f
	cross3f(s, f, u);

	float M[16] = {
		s[0], s[1], s[2], 0,
		u[0], u[1], u[2], 0,
		-f[0], -f[1], -f[2], 0,
		0, 0, 0, 1
	};
	float T[16] = {
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		-glCameraX, -glCameraX,-glCameraZ, 1
	};


	float view[16];
	mat4fXmat4f(T, M, view);  // I'm using ROW vectors, so every new transform is PRE multiplied w/ current transform
	// convert quaternion to matrix so we can stuff it into OpenGL.

	float x = quat3D[0];
	float y = quat3D[1];
	float z = quat3D[2];
	float w = quat3D[3];

	float xx = x * x;
	float yy = y * y;
	float zz = z * z;

	float xy = x * y;
	float xz = x * z;

	float yz = y * z;

	float wx = w * x;
	float wy = w * y;
	float wz = w * z;

	float rotation[16] = {
		1 - 2.0f * (yy + zz),     2.0f * (xy + wz),       2.0f * (xz - wy), 0.0f,
		2.0f * (xy - wz),      1.0f - 2.0f * (xx + zz),        2.0f * (yz + wx), 0.0f,
		2.0f * (xz + wy),         2.0f * (yz - wx),       1.0f - 2.0f * (xx + yy), 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	};
	float modelview[16];
	mat4fXmat4f(rotation, view, modelview);
	////// END  MODEL TRANSFORM  /////////////////////////////////////////////////////////////
	// setup model view based on above transforms.
	glLoadIdentity();    // reset the ModelView Matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(modelview);

	// draw origin  (0,0,0)
	glPointSize(6);
	glColor3f(1.0f, 1.0f, 1.0f);
	glBegin(GL_POINTS);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glEnd();


	// draw axis
	glLineWidth(2.0);
	glBegin(GL_LINES);
	glColor3f(1.0f, 0.0f, 0.0f);
	glVertex3f(0.3f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);

	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.3f, 0.0f);

	glColor3f(0.0f, 0.0f, 1.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.3f);
	glEnd();

	// draw board extents
	if (worldPoints3d.size() > 3) {
		glLineWidth(1.0);
		glBegin(GL_LINES);
		glColor3f(1.0f, 1.0f, 1.0f);

		glVertex3f(worldPoints3d[0].x / 1000.0f, worldPoints3d[0].y / 1000.0f, worldPoints3d[0].z / 1000.0f);
		glVertex3f(worldPoints3d[1].x / 1000.0f, worldPoints3d[1].y / 1000.0f, worldPoints3d[1].z / 1000.0f);

		glVertex3f(worldPoints3d[1].x / 1000.0f, worldPoints3d[1].y / 1000.0f, worldPoints3d[1].z / 1000.0f);
		glVertex3f(worldPoints3d[3].x / 1000.0f, worldPoints3d[3].y / 1000.0f, worldPoints3d[3].z / 1000.0f);

		glVertex3f(worldPoints3d[3].x / 1000.0f, worldPoints3d[3].y / 1000.0f, worldPoints3d[3].z / 1000.0f);
		glVertex3f(worldPoints3d[2].x / 1000.0f, worldPoints3d[2].y / 1000.0f, worldPoints3d[2].z / 1000.0f);

		glVertex3f(worldPoints3d[2].x / 1000.0f, worldPoints3d[2].y / 1000.0f, worldPoints3d[2].z / 1000.0f);
		glVertex3f(worldPoints3d[0].x / 1000.0f, worldPoints3d[0].y / 1000.0f, worldPoints3d[0].z / 1000.0f);
		glEnd();

	}


	// draw camera vector if available.
	if (camera3dPts.size() > 1) {
		glColor3f(1.0f, 1.0f, 0.0f);
		glBegin(GL_POINTS);
		glVertex3d(camera3dPts[0][0], camera3dPts[0][1], camera3dPts[0][2]);
		glEnd();

		glLineWidth(1.0);
		glBegin(GL_LINES);
		glVertex3d(camera3dPts[0][0], camera3dPts[0][1], camera3dPts[0][2]);
		glVertex3d(camera3dPts[1][0], camera3dPts[1][1], camera3dPts[1][2]);
		glEnd();
	}

		glColor3f(1.0f, 1.0f, 1.0f);
	glBegin(GL_POINTS);
	glVertex3d(hit3dX, hit3dY, 0.0f);
	glEnd();


	// Flush drawing command buffer to make drawing happen as soon as possible.
	glFlush();
}

void mouseClick(int button, int state, int x, int y)
{

	switch (button)
	{
		case GLUT_LEFT_BUTTON:

			if(state == GLUT_DOWN)
			{
				prevMouseX = x;
				prevMouseY = y;
				mouseDown = true;
			} else if( state == GLUT_UP ) {
				prevMouseX = x;
				prevMouseY = y;
				mouseDown = true;
			}
			break;
		case 3:  //mouse wheel scrolls
			glCameraZ -= 0.1f;

			if (glCameraZ < 1.3f) {
				glCameraZ = 1.3f;
			}
			break;
		case 4:
			glCameraZ += 0.1f;
			if (glCameraZ > 12.0f) {
				glCameraZ = 12.0f;
			}
			break;
		default:
			break;
	}
	glutPostRedisplay();
}

void mouseMove(int x, int y) {

	float center_x = (float)glWidth / 2.0f;
	float center_y = (float)glHeight / 2.0f;

	float radius = center_x; // go full width of screen (this assumes portrait)

	// FIRST:  GET axis/angle of rotations using
	//--  imaginary trackball mappings for p1 (start) and p2(end) vectors
	float p1[3];
	p1[0] = prevMouseX - center_x;
	p1[1] = center_y - prevMouseY; // top to bottom
	float p1_len = sqrt(p1[0] * p1[0] + p1[1] * p1[1]);
	// is p1 within radius
	if (p1_len > radius - 1.0) {
		//If a point is outside the circle, project
		// it to the nearest point on the circle
		// set z to 0 and renormalize x,y
		p1[0] = p1[0] / p1_len;
		p1[1] = p1[1] / p1_len;
		p1[2] = 0.0f;
	} else {
		p1[2] = sqrt(radius * radius - p1[0] * p1[0] - p1[1] * p1[1]);
	}

	float p2[3];
	p2[0] = ((float)x - center_x);
	p2[1] = (center_y - (float)y); // flip this stuff top to bottom
	float p2_len = sqrt(p2[0] * p2[0] + p2[1] * p2[1]);
	// check if p2 within radius
	if (p2_len > radius - 1.0) {
		p2[0] = p2[0] / p2_len;
		p2[1] = p2[1] / p2_len;
		p2[2] = 0.0f;
	}
	else {
		p2[2] = sqrt(radius * radius - p2[0] * p2[0] - p2[1] * p2[1]);
	}
	// cross product of p1 and p2 gets us axis of rotation
	float axis[3];
	cross3f(p1, p2, axis);
	// angle is from   | sin(theta) | |n|/  |p1|.|p2|  Need normalized vetors.
	float pn1[3];
	p1_len = sqrt(p1[0] * p1[0] + p1[1] * p1[1] + p1[2] * p1[2]);
	pn1[0] = p1[0] / p1_len;
	pn1[1] = p1[1] / p1_len;
	pn1[2] = p1[2] / p1_len;
	float pn2[3];
	p2_len = sqrt(p2[0] * p2[0] + p2[1] * p2[1] + p2[2] * p2[2]);
	pn2[0] = p2[0] / p2_len;
	pn2[1] = p2[1] / p2_len;
	pn2[2] = p2[2] / p2_len;
	float dotprod = dot3f(pn1, pn2);
	float angle = acos(dotprod);  // could be blowing us out?
	if (__isnan(angle))
	{
		return;
	}
	// convert to quaternion
	// qx = ax * sin(angle/2)
	// qy = ay * sin(angle/2)
	// qz = az * sin(angle/2)
	// qw = cos(angle/2)
	float quat[4];
	float axis_len = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);  //
	float sin_angle = sin(angle / 2.0);     /// Some webpages use - of angle??
	quat[0] = sin_angle * axis[0] / axis_len;  // x
	quat[1] = sin_angle * axis[1] / axis_len;  // y
	quat[2] = sin_angle * axis[2] / axis_len;  // z
	quat[3] = cos(angle / 2.0);            // w

	if (__isnan(quat[0]) || __isnan(quat[1]) || __isnan(quat[2]) || __isnan(quat[3]))
	{
		return;
	}
	// multiply against current orientation
	float temp_quat[4];
	temp_quat[0] = quat[3] * quat3D[0] + quat[0] * quat3D[3] + quat[1] * quat3D[2] - quat[2] * quat3D[1];
	temp_quat[1] = quat[3] * quat3D[1] + quat[1] * quat3D[3] + quat[2] * quat3D[0] - quat[0] * quat3D[2];
	temp_quat[2] = quat[3] * quat3D[2] + quat[2] * quat3D[3] + quat[0] * quat3D[1] - quat[1] * quat3D[0];
	temp_quat[3] = quat[3] * quat3D[3] - quat[0] * quat3D[0] - quat[1] * quat3D[1] - quat[2] * quat3D[2];
	float mag = sqrt(temp_quat[0] * temp_quat[0]
			+ temp_quat[1] * temp_quat[1]
			+ temp_quat[2] * temp_quat[2]
			+ temp_quat[3] * temp_quat[3]
			);

	// save quat for next round
	quat3D[0] = temp_quat[0] / mag;
	quat3D[1] = temp_quat[1] / mag;
	quat3D[2] = temp_quat[2] / mag;
	quat3D[3] = temp_quat[3] / mag;
	prevMouseX = x;
	prevMouseY = y;

	glutPostRedisplay();
}

#endif


// intersect rectangle with a ray
// * Ray is defined as R0 + t * D
// 
// * Rectangle is represented with a corner point P0
// and two vectors ( S1 and S2 ) indicating the length
// of the sides of the  rectangle.

bool intersectRect(const cv::Vec3f& R0, // ray start
		const cv::Vec3f& D, // ray direction 
		const cv::Vec3f& P0,// Rectangle Origin 
		const cv::Vec3f& S1, // side 1. direction
		const cv::Vec3f& S2, // side 2. direction 
		const float& S1Len,  // side 1. length
		const float& S2Len,  // side 2. length
		float& u,            // u 
		float& v	     // v	
		) {

	// compute plane Normal 
	cv::Vec3f N = S1.cross(S2);
	float DdotN = D.dot(N);

	// get point on plane P
	auto a = ((P0 - R0).dot(N)) / D.dot(N);
	// we assume that P = R0 + a * D
	cv::Vec3f P = R0 + a * D;  // Point on Plane

	// make vector out of point on plane and orign
	cv::Vec3f P0P = P - P0;

	// project P0P vector onto the sides.
	u = P0P.dot(S1);
	v = P0P.dot(S2);
	return (u >= 0 && u <= S1Len && v >= 0 && v <= S2Len);

}



int main(int argc, char* argv[] )
{
	const float width = 510.0f;
	const float height = 260.0f;

	// Read in camera calibration calibration
	cv::FileStorage fs("calib.yml", cv::FileStorage::READ);
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	fs["camera_matrix"] >> cameraMatrix;
	fs["dist_coeffs"] >> distCoeffs;


	// setup videocapture
	cv::VideoCapture inputVideo;
	inputVideo.open(0);
	inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 480);


	// Setup object points
	std::vector<cv::Point3f> worldPoints;
	worldPoints.push_back(cv::Point3f(0, 0, 0));
	worldPoints.push_back(cv::Point3f(width, 0, 0));
	worldPoints.push_back(cv::Point3f(0, height, 0));
	worldPoints.push_back(cv::Point3f(width, height, 0));
	for( const auto& wp: worldPoints ) {
		worldPoints3d.push_back(wp);
	}
	// setup rectangle for intersection test
	cv::Vec3f S1, S2, P0;
	P0[0] = worldPoints[0].x;
	P0[1] = worldPoints[0].y;
	P0[2] = worldPoints[0].z;

	S1[0] = worldPoints[1].x - P0[0];
	S1[1] = worldPoints[1].y - P0[1];
	S1[2] = worldPoints[1].z - P0[2];
	S1 = cv::normalize(S1);

	S2[0] = worldPoints[2].x - P0[0];
	S2[1] = worldPoints[2].y - P0[1];
	S2[2] = worldPoints[2].z - P0[2];
	S2 = cv::normalize(S2);

#ifdef SHOW_3D
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);

	glutInitWindowPosition(80, 80);
	glutInitWindowSize(500, 500);
	glutCreateWindow("3D View");

	glutDisplayFunc(display3d);
	glutMouseFunc(mouseClick);
	glutMotionFunc(mouseMove);


	quat3D[0] = 1.0;
	quat3D[1] = 0.0;
	quat3D[2] = 0.0;
	quat3D[3] = 0.0;

	// shader model
	glShadeModel(GL_SMOOTH);  //  shading technique used on primitives ( GL_FLAT or GL_SMOOTH)
	// set color used to clear
	glClearColor(0.0f, 0.0f, 0.1f, 0.5f);
	// setup depth buffer
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);  // needed to allow  depth buffer to work
	glDepthFunc(GL_LEQUAL);  // passes if incomoing z falue is less than or equali to stored Z

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // perspective calculations


	glViewport(0, 0, glWidth, glHeight); // set to current window size



	// reset the projection matrix.
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	// figure out the Aspect Ratio of the window
	gluPerspective(45.0f, (GLfloat)glWidth / (GLfloat)glHeight, 0.1f, 100.0f);



	// reset the Model/View matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

#endif

	// setup serial communication
	struct termios serial;
	char buffer[BUFFER_SIZE];

	// not controlling TTY.  
	bool serialPortReady = true;
	int fd = open( SERIAL_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
	if( fd < 0 ) {
		std::cerr << "Unable to open serial" << std::endl;
		perror( SERIAL_DEVICE );
		serialPortReady = false;
	}
	auto result = tcgetattr( fd, & serial );
	if( result < 0 ) {
		std::cerr << "Unable to get serial attributes" << std::endl;
		serialPortReady = false;
	}

	// Set up Serial Configuration
	cfmakeraw(&serial);

	serial.c_cflag |= (CLOCAL | CREAD);
	serial.c_iflag &= ~(IXOFF | IXANY);

	serial.c_cc[VMIN] = 0;
	serial.c_cc[VTIME] = 0;

	cfsetispeed(&serial, B9600);
	cfsetospeed(&serial, B9600);

	tcsetattr(fd, TCSANOW, &serial); // Apply configuration	

	// setup processing variables
	std::vector<cv::Point2f> centers;
	centers.resize(4);
	cv::Mat frame;
	cv::Mat gray;
	cv::Mat thresh;
	cv::Mat displayCopy;
	const unsigned char offscreen[] = { 0xFF, 0xFF };
	unsigned char xy[2];


	// look at the cmameras
	cv::Point2f pt;
	while (true) {
		glutMainLoopEvent();
		inputVideo >> frame;

		// cv to grey
		cv::cvtColor( frame, gray, cv::COLOR_BGR2GRAY); 

		// threshold
		cv::threshold( gray, thresh, 200, 255, cv::THRESH_BINARY);
		std::vector< std::vector< cv::Point> > contours;

		// look for IR lights
		cv::findContours( thresh, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

		// process if we have at least 4 points
		if (contours.size() >= 4) {
			// compute moments to get centers.  
			std::vector< cv::Moments > moments( contours.size() );
			for( size_t i = 0; i < contours.size(); ++i ) {
				moments[i] = cv::moments( contours[i] );
			}

			int current = 0;
			for( size_t i = 0; i < contours.size(); ++i ) {
				// using size cutoff to find 'good' candidates.  Look at other features like circularity, position, etc.
				if( moments[i].m00 > 6 && moments[i].m00 < 250 ) {
					centers[current] =  cv::Point2f( static_cast<float> ( moments[i].m10 / ( moments[i].m00 + 1e-5)), static_cast<float> ( moments[i].m01 / ( moments[i].m00 + 1e-5)) );
					//#ifdef SHOW_CALC
					//std::cout << "center[" << current << "] = " << centers[current] <<  " area: " << moments[i].m00 <<std::endl;
					//#endif
					++current;
					if( current == 4 ) {
						break;
					}
				}
			}

			////// DUMB REORDER | GET RID OF THIS ///////////////////
			cv::Point2f a = centers[0];		
			cv::Point2f b = centers[1];		
			if( centers[3].x > centers[2].x ) {
				centers[0] = centers[2];
				centers[1] = centers[3];
			} else {
				centers[0] = centers[3];
				centers[1] = centers[2];
			}
			if( b.x > a.x ) {
				centers[2] = a;
				centers[3] = b;
			} else {
				centers[2] = b;
				centers[3] = a;
			}
			for( int i=0; i < centers.size(); ++i ) {
				std::cout << "center[" << i << "] = " << centers[i] << std::endl;
			}
			////// DUMB REORDER | GET RID OF THIS ///////////////////

#ifdef SHOW_IMAGE
			// display it
			//cv::cvtColor( thresh, displayCopy, cv::COLOR_GRAY2BGR); 
			displayCopy = frame.clone();
			for( int i=0; i < centers.size(); ++ i ) {
				// draw point on image
				cv::Scalar color( 0,0,255 );
				if( i > 1 ) {
					// Green are the later two
					color = cv::Scalar( 0,255,0 );
					cv::circle( displayCopy, centers[i], 2.0f, color, 2.0f);
				} else {
					// RED
					cv::circle( displayCopy, centers[i], 10.0f, color, 2.0f);
				}
			}
			cv::circle( displayCopy, pt, 5.0f, cv::Scalar(0,255,255), 3.0f);

			cv::imshow("points", displayCopy );
			cv::waitKey(1);
#endif

			// rvec- is the rotation vector
			// tvec- is the translation vector 
			cv::Mat rvec, tvec;
			std::vector< cv::Mat > rvecs, tvecs;	
			auto solveRet = cv::solvePnP(worldPoints, centers, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_AP3P);
			//auto solveRet = cv::solvePnPGeneric(worldPoints, centers, cameraMatrix, distCoeffs, rvecs, tvecs, false, cv::SOLVEPNP_AP3P, rvec, tvec);

			std::cout << "solveRet: " << solveRet << std::endl;
			//if( solveRet ) {
			cv::Mat R;
			cv::Rodrigues(rvec, R); // get rotation matrix R ( 3x3 ) from rotation vector 
			R = R.t(); // inverse
			tvec = -R * tvec; // translation of inverseA == actual camera position



			// compute itersection
			cv::Vec3f Ray0,D;
			Ray0[0] = tvec.at<double>(0);
			Ray0[1] = tvec.at<double>(1);
			Ray0[2] = tvec.at<double>(2);

			D[0] = R.at<double>(0, 2);
			D[1] = R.at<double>(1, 2);
			D[2] = R.at<double>(2, 2);

			float u, v;
			bool hit = intersectRect(Ray0, D, P0, S1, S2, width, height, u, v);
			std::cout << "U: " << u << " V: " << v << " hit: " << hit << std::endl;

#ifdef SHOW_CALC
			std::cout << "rvec: " << rvec << std::endl;	
			std::cout << "tvec: " << tvec << std::endl;
			std::cout << "R: " << R << std::endl;
			std::cout << "inverse tvec: " << tvec << std::endl;

			std::cout << "Ray0: " << Ray0 << std::endl;
			std::cout << "D: " << D << std::endl;
			std::cout << "P0: " << P0 << std::endl;
			std::cout << "S1: " << S1 << std::endl;
			std::cout << "S2: " << S2 << std::endl;
			std::cout << "width: " << width  << std::endl;
			std::cout << "height: " << height << std::endl;
#endif

#ifdef SHOW_3D
			camera3dPts.clear();
			// start of camera axis
			cv::Point3f pt;
			// scale down point for viewing
			pt.x = tvec.at<double>(0) / 1000.0;
			pt.y = tvec.at<double>(1) / 1000.0;
			pt.z = tvec.at<double>(2) / 1000.0;
			camera3dPts.push_back(pt);
			// direction of camera axis
			cv::Point3f dir;
			dir.x = R.at<double>(0, 2);
			dir.y = R.at<double>(1, 2);
			dir.z = R.at<double>(2, 2);
			cv::Point3f pt2 = dir + pt; 
			camera3dPts.push_back(pt2);						

			// set uv on plane 
			hit3dX = u / 1000.0f;
			hit3dY = v / 1000.0f;
			// refresh 
			glutPostRedisplay();
			glutMainLoopEvent();
#endif


			if( hit ) {
#ifdef SHOW_IMAGE
				cv::Point2f pt;
				pt.x = (u/width) * 640.0f;
				pt.y = (v/height) * 480.0f;
				cv::circle( displayCopy, pt, 5.0f, cv::Scalar(0,255,255), 3.0f);
				cv::imshow("points", displayCopy );
				cv::waitKey(1);
#endif
				// from obvservation with delay4Cycles() on arduion
				// X range is 73 to 269 : send 0 through 196
				// Y range is 30 to 250 : send 0 through 220

				xy[0] = (unsigned char)( ( u / width) * 196.0f  );
				xy[1] = (unsigned char)( ( v / height) * 220.0f  );
				if(serialPortReady ) {
					auto ret = write( fd, xy, 2 );
				}
				continue;  // head back up the loop
			} // if( hit ) 
			//} // if( solveRet ) 


		} // if (contours.size() >= 4) 

		// send -1, -1 to arduino
		if(serialPortReady ) {
			auto ret = write( fd, offscreen, 2 );
		}

	}// while(true)
}
