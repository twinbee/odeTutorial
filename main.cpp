#include <iostream>
#include <math.h>

//Graphics library stuff
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

//our physics library. for installation instructions, see the ODE User's Guide
#include <ode/ode.h>

using namespace std;

#include <unistd.h>     // Header File For sleeping.

//was going to use these for sound, but decided sound unneccessary
// (Simple DirectMedia Layer - Cross platform, open souce library for games)
//#include <SDL/SDL.h>
//#include <SDL/SDL_timer.h>
//#include <SDL/SDL_mixer.h>

//keyboard mnemonic
#define ESCAPE 27

//---- global physics stuff ----
const int numBodies = 6;
const int explosionTime=500;
//minimum of 2, max of 16

void explosion();
void pull();

int explosionTimer = explosionTime; //we use this to keep the balls from stagnating
dReal balRadius = 48.0/numBodies;
dWorldID theWorld; 		//ode world
dBodyID  bodies[numBodies]; 	//ode bodies
dSpaceID theSpace;		// we'll use only one space for collision detection
dGeomID	 bodiesG[numBodies]; 	//we'll need a gemoetry describing each body
dGeomID  theFloor[2];		//we'll also need to detect a collision with the floor/ceiling
dGeomID  walls[4];		//and some walls

dJointGroupID contacts;
dContactGeom contactAry[255]; //well need to store a list of contacts temporarily
//we'll need to keep a group listing of all contacting points for collision handling

//---- global graphics stuff
int window;
float stepBack = 100.0;
const int fps = 10;
const float dt = 1.0/fps;
GLfloat LightAmbient[] =  { 0.0f, 0.0f, 0.0f, 0.0f };
GLfloat LightDiffuse[] =  { 1.0f, 1.0f, 1.0f, 0.0f };
GLfloat LightPosition[] = { 0.0f, 0.0f, stepBack, 0.0f };
GLfloat material_red[]    = { 1.0, 0.0, 0.0, 1 };
GLfloat material_orange[] = { 1.0, 0.7, 0.0, 1 };
GLfloat material_yellow[] = { 0.9, 0.9,   0, 1 };
GLfloat material_green[]  = { 0.0, 0.9,   0, 1 };
GLfloat material_blue[]   = { 0.0,   0, 0.9, 1 };
GLfloat material_violet[] = { 0.5,   0, 0.7, 1 };
GLfloat material_black[]  = { 0.0, 0.0, 0.0, 1 };

GLfloat material_specular[] = { 1, 1, 1, 1 };
GLfloat material_shininess[] = { 100 };

float rotation=0.0;

inline float rad2Deg(float rhs) { return 180.0*rhs/(3.14159257); }

void nearCallback (void * data, dGeomID o1, dGeomID o2)
{
 int i;
 const int MAX_CONTACTS = 1;
 dBodyID b1 = dGeomGetBody(o1);
 dBodyID b2 = dGeomGetBody(o2);
 if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;
                                                                                                     
 dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
 for (i=0; i<MAX_CONTACTS; i++)
 {
  contact[i].surface.mode = dContactBounce;
  contact[i].surface.mu = 5000.0;
  contact[i].surface.bounce = 0.8;
  contact[i].surface.bounce_vel = 0.8;
 }

 if (int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom, sizeof(dContact)))
 {
  dMatrix3 RI;
  dRSetIdentity (RI);
  const dReal ss[3] = {0.02,0.02,0.02};
  for (i=0; i<numc; i++)
  {
   dJointID c = dJointCreateContact (theWorld,contacts, contact+i);
   dJointAttach (c,b1,b2);
  }
 }

}

void explosion()
{ //cause an explosive force that is inverse to the sq of the distnce from center
 // pushing all away from center
 for (int i=0; i<numBodies;i++)
{
 const dReal * p = dBodyGetPosition (bodies[i]);
 float p1[3] = {p[0]-25.0, p[1], p[2]-25.0};
 float d = sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]); //length of vector
 d/=25;
 float a = 40000*(1.0-0.2*d*d); //inverse square law!!! :D
 if (a < 0) a=0;
 p1[0]=p[0]/1.1;
 p1[1]=p[1];
 p1[2]=p[2]/1.1;
 d = sqrt(p1[0]*p1[0]+p1[1]*p1[1]+p1[2]*p1[2]); //length of vector 
 dBodyAddForce(bodies[i], a*p1[0]/d, a*p1[1]/d, a*p1[2]/d);
}
}

void pull()  
{
for (int i=0; i<numBodies; i++)
{
 const dReal * p = dBodyGetPosition (bodies[i]);
 dReal d= sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]);
 dReal p1[3] = { -1000*p[0]/d, 0.0, -1000*p[2]/d };
 dBodyAddForce(bodies[i], p1[0], p1[1], p1[2]);
}
}

void InitPhysics()
{
 theWorld=dWorldCreate(); //in the beginning...
 dWorldSetGravity(theWorld, 0.0, -9.8, 0.0); //and there was gravity

 dWorldSetERP(theWorld, 0.8);  //set the error correction to 80% NECCESSARY
 dWorldSetCFM(theWorld, 1.0E-5); //set the constraint force mixing

 //collision-detection stuff
 theSpace = dSimpleSpaceCreate(theSpace); // Create a space object
 //create geometries within the space
 //the eq for describing a plane is Ax+By+Cz=D
 //the syntax for dCreatePlane is dCreatePlane(dSpaceid ___, dReal A, dReal B, dReal C,dReal D)
 theFloor[0] = dCreatePlane(theSpace, 0.0, 1.0, 0, -50.0);
 theFloor[1] = dCreatePlane(theSpace, 0.0, -1.0, 0, -50.0);
 walls[0] = dCreatePlane(theSpace,1.0,0.0,0.0,-50.0); 
 walls[1] = dCreatePlane(theSpace,-1.0,0.0,0.0,-50.0);
 walls[2] = dCreatePlane(theSpace,0.0,0.0,1.0,-50.0);
 walls[3] = dCreatePlane(theSpace,0.0,0.0,-1.0,-50.0);
 
 contacts=dJointGroupCreate(0); //create an empty group of contacts for sotring contact joints

for (int i=0; i<numBodies;i++)
{
 bodies[i]=dBodyCreate(theWorld); //create a body in this world
 dBodySetPosition(bodies[i], (i+1)*balRadius*2.0-numBodies*balRadius-.5*balRadius,
  stepBack/3.0, 0.0);
 //put it at the origin, and located in some place along X based upon index
//set the position to some place on a line that bisects the center of the bounding cube
 dBodySetAngularVel (bodies[i], rand()%5/8.0, rand()%5/8.0, rand()%5/8.0);
//randomize its spin so that collisions will be more interesting

/* //the following are not used, but have been provided to demonstrate syntax
 dBodySetLinearVel(bodies[i], 0.0, 0.0, 0.0 );
 dBodyAddForce(bodies[i], 0.0, 0.0, 0.0);
*/
 bodiesG[i] = dCreateSphere(theSpace, balRadius);
 //we want each body's associated geom to be that of a sphere of radius 1
 dGeomSetBody(bodiesG[i], bodies[i]);
}
}

void phyTurn() //gets called each frame
{
for (int i = 1; i<numBodies; i++) { dSpaceCollide(theSpace, contacts, nearCallback); }
//do collision detection for all bodies using the function nearCallback when there is a
// likely collision
dWorldStep(theWorld, dt/2.0); // step the world
dJointGroupEmpty(contacts); //empty the list of joint contacts

for (int i=0; i< numBodies; i++)
{
 glPushMatrix();
  const dReal * pos = dBodyGetPosition(bodies[i]);
  const dReal * rot = dBodyGetRotation(bodies[i]);
  GLfloat matrix[16];
  matrix[0]=rot[0];
  matrix[1]=rot[4];
  matrix[2]=rot[8];
  matrix[3]=0;
  matrix[4]=rot[1];
  matrix[5]=rot[5];
  matrix[6]=rot[9];
  matrix[7]=0;
  matrix[8]=rot[2];
  matrix[9]=rot[6];
  matrix[10]=rot[10];
  matrix[11]=0;
  matrix[12]=pos[0];
  matrix[13]=pos[1];
  matrix[14]=pos[2];
  matrix[15]=1;
  glMultMatrixf (matrix);

switch (i%6)
{ //color the ball based upon its index
 case 0:  glMaterialfv(GL_FRONT, GL_DIFFUSE, material_red); break;
 case 1:  glMaterialfv(GL_FRONT, GL_DIFFUSE, material_orange); break;
 case 2:  glMaterialfv(GL_FRONT, GL_DIFFUSE, material_yellow); break;
 case 3:  glMaterialfv(GL_FRONT, GL_DIFFUSE, material_green); break;
 case 4:  glMaterialfv(GL_FRONT, GL_DIFFUSE, material_blue); break;
 case 5:  glMaterialfv(GL_FRONT, GL_DIFFUSE, material_violet); break;
}
 glutSolidSphere(balRadius, 20, 10);
 glPopMatrix();
}
}

// --- EVERYTHING BEYOND THIS POINT HAS NOTHING TO DO WITH PHYSICS -- ALL GRAPHICS RELATED STUFF -- //

void InitGL(int Width, int Height)
{
 glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
 glClearDepth(1.0);
 glDepthFunc(GL_LESS);
 glEnable(GL_DEPTH_TEST);
 glShadeModel(GL_FLAT);

 glMatrixMode(GL_PROJECTION);
 glLoadIdentity();

 gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.0,1000.0);

 glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);  // add lighting. (ambient)
 glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);  // add lighting. (diffuse).
 glLightfv(GL_LIGHT1, GL_POSITION,LightPosition); // set light position.
 glEnable(GL_LIGHT1);                             // turn light 1 on.
 glEnable(GL_LIGHTING);

 glMaterialfv(GL_FRONT, GL_SPECULAR, material_specular);
 glMaterialfv(GL_FRONT, GL_SHININESS, material_shininess);

 glMatrixMode(GL_MODELVIEW);
 glColor3f(0.0,0.0,0.0);

 glLineWidth(3.0); //so that we can see the bounding cube
}

void ReSizeGLScene(int Width, int Height)
{
  if (Height==0) Height=1;

  glViewport(0, 0, Width, Height);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.1f,500.0f);
  glMatrixMode(GL_MODELVIEW);
}

void DrawGLScene()
{
 if (explosionTimer) {explosionTimer--; glClearColor(1.0, 1.0, 1.0, 0.0); }
 else //this is to keep the balls moving and interesting
 {
  glClearColor(1.0, 0.0, 0.0, 0.0);
  pull();
  explosionTimer=explosionTime;
 }

 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 glLoadIdentity();
 glTranslatef(0.0, 0.0, -stepBack*2); //draw scene into the screen
 glRotatef(rotation, 0.0, 1.0, 0.0); //rotate whole scene
 if (rotation >= 360.0) rotation = 0.0; else rotation += 0.1;

 glMaterialfv(GL_FRONT, GL_DIFFUSE, material_black);
 glutWireCube(100);
 phyTurn();
 glutSwapBuffers();
}

void keyPressed(unsigned char key, int x, int y) 
{
 usleep(10); //avoid thrashing

 if (key == 27) 
 { 
  glutDestroyWindow(window); 
  exit(0);                   
 }

else if (key==13) pull();
}

int main(int argc, char **argv) 
{  
 srand(time(0)); //good rndom start
  
  glutInit(&argc, argv);  
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);  
  glutInitWindowSize(640, 480);  
  glutInitWindowPosition(0, 0);  
  window = glutCreateWindow("My Very First Physics/ODE tutorial");  

  glutDisplayFunc(&DrawGLScene);  
  //glutFullScreen();
  glutIdleFunc(&DrawGLScene);
  glutReshapeFunc(&ReSizeGLScene);
  glutKeyboardFunc(&keyPressed);

  InitGL(640, 480);
  InitPhysics();
  
  glutMainLoop();  

  return 1;
}

