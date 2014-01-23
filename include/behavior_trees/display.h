#ifndef DISPLAY_H_
#define DISPLAY_H_

#include "behavior_trees/node.h"

#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/freeglut.h>
#include <GL/glu.h>
#include <GL/glx.h>
#include <X11/Intrinsic.h>

#define WINDOWS_NAME "Behavior Trees"
#define FULLSCREEN false
#define SCREEN_POSITION_X 200
#define SCREEN_POSITION_Y 200
#define SCREEN_WIDTH 1200
#define SCREEN_HEIGHT 600 // (9.0/16)
#define NODE_WIDTH 5
#define NODE_HEIGHT 5
#define CURSOR_WIDTH 2 * NODE_WIDTH
#define CURSOR_HEIGHT 2 * NODE_HEIGHT
#define STATUS_WIDTH 1.5 * NODE_WIDTH
#define STATUS_HEIGHT 1.5 * NODE_HEIGHT
#define SPACE_HEIGHT SCREEN_HEIGHT / 10
#define FIRST_LINE SCREEN_HEIGHT
#define CENTER_ROW SCREEN_WIDTH / 2

// functions to manage keystrokes (pressed down)
void keyPressed (unsigned char key, int x, int y);

// functions to manage keystrokes (released )
void keyUp (unsigned char key, int x, int y);

// functions to manage special keystrokes (pressed down)
void keySpecial (int key, int x, int y);

// functions to manage special keystrokes (released)
void keySpecialUp (int key, int x, int y);

// draw the bt node's current status using a frame (color coded)
void draw_status(GLfloat x, GLfloat y, STATE node_status);

// draw the cursor where the user is standing on the bt using a big frame (white)
void draw_cursor(GLfloat x, GLfloat y);

// draw the node itself using a solid square (color coded)
void draw_node(GLfloat x, GLfloat y, NODE_TYPE node_type);

// draw the edge connecting one node to the other
void draw_connector(GLfloat parent_x, GLfloat parent_y, GLfloat x, GLfloat y);

// draws the whole tree, cursor, and statuses, calls swap buffers
void draw_all();

// initialization of the Glut parameters, projection type, screen size
void initialize();

// more general initialization which includes the keyboard listener
void glut_setup(int iArgc, char** cppArgv);

// process called to process events
void glut_process();

// load fonts to display the node names
void init_font(GLuint base, const char* f);

// print a string in OpenGL
void print_string(GLuint base, const char* s);

// load font to be used with OpenGL
void my_init(char* f);

// reshape the contents of the window keeping aspect ratio (disabled / malfunctioning)
void my_reshape(int w, int h);

// draw string in OpenGL at position x, y
void draw_string(GLfloat x, GLfloat y, const char*  word);

#endif
