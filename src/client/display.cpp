#include "behavior_trees/display.h"

static GLuint font_base;
extern NodeRoot root;
bool* keyStates = new bool[256];
bool* keySpecialStates = new bool[246];

// functions to manage keystrokes (pressed down)
void keyPressed (unsigned char key, int x, int y)
{
	std::cout << "Key Value: " << key << std::endl;
	keyStates[key] = true;
}

// functions to manage keystrokes (released )
void keyUp (unsigned char key, int x, int y)
{
	keyStates[key] = false;
}

// functions to manage special keystrokes (pressed down)
void keySpecial (int key, int x, int y)
{
	std::cout << "Special Key Value: " << key << std::endl;
	keySpecialStates[key] = true;
}

// functions to manage special keystrokes (released)
void keySpecialUp (int key, int x, int y)
{
	keySpecialStates[key] = false;
}

// draw the bt node's current status using a frame (color coded)
void draw_status(GLfloat x, GLfloat y, STATE node_status)
{
	switch (node_status)
	{
	case SUCCESS:    glColor3f(0.0, 1.0, 0.0); break;
	case FAILURE:    glColor3f(1.0, 0.0, 0.0); break;
	case RUNNING:    glColor3f(1.0, 1.0, 0.0); break;
	case NODE_ERROR: glColor3f(1.0, 1.0, 1.0); break;
	default: std::cout << "Error Node Status Color Selection" << std::cout;
	}
	glLineWidth(3.0);
	glBegin(GL_LINE_LOOP);
	glVertex3f((GLfloat) (x + STATUS_WIDTH), (GLfloat) (y - STATUS_HEIGHT), (GLfloat) 0.0);
	glVertex3f((GLfloat) (x + STATUS_WIDTH), (GLfloat) (y + STATUS_HEIGHT), (GLfloat) 0.0);
	glVertex3f((GLfloat) (x - STATUS_WIDTH), (GLfloat) (y + STATUS_HEIGHT), (GLfloat) 0.0);
	glVertex3f((GLfloat) (x - STATUS_WIDTH), (GLfloat) (y - STATUS_HEIGHT), (GLfloat) 0.0);
	glEnd();
	glLineWidth(1.0);
}

// draw the cursor where the user is standing on the bt using a big frame (white)
void draw_cursor(GLfloat x, GLfloat y)
{
	glLineWidth(3.0);
	glBegin(GL_LINE_LOOP);
	glColor3f(1.0, 1.0, 1.0);
	glVertex3f((GLfloat) (x + CURSOR_WIDTH), (GLfloat) (y - CURSOR_HEIGHT), (GLfloat) 0.0);
	glVertex3f((GLfloat) (x + CURSOR_WIDTH), (GLfloat) (y + CURSOR_HEIGHT), (GLfloat) 0.0);
	glVertex3f((GLfloat) (x - CURSOR_WIDTH), (GLfloat) (y + CURSOR_HEIGHT), (GLfloat) 0.0);
	glVertex3f((GLfloat) (x - CURSOR_WIDTH), (GLfloat) (y - CURSOR_HEIGHT), (GLfloat) 0.0);
	glEnd();
	glLineWidth(1.0);
}

// draw the node itself using a solid square (color coded)
void draw_node(GLfloat x, GLfloat y, NODE_TYPE node_type)
{
	switch (node_type)
	{
	case SELECTOR:      glColor3f(1.0, 0.0, 0.0); break;
	case SELECTOR_STAR: glColor3f(0.6, 0.0, 0.0); break;
	case SEQUENCE:      glColor3f(0.0, 0.0, 1.0); break;
	case SEQUENCE_STAR: glColor3f(0.0, 0.0, 0.6); break;
	case PARALLEL:      glColor3f(0.0, 1.0, 1.0); break;
	case DECORATOR:     glColor3f(1.0, 0.0, 1.0); break;
	case ACTION:        glColor3f(0.0, 1.0, 0.0); break;
	case CONDITION:     glColor3f(1.0, 1.0, 0.0); break;
	case ROOT:          glColor3f(1.0, 1.0, 1.0); break;
	default: std::cout << "Invalid Node Selected (color)" << std::endl;
	}
	glBegin(GL_QUADS);
	glVertex3f((GLfloat) (x + NODE_WIDTH), (GLfloat) (y - NODE_HEIGHT), (GLfloat) 0.0);
	glVertex3f((GLfloat) (x + NODE_WIDTH), (GLfloat) (y + NODE_HEIGHT), (GLfloat) 0.0);
	glVertex3f((GLfloat) (x - NODE_WIDTH), (GLfloat) (y + NODE_HEIGHT), (GLfloat) 0.0);
	glVertex3f((GLfloat) (x - NODE_WIDTH), (GLfloat) (y - NODE_HEIGHT), (GLfloat) 0.0);
	glEnd();
}

// draw the edge connecting one node to the other
void draw_connector(GLfloat parent_x, GLfloat parent_y, GLfloat x, GLfloat y)
{
	glBegin(GL_LINES);
	glColor3f(1.0, 1.0, 1.0);
	glVertex2f(parent_x, parent_y);
	glVertex2f(x, y);
	glEnd();
}

// draws the whole tree, cursor, and statuses, calls swap buffers
void draw_all()
{
	std::cout << "Drawing Everything" << std::endl;
	glClear(GL_COLOR_BUFFER_BIT);
	root.draw_subtree((GLfloat) (CENTER_ROW), (GLfloat) (FIRST_LINE), 1,
	                  (GLfloat) (SCREEN_WIDTH));
	glutSwapBuffers();
}

// initialization of the Glut parameters, projection type, screen size
void initialize()
{
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, SCREEN_WIDTH, 0, SCREEN_HEIGHT, -1.0, 1.0);
}

// more general initialization which includes the keyboard listener
void glut_setup(int iArgc, char** cppArgv)
{
	std::string font_name_str = "fixed";
	char* font_name = (char*)font_name_str.c_str();

	glutInit(&iArgc, cppArgv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize((int) (SCREEN_WIDTH), (int) (SCREEN_HEIGHT));
	glutInitWindowPosition(SCREEN_POSITION_X, SCREEN_POSITION_Y);
	glutCreateWindow(WINDOWS_NAME);

	initialize();
	my_init(font_name);

	if (FULLSCREEN)
		glutFullScreen();
	glutDisplayFunc(draw_all);	// function called to display
	// glutReshapeFunc(my_reshape);

	glutKeyboardFunc(keyPressed);
	glutKeyboardUpFunc(keyUp);
	glutSpecialFunc(keySpecial);
	glutSpecialUpFunc(keySpecialUp);
}

// process called to process events
void glut_process()
{
	glutMainLoopEvent();
}

// load fonts to display the node names
void init_font(GLuint base, const char* f)
{
	Display* display;
	XFontStruct* font_info;
	int first;
	int last;

	display = XOpenDisplay(0);
	if (display == 0)
	{
		fprintf(stderr, "XOpenDisplay() failed.  Exiting.\n");
		exit(-1);
	}
	else
	{
		font_info = XLoadQueryFont(display, f);
		if (!font_info)
		{
			fprintf(stderr, "XLoadQueryFont() failed - Exiting.\n");
			exit(-1);
		}
		else
		{
			first = font_info->min_char_or_byte2;
			last  = font_info->max_char_or_byte2;
			glXUseXFont(font_info->fid, first, last-first+1, base+first);
		}
		XCloseDisplay(display);
		display = 0;
	}
}

// print a string in OpenGL
void print_string(GLuint base, const char* s)
{
	if (!glIsList(font_base))
	{
		fprintf(stderr, "print_string(): Bad display list. - Exiting.\n");
		exit (-1);
	}
	else if (s && strlen(s))
	{
		glPushAttrib(GL_LIST_BIT);
		glListBase(base);
		glCallLists(strlen(s), GL_UNSIGNED_BYTE, (GLubyte *)s);
		glPopAttrib();
	}
}

// load font to be used with OpenGL
void my_init(char* f)
{
	font_base = glGenLists(256);
	if (!glIsList(font_base))
	{
		fprintf(stderr, "my_init(): Out of display lists. - Exiting.\n");
		exit (-1);
	}
	else
	{
		init_font(font_base, f);
	}
}

// reshape the contents of the window keeping aspect ratio (disabled / malfunctioning)
void my_reshape(int w, int h)
{
	GLdouble size;
	GLdouble aspect;

	// use the whole window
	glViewport(0, 0, w, h);

	// 2-D orthographic drawing
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	size = (GLdouble)((w >= h) ? w : h) / 2.0;
	if (w <= h)
	{
		aspect = (GLdouble)h/(GLdouble)w;
		glOrtho(-size, size, -size*aspect, size*aspect, -100000.0, 100000.0);
	}
	else
	{
		aspect = (GLdouble)w/(GLdouble)h;
		glOrtho(-size*aspect, size*aspect, -size, size, -100000.0, 100000.0);
	}

	// make the world and window coordinates coincide so that 1.0 in
	// model space equals one pixel in window space.
	glScaled(aspect, aspect, 1.0);

	// determine where to draw things
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

// draw string in OpenGL at position x, y
void draw_string(GLfloat x, GLfloat y, const char* word)
{
	glColor4f(1.0, 1.0, 1.0, 0.0);
	glRasterPos2f(x, y);
	print_string(font_base, word);
}
