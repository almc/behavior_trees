#include "behavior_trees/keystroke.h"
#include "behavior_trees/glutkey.h"
#include "behavior_trees/navigation.h"

// keystrokes
bool run = true;
bool L_keypressed = false;
bool L_keydown = false;
bool R_keypressed = false;
bool R_keydown = false;
bool U_keypressed = false;
bool U_keydown = false;
bool D_keypressed = false;
bool D_keydown = false;
bool ENTER_keypressed = false;
bool ENTER_keydown = false;
bool BACK_keypressed = false;
bool BACK_keydown = false;
bool PAUSE_keypressed = false;
bool PAUSE_keydown = false;
bool ESC_keypressed = false;
bool ESC_keydown = false;
bool S1_keypressed = false;
bool S1_keydown = false;
bool S2_keypressed = false;
bool S2_keydown = false;
bool S3_keypressed = false;
bool S3_keydown = false;

extern NodeRoot root;
extern bool* keyStates;

int get_keypressed()
{
	// check for left
	bool L_tmp = keyStates['a'];
	if( !L_keydown && L_tmp )    // key wasn't down previously, but now is down
		L_keypressed = true;     // so it was just pressed
	else                         // otherwise, it was not just pressed
		L_keypressed = false;
	L_keydown = L_tmp;

	// check for right
	bool R_tmp = keyStates['e'];
	if( !R_keydown && R_tmp )
		R_keypressed = true;
	else
		R_keypressed = false;
	R_keydown = R_tmp;

	// check for up
	bool U_tmp = keyStates['l'];
	if( !U_keydown && U_tmp )
		U_keypressed = true;
	else
		U_keypressed = false;
	U_keydown = U_tmp;

	// check for down
	bool D_tmp = keyStates['r'];
	if( !D_keydown && D_tmp )
		D_keypressed = true;
	else
		D_keypressed = false;
	D_keydown = D_tmp;

	// check for enter
	bool ENTER_tmp = keyStates[GLUT_KEY_ENTER];
	if( !ENTER_keydown && ENTER_tmp )
		ENTER_keypressed = true;
	else
		ENTER_keypressed = false;
	ENTER_keydown = ENTER_tmp;

	// check for backspace
	bool BACK_tmp = keyStates[GLUT_KEY_BACKSPACE];
	if( !BACK_keydown && BACK_tmp )
		BACK_keypressed = true;
	else
		BACK_keypressed = false;
	BACK_keydown = BACK_tmp;

	// check for spacebar
	bool PAUSE_tmp = keyStates[GLUT_KEY_SPACE];
	if( !PAUSE_keydown && PAUSE_tmp )
		PAUSE_keypressed = true;
	else
		PAUSE_keypressed = false;
	PAUSE_keydown = PAUSE_tmp;

	// check for exit
	bool ESC_tmp = keyStates[GLUT_KEY_ESC];
	if( !ESC_keydown && ESC_tmp )
		ESC_keypressed = true;
	else
		ESC_keypressed = false;
	ESC_keydown = ESC_tmp;

	// check for 1 (running)
	bool S1_tmp = keyStates[GLUT_KEY_1];
	if( !S1_keydown && S1_tmp )
		S1_keypressed = true;
	else
		S1_keypressed = false;
	S1_keydown = S1_tmp;

	// check for 2 (success)
	bool S2_tmp = keyStates[GLUT_KEY_2];
	if( !S2_keydown && S2_tmp )
		S2_keypressed = true;
	else
		S2_keypressed = false;
	S2_keydown = S2_tmp;

	// check for 3 (fail)
	bool S3_tmp = keyStates[GLUT_KEY_3];
	if( !S3_keydown && S3_tmp )
		S3_keypressed = true;
	else
		S3_keypressed = false;
	S3_keydown = S3_tmp;

	return 0;
}

int process_keypressed()
{
#ifdef DEBUG_MAIN
	if (L_keypressed)
		std::cout << "Left button pressed" << std::endl;
	if (R_keypressed)
		std::cout << "Right button pressed" << std::endl;
	if (U_keypressed)
		std::cout << "Up button pressed" << std::endl;
	if (D_keypressed)
		std::cout << "Down button pressed" << std::endl;
	if (ENTER_keypressed)
		std::cout << "Enter button pressed" << std::endl;
	if (BACK_keypressed)
		std::cout << "Backspace button pressed" << std::endl;
	if (PAUSE_keypressed)
		std::cout << "Spacebar button pressed" << std::endl;
	if (ESC_keypressed)
		std::cout << "Escape button pressed" << std::endl;
#endif

	if (L_keypressed)
		navigate_left();

	if (R_keypressed)
		navigate_right();

	if (U_keypressed)
		navigate_up();

	if (D_keypressed)
		navigate_down();

	if (ENTER_keypressed)
		print_node_info();

	if (BACK_keypressed)
		reset_overwritten();

	if (PAUSE_keypressed)
	{}

	if (ESC_keypressed)
		run = false;

	// if (S1_keypressed)
	// 	set_node_state(FAILURE);

	// if (S2_keypressed)
	// 	set_node_state(SUCCESS);

	// if (S3_keypressed)
	// 	set_node_state(RUNNING);

	// if (S1_keypressed || S2_keypressed || S3_keypressed || BACK_keypressed)
	// 	root.execute();

	return 0;
}
