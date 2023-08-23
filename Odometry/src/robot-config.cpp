#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
/*vex-vision-config:begin*/
signature Vision1__BLUE_BALL = signature (1, -3641, -2723, -3182, 8149, 15655, 11902, 3, 0);
signature Vision1__RED_BALL = signature (2, 6041, 8139, 7090, -315, 499, 92, 3, 0);
signature Vision1__SIG_3 = signature (3, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision1__SIG_4 = signature (4, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision1__SIG_5 = signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision1__SIG_6 = signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision1__SIG_7 = signature (7, 0, 0, 0, 0, 0, 0, 3, 0);
vision Vision1 = vision (PORT1, 41, Vision1__BLUE_BALL, Vision1__RED_BALL, Vision1__SIG_3, Vision1__SIG_4, Vision1__SIG_5, Vision1__SIG_6, Vision1__SIG_7);
/*vex-vision-config:end*/
inertial Inertial = inertial(PORT12);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}