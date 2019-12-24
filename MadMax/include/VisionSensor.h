/*vex-vision-config:begin*/
#include "v5.h"
#include "v5_vcs.h"
vex::vision::signature ORANGE_CUBE = vex::vision::signature (1, 9363, 10579, 9971, -2859, -2439, -2649, 5.6, 0);
vex::vision::signature GREEN_CUBE = vex::vision::signature (2, -7829, -2783, -5306, -6241, -2047, -4144, 1.4, 0);
vex::vision::signature PURPLE_CUBE = vex::vision::signature (3, 1585, 2913, 2249, 8589, 10735, 9662, 4.2, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision Vision1 = vex::vision (vex::PORT1, 50, ORANGE_CUBE, GREEN_CUBE, PURPLE_CUBE, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/