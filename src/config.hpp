#ifndef CONFIG_H
#define CONFIG_H
//Misc
#define spc " "            // Nice way to make spaces.
#define tab "\t"            // Nice way to make spaces.
#define MAXLINE 1024            /*  */
// tracking level
#define ARG_SHOW_TRACKING "-st"                 // Display tracking live.
#define SHOW_TRACKING 1

#define DEFAULT_WINDOW_NAME "Window"
#define TRACKBAR_WINDOW "Trackbars"
#define DEFAULT_IMG_FILENAME "out.jpg"
#define DEFAULT_VID_FILENAME "out.avi"
#define DEFAULT_TXT_FILENAME "matches.txt"

// Options
#define ARG_VID_FILE "-vf"                      // Set video filename.
#define ARG_TXT_FILE "-o"                      // Set output text filename.

#define ARC_ARG_START_FRAME "-sf"
#define ARC_DEFAULT_START_FRAME 0
// Match parameters
#define ARC_ARG_NUM_GOOD_FEATURES_TO_TRACK "-gft"
#define ARC_DEFAULT_NUM_GOOD_FEATURES_TO_TRACK 25
#define ARC_ARG_PATCH_SIZE "-ps"
#define ARC_DEFAULT_PATCH_SIZE 50
#define ARC_ARG_NUM_REGIONS "-nr"
#define ARC_DEFAULT_NUM_REGIONS 25
#define ARC_ARG_STD "-std"
#define ARC_DEFAULT_STD 1.0
#define ARC_ARG_EIG "-eig"
#define ARC_DEFAULT_EIG 0.2
#define ARC_ARG_MAX_DIST "-md"
#define ARC_DEFAULT_MAX_DIST 20
#define ARC_ARG_THETA_DEV "-td"
#define ARC_DEFAULT_THETA_DEV 0.10
#define ARG_BLUR "--blur"                       // Blur the source before matching.
#define ARC_DEFAULT_BLUR 3

// Track parameters
#define ARC_ARG_FEATURES_BEFORE_TRACK "-fbt"         // Get goodFeaturesToTrack on point before tracking.
#define ARC_DEFAULT_FEATURES_BEFORE_TRACK false

// GFT parameters
#define ARC_ARG_GFT_MIN "-gm"            /*  */
#define ARC_DEFAULT_GFT_MIN 5            /*  */
// Main parameters
#define ARG_REFRESH_COUNT "-rc"
#define DEFAULT_REFRESH_COUNT 200                 // # Iterations before rematching.

#endif /* CONFIG_H */
