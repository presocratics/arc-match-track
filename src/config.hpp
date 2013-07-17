#ifndef CONFIG_H
#define CONFIG_H
//Misc
#define spc " "            // Nice way to make spaces.
#define tab "\t"            // Nice way to make spaces.

//Debug
#define ARG_DEBUG_MODE "--debug"                // Show program debugging info.
#define ARG_VERBOSE "-v"                        // Show non-looping text info about matching.
#define ARG_VERY_VERBOSE "-vv"                  // Adds some looping info.
#define ARG_VERY_VERY_VERBOSE "-vvv"            // Everything else.

#define NO_DEBUG 0                              // Do not show debug info.
#define DEBUG 1                                 // Show debug info.
#define NOT_VERBOSE 0                           // Quiet.
#define VERBOSE 1
#define VERY_VERBOSE 2
#define VERY_VERY_VERBOSE 3

// show match level
#define NO_SHOW_MATCHES 0                       // Do not show match results.
#define ARG_SHOW_MATCHES "-sm"                  // Show results of matching.
#define SHOW_MATCHES 1

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

// Match parameters
#define ARC_ARG_RADIUS "-r"
#define ARC_DEFAULT_RADIUS 1
#define ARC_ARG_PATCH_SIZE "-ps"
#define ARC_DEFAULT_PATCH_SIZE 50
#define ARC_ARG_NUM_REGIONS "-nr"
#define ARC_DEFAULT_NUM_REGIONS 25
#define ARC_ARG_SLOPE_DEV "-sd"
#define ARC_DEFAULT_SLOPE_DEV 10
#define ARG_MATCH_RATIO "--match-ratio"         // Set threshold for ratio_test.
#define DEFAULT_MATCH_RATIO 0.70                // 
#define ARG_MIN_MATCH_POINTS "--min-match-points" // Set minimum points to declare successful match. For RANSAC only.
#define DEFAULT_MIN_MATCH_POINTS 7            
#define ARG_RATIO_OFF "--no-ratio"              // Do not perform ratio_test.
#define ARG_SYMTEST_OFF "--no-sym"              // Do not perform symmetry_test.
#define ARG_RANSAC_OFF "--no-ransac"            // Do not perform ransac_test.
#define ARG_RANSAC_ON "--ransac"            // Perform ransac_test.
#define ARG_BLUR "--blur"                       // Blur the source before matching.
#define ARG_NO_BLUR "--no-blur"                       // Don't blur the source before matching.
#define ARC_DEFAULT_BLUR true
#define ARC_DEFAULT_RANSAC false

#define UP 1                                    // match to scene above object
#define DOWN 2                                  // match to scene below object

// Main parameters
#define ARG_REFRESH_COUNT "-rc"
#define DEFAULT_REFRESH_COUNT 200                 // # Iterations before rematching.

#endif /* CONFIG_H */
