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

#define NO_DEBUG 0
#define DEBUG 1
#define NOT_VERBOSE 0
#define VERBOSE 1
#define VERY_VERBOSE 2
#define VERY_VERY_VERBOSE 3

// show match level
#define NO_SHOW_MATCHES 0
#define ARG_SHOW_MATCHES "-sm"                  // Show results of matching.
#define SHOW_MATCHES 1

// tracking level
#define ARG_SHOW_TRACKING "-st"               // Display tracking live.
#define SHOW_TRACKING 1

#define DEFAULT_WINDOW_NAME "Window"
#define DEFAULT_IMG_FILENAME "out.jpg"
#define DEFAULT_VID_FILENAME "out.avi"

// Options
#define ARG_VID_FILE "-vf"

// Match parameters
#define ARG_MATCH_RATIO "--match-ratio"            // 
#define DEFAULT_MATCH_RATIO 0.87             // 
#define ARG_MIN_MATCH_POINTS "--min-match-points"            // 
#define DEFAULT_MIN_MATCH_POINTS 7            // 

#define UP 1                                    // match to scene above object
#define DOWN 2                                  // match to scene below object

// Main parameters
#define ARG_REFRESH_COUNT "-rc"            // 
#define DEFAULT_REFRESH_COUNT 5                 // # Iterations before rematching.
// KLT Parameters
#define MAX_COUNT 10

#endif /* CONFIG_H */
