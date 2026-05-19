#ifndef DMAPF_INCLUDE_COLOR_H_
#define DMAPF_INCLUDE_COLOR_H_

// Terminal color codes for UBUNTU/LINUX and MacOS
#ifdef _WIN32
#define RESET     ""
#define BOLD      ""
#define BLACK     ""
#define RED       ""
#define GREEN     ""
#define YELLOW    ""
#define BLUE      ""
#define MAGENTA   ""
#define CYAN      ""
#define WHITE     ""
#else
#define RESET     "\033[0m"
#define BOLD      "\033[1m"
#define BLACK     "\033[30m"    /* Black */
#define RED       "\033[31m"    /* Red */
#define GREEN     "\033[32m"    /* Green */
#define YELLOW    "\033[33m"    /* Yellow */
#define BLUE      "\033[34m"    /* Blue */
#define MAGENTA   "\033[35m"    /* Magenta */
#define CYAN      "\033[36m"    /* Cyan */
#define WHITE     "\033[37m"    /* White */
#endif

#endif
