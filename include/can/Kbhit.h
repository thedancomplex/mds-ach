/*! Keyboard hit detect function
* basic function used to detect a keyboard hit
* \brief declaration to detect keyboard hit
* \file Kbhit.h
* \ingroup CommonHelperFunctions
*/

#ifdef __WIN32__

#include <conio.h>

#endif

#ifdef __LINUX__

#include <sys/select.h>

int kbhit(void);

#endif
