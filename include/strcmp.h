/*! String Compare defines
* header to define _XPLATFORMSTRINGCOMPARE to abstract specific string compare functions as they have different names
* under windows and linux
* \brief Cross platofrm string compare defines
* \file strcmp.h
* \ingroup CommonHelperFunctions
*/

#ifdef __LINUX__
//! cross platform string compare
#define _XPLATFORMSTRINGCOMPARE(x,y)	strcasecmp(x,y)
#endif

#ifdef __WIN32__
//! cross platform string compare
#define _XPLATFORMSTRINGCOMPARE(x,y)	stricmp(x,y)
#endif
