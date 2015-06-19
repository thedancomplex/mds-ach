/*! Parses command line
* helper routine to parse command line
* \brief parses command line
* \file ParseCMDLine.h
* \ingroup CommonHelperFunctions
*/
#pragma once

//! Type of command line arguments
/*! in the clparamstruct, each parameter type is defined here
*/
enum cmdArg{
	//! hex decimal value
	argHex,			
	//! floating point value
	argFloat,
	//! string argument
	argString,
	//! integer value
	argInteger,
	//! a binary switch (if it exists, switch is set)
	argSwitch
};

//! struct to define each parameter element
/*
*/
struct clparamstruct {
	//! switch to look for (ie "-v", or "-blah")
	char strSwitch[10];	
	//! pointer to the target for the argument
	void *target;		
	//! type of argument
	cmdArg type;		
	//! if it's an optional argument
	char bOptional;		
};

//! Type for structure to define parameters element
typedef  struct clparamstruct clparam;

int ParseCommandLine(char **argv,clparam * params,int iNumParameters, int argc);
int ParseCommandLineSingle(char **argv, int argc, char * strSwitch, void * target, cmdArg type, char bOptional);
