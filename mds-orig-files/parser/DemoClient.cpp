/*! Utility to accept command-line input to a DOF
* Command line utitlity to move a DOF to a desired poisition
*
*<pre>
*	Usage: NRLMove
*   Command-line input : -n [address] -d [destination] -a [accel] -t [timeout]
*   Example: -n 0x0030 -d 10000 -a 500 -t 10
*	Timeout of zero means no timeout
* </pre>
*
* \author Magda Bugajska
*/

#include <cstdio>
#include <time.h>
#include "strcmp.h"
#include "ParseCMDLine.h"
#include <iostream>
#include <sstream>
#include <list>
#include <vector>
#include <cstring>

#ifdef WIN32_COMPILE
// For Sleep
#include <windows.h>
#endif

#include "comm.h"

using namespace std;

int cyclic_contact_server(char *name, int port)
{
  int ch,done;

  done = 0;
  do {
    if ((ch = init_internet_client(name,port)) < 0)
#ifdef WIN32_COMPILE
    	Sleep(1000);
#else
	sleep(1);
#endif
    else
      done = 1;
  } while (!done);
  return(ch);
}

int receiveMessage(int ch, char * buf, int n) {
	int readresult;

    if ((readresult=read_comm(ch,buf,n)) > 0) {

                printf("\nReceived Reply \n");

		for (int i = 0; i < readresult; i++) {
			if (buf[i] == '\0') printf("\n");
			else printf("%c",buf[i]);
		}

		printf("\nTotal size = %d", readresult);
		fflush(stdout);
    }

    return readresult;
}

int sendMessage(int ch, char * buf, int n) {
	int writeresult;

	   writeresult = robust_write_comm(ch,buf,n);
	   return writeresult;
}
//! Main Function for NRLMove
/*!
*	Usage: MotionDemoClient
*/
int main(void)
{
	clparam cmdarg;
	char iCommand[20];

	strcpy(cmdarg.strSwitch,"-c");
	cmdarg.target=iCommand;
	cmdarg.type=argString;
	cmdarg.bOptional=0;

	char buffer[250000];
	char hostname[256];
	int port;
	int result = 0;
	int ch;

	cout << "\nPlease enter name of server's machine (e.g localhost): ";
	cin >> hostname;

	cout << "\nPlease enter port of the server machine (eg. 5100): ";
	cin >> port;

	init_comm_package();
	ch = cyclic_contact_server(hostname,port);

	nonblocking_read(ch);

	string line;
	int done = 0;
	vector<int> commands;

	cout << endl << "Input:  ";

	while(!done)
	{
		if (result < 0 || receiveMessage(ch, buffer, sizeof(buffer)) < 0) {
			ch = cyclic_contact_server(hostname,port);
			result = 0;
		}

		if (!cin.eof()) {

			getline(cin, line);

//			cout << endl << "Line [" << line << "]";

			string token; // Have a buffer string
			stringstream st(stringstream::in | stringstream::out);
			st << line ;

			char * tokens[80]; // Create vector to hold our words
			int tokensNum = 0;

//			printf("\nTokenizing input ...");

			while (st >> token) {
//				printf("\nToken[%d]:  %s", token.size()+1, token.c_str());

				tokens[tokensNum] = new char[token.size()+1];
				strcpy(tokens[tokensNum], token.c_str());

				if (!token.compare("-c"))
				{
//					printf("\nPushing %d", tokensNum);
					commands.push_back(tokensNum);
				}

				tokensNum++;
			}
			if (commands.size() > 0) {
//				printf("\nPushing %d", tokensNum); fflush(stdout);
				commands.push_back(tokensNum);
			}

//			printf("\nTotal tokens: %d", tokensNum);

//			printf("\nCommands %d", commands.size());
			for (int i = 0; i < ((int)commands.size() - 1); i++) {

//				printf("\n Looping [%d] with %d %d", i, commands[i], commands[i+1]);
				tokensNum = commands[i+1] - commands[i];

				if (tokensNum > 0) {
//					printf("\n Processing new input ...");

					if(ParseCommandLine(&tokens[commands[i]],&cmdarg,1,tokensNum))
					{

					}
					else {

						printf("\n Processing %s command ...", iCommand);

						string command = iCommand;
						if (!command.compare("exit")) {
							sprintf(buffer, "-c exit");
							done = 1;
							sendMessage(ch, buffer, strlen(buffer)+1);
						}
						else {
							sprintf(buffer, "%s", line.c_str());
							sendMessage(ch, buffer, line.size()+1);
						}
					}

					if (!done) cout << endl << "Input:  ";

					line.clear();
				}
			}
			commands.clear();

		}
	}

	close_channel_comm(ch);

	return 0;
}


