// XitomeMCBCommandParser.h

#ifndef _XITOMEMCBCOMMANDPARSER_H_
#define _XITOMEMCBCOMMANDPARSER_H_

#define MAXRESPONSESTRING	80

int ParseCommand(char *cmd,char *data);
int ParseResponse(char *str,char *data, unsigned long timestamp);

#endif
