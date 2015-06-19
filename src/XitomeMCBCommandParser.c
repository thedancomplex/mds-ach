#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "XitomeMCBCommandParser.h"
#include "XitomeMCBCommands.h"
#include "CANAddresses.h"

int ParseResponse(char *str,char *data, unsigned long timestamp)
{
	unsigned short int usitmp;
	short int sitmp;
	int itmp;
	float ftmp;

	sprintf(str,"\n%ul\t",(unsigned int)timestamp);

	// if(stat<0)
		// sprintf("[%d]",stat);
		
	switch(data[7])
	{
		case RESPONSE_STATE:
			sprintf(str,"%s0x%02X%02X\t",str,data[5],data[4]);
			switch(data[6])
			{
				case ARGUMENT_STATE_MAX_ENCODER_FREQUENCY:
					sprintf(str,"%s\tMax Encoder Frequency\t%c",str,data[0]);
					break;
				case ARGUMENT_STATE_TRAJECTORY_BUFFER_SIZE:
					sprintf(str,"%s\tTrajectory Buffer Size\t%c",str,data[0]);
					break;
				case ARGUMENT_STATE_AUTO_UPDATE_VALUE:
					memcpy(&usitmp,data,2);
					sprintf(str,"%s\tAutoUpdate Value\t%d",str,usitmp);
					break;
				case ARGUMENT_STATE_TRAJECTORY_PERIOD:
					memcpy(&usitmp,data,2);
					sprintf(str,"%s\tTrajectory Period\t%d",str,usitmp);
					break;			
				case ARGUMENT_STATE_MAX_PWM_DC:
					itmp=0;
					memcpy(&itmp,data,2);
					sprintf(str,"%s\tMax DC\t%d",str,itmp);
					break;
			
				case ARGUMENT_STATE_LOWERLIMIT:
					memcpy(&itmp,data,4);
					sprintf(str,"%s\tLower Limit\t%d",str,itmp);
					break;
				case ARGUMENT_STATE_UPPERLIMIT:
					memcpy(&itmp,data,4);
					sprintf(str,"%s\tUpper Limit\t%d",str,itmp);
					break;
				case ARGUMENT_STATE_LIMITFLAGS:
					memcpy(&usitmp,data,2);
					sprintf(str,"%s\tLimitFlags\t0x%02X%02X",str,data[1],data[0]);
					break;			
				case ARGUMENT_STATE_ACTUAL_POSITION:
					//memcpy(&itmp,data,4);
					memcpy(&ftmp,data,4);
					sprintf(str,"%s\tActual Position\t%f",str,ftmp);
					break;
				case ARGUMENT_STATE_DESIRED_POSITION:
					memcpy(&ftmp,data,4);
					sprintf(str,"%s\tDesired Position\t%3.5f",str,ftmp);
					break;
				case ARGUMENT_STATE_COMMAND_SIGNAL:
					memcpy(&sitmp,data,2);
					sprintf(str,"%s\tCommand Signal\t%d",str,sitmp);
					break;
				case ARGUMENT_STATE_VELOCITY:
					memcpy(&itmp,data,2);
					sprintf(str,"%s\tVelocity\t%d",str,itmp);
					break;
				case ARGUMENT_STATE_CURRENT:
					memcpy(&usitmp,data,2);
					sprintf(str,"%s\tCurrent\t%d",str,usitmp);
					break;
				case ARGUMENT_STATE_ENCODER:
					memcpy(&itmp,data,4);
//					sprintf(str,"%s\tEncoder\t%d [0x%02X%02X%02X%02X]",str,itmp,data[3]&0xFF,data[2]&0xFF,data[1]&0xFF,data[0]&0xFF);
					sprintf(str,"%s\tEncoder\t%d",str,itmp);
					break;
				case ARGUMENT_STATE_POT:
					memcpy(&usitmp,data,2);
					sprintf(str,"%s\tPotentiometer\t%d",str,usitmp);
					break;
				case ARGUMENT_STATE_RESET:
					memcpy(&usitmp,data,2);
					sprintf(str,"%s\tReset\t0x%02X%02X",str,data[1],data[0]);
					break;
				case ARGUMENT_STATE_MODE:
					memcpy(&usitmp,data,2);
					sprintf(str,"%s\tMode\t0x%02X%02X",str,data[1],data[0]);
					break;
				case ARGUMENT_STATE_CAN_SID:
					memcpy(&usitmp,data,2);
					sprintf(str,"%s\tSID\t0x%02X%02X",str,data[1],data[0]);
					break;
				case ARGUMENT_STATE_DEADZONE:
					memcpy(&sitmp,data,2);
					sprintf(str,"%s\tDeadzone\t%d",str,sitmp);
					break;
				case ARGUMENT_STATE_ACCUMULATOR_LIMIT:
					memcpy(&ftmp,data,4);
					sprintf(str,"%s\tAccumulator Limit\t%3.5f",str,ftmp);
					break;
				case ARGUMENT_STATE_QEI_CONFIG:
					memcpy(&itmp,data,2);
					sprintf(str,"%s\tEncoder Config\t0x%02X : ",str,itmp&0xFF);
					if(itmp&QEI2X)
						sprintf(str,"%s2X ",str);
					if(itmp&QEI4X)
						sprintf(str,"%s4X ",str);
					if(itmp&NOSWAPAB)
						sprintf(str,"%sNO-",str);
					sprintf(str,"%sSWAP",str);
					break;
				default :
					sprintf(str,"%s\tUnknown Argument\t0x%0X",str,data[6]);
					break;				
			}
			break;
		case RESPONSE_VERSION:
			sprintf(str,"%s\tVersion\t0x%02X%02X",str,data[1],data[0]);
			break;
		case RESPONSE_ACK:
			sprintf(str,"%s0x%02X%02X\tACK\t",str,data[5],data[4]);
			break;
		case RESPONSE_ID:
			sprintf(str,"%s0x%02X%02X\tACK\t",str,data[1],data[0]);
			break;
		case RESPONSE_GAIN:
			sprintf(str,"%s0x%02X%02X\t",str,data[6],data[5]);

			switch(data[4]) {
				case ARGUMENT_PGAIN:
					sprintf(str,"%sPGain\t",str);
					break;
				case ARGUMENT_IGAIN:
					sprintf(str,"%sIGain\t",str);
				break;
				case ARGUMENT_DGAIN:
					sprintf(str,"%sDGain\t",str);
					break;
			}
			memcpy(&ftmp,data,4);
			sprintf(str,"%s%3.5f",str,ftmp);
			break;
		case RESPONSE_ERROR:
			sprintf(str,"%s0x%02X%02X\tError\t[ %02x %02x %02x %02x %02x %02x %02x %02x ]",
			str,data[5]&0xff,
			data[4]&0xff,
			data[7]&0xff,data[6]&0xff,
			data[5]&0xff,
			data[4]&0xff,data[3]&0xff,
			data[2]&0xff,
			data[1]&0xff,data[0]&0xff);

			break;
		case RESPONSE_FILLBUFFER:
			//EAT it.
			//str[0]=0x00;
			return 1;
			break;
		default:
			sprintf(str,"%s????\tUnknown packet\t",str);
			sprintf(str,"%s[ %02x %02x %02x %02x %02x %02x %02x %02x ]",str,
				data[7],data[6],data[5],data[4],data[3],data[2],data[1],data[0]);
			break;
	
	}

	sprintf(str,"%s\t[ %02x %02x %02x %02x %02x %02x %02x %02x ]",str,
	data[7] &0xff,data[6]&0xff,data[5]&0xff,data[4]&0xff,data[3]&0xff,data[2]&0xff,data[1]&0xff,data[0]&0xff);
	return 0;
}


int ParseCommand(char *cmd,char *data)
{
	int badcommand,t,itmp;
	char *pch;
	short int sitmp;
	float ftmp;
	char *pEnd;
	char command[80];
	
	strncpy(command,cmd,80);
//	printf("\n|%s|",command);
	
	for(t=0;t<8;t++)
			data[t]=0;

	pch=strtok(command," ");

	t=7;
	badcommand = 999;
	if(pch)
	{
		badcommand=0;
		
		switch(*pch)
		{
			case 'V':
			case 'v':
				data[t--]=COMMAND_QUERY_VERSION;
				break;				
			case 'X':
			case 'x':
				data[t--]=BOOTLOADER_COMMAND_EXIT;
				break;
				
			case 'g':
			case 'G':
				pch=strtok(NULL," ");
				if(pch)
				{
					data[t--]=COMMAND_QUERY_GAIN;
					switch(*pch)
					{
					case 'p':
					case 'P':
						data[t--]=ARGUMENT_PGAIN;
						break;
					case 'i':
					case 'I':
						data[t--]=ARGUMENT_IGAIN;
						break;				
					case 'd':
					case 'D':
						data[t--]=ARGUMENT_DGAIN;
						break;
					default:
						badcommand=31;
						break;
					}
				}
				else
					badcommand=32;
				if(badcommand)
					printf("\ngain query Syntax : g [p/i/d]");
				break;
			case '?':
				pch=strtok(NULL," ");
				if(pch)
				{
					data[t--]=COMMAND_QUERY_STATE;
					switch(*pch)
					{
					case 'w':
					case 'W':
						data[t--]=ARGUMENT_STATE_MAX_PWM_DC;
						break;
					case 'l':
					case 'L':
						data[t--]=ARGUMENT_STATE_LOWERLIMIT;
						break;
					case 'U':
					case 'u':
						data[t--]=ARGUMENT_STATE_UPPERLIMIT;
						break;
					case 'K':
					case 'k':
						data[t--]=ARGUMENT_STATE_LIMITFLAGS;
						break;

					case 'p':
					case 'P':
						data[t--]=ARGUMENT_STATE_ACTUAL_POSITION;
						break;
					case 't':
					case 'T':
						data[t--]=ARGUMENT_STATE_DESIRED_POSITION;
						break;				
					case 'c':
					case 'C':
						data[t--]=ARGUMENT_STATE_COMMAND_SIGNAL;
						break;
					case 'V':
					case 'v':
						data[t--]=ARGUMENT_STATE_VELOCITY;
						break;
					case 'i':
					case 'I':
						data[t--]=ARGUMENT_STATE_CURRENT;
						break;
					case 'D':
					case 'd':
						data[t--]=ARGUMENT_STATE_POT;
						break;
					case 'R':
					case 'r':
						data[t--]=ARGUMENT_STATE_RESET;
						break;
					case 'M':
					case 'm':
						data[t--]=ARGUMENT_STATE_MODE;
						break;
					case 's':
					case 'S':
						data[t--]=ARGUMENT_STATE_CAN_SID;
						break;
					case 'z':
					case 'Z':
						data[t--]=ARGUMENT_STATE_DEADZONE;
						break;
					case 'a':
					case 'A':
						data[t--]=ARGUMENT_STATE_ACCUMULATOR_LIMIT;
						break;							
					case 'e':
					case 'E':
						data[t--]=ARGUMENT_STATE_QEI_CONFIG;
						break;
					case 'Y':
					case 'y':
						data[t--]=ARGUMENT_STATE_ENCODER;
						break;
					default:
						badcommand=3;
						break;
					}
				}
				else
					badcommand=4;
				break;
			case 'b':
			case 'B':
				pch=strtok(NULL," ");
				if(pch)
				{
					data[t--]=COMMAND_SET_GAIN;
					switch(*pch)
					{
					case 'p':
					case 'P':
						data[t--]=ARGUMENT_PGAIN;
						break;
					case 'i':
					case 'I':
						data[t--]=ARGUMENT_IGAIN;
						break;				
					case 'd':
					case 'D':
						data[t--]=ARGUMENT_DGAIN;
						break;
					default:
						badcommand=5;
						break;
					}
				}
				else
					badcommand=6;
				
				pch=strtok(NULL," ");
				if(pch)
				{
					ftmp=atof(pch);
					memcpy(data,&ftmp,4);
					t=0;
				}
				else
					badcommand=7;
				
				if(badcommand)
					printf("\ngain set Syntax : b [p/i/d] [gain value]");
				break;
			case 'o':
			case 'O':
				pch=strtok(NULL," ");
				if(pch)
				{
					data[t--]=COMMAND_SET;
					
					switch(*pch)
					{
					case 'y':
					case 'Y':
						data[t--]=ARGUMENT_STATE_ENCODER;
						pch=strtok(NULL," ");
						if(pch)
						{
							itmp=(int)strtol(pch,&pEnd,10);
							memcpy(data,&itmp,4);
							t=0;
						}
						else
							badcommand=313;
						break;
					
					case 'w':
					case 'W':
						data[t--]=ARGUMENT_STATE_MAX_PWM_DC;
						pch=strtok(NULL," ");
						if(pch)
						{
							itmp=(int)strtol(pch,&pEnd,10);
							memcpy(data,&itmp,2);
							t=0;
						}
						else
							badcommand=312;

						break;
					
						case 'u':
						case 'U':
							pch=strtok(NULL," ");
							data[t--]=ARGUMENT_STATE_UPPERLIMIT;
							if(pch)
							{
								itmp=(int)strtol(pch,&pEnd,10);
								memcpy(data,&itmp,4);
								t=0;
							}
							else
								badcommand=42;				
							break;
						case 'l':
						case 'L':
							pch=strtok(NULL," ");
							data[t--]=ARGUMENT_STATE_LOWERLIMIT;
							if(pch)
							{
								itmp=(int)strtol(pch,&pEnd,10);
								memcpy(data,&itmp,4);
								t=0;
							}
							else
								badcommand=42;				
							break;
						case 'k':
						case 'K':
							pch=strtok(NULL," ");
							data[t--]=ARGUMENT_STATE_LIMITFLAGS;
							if(pch)
							{
								itmp=(int)strtol(pch,&pEnd,16);
								memcpy(data,&itmp,2);
								t=0;
							}
							else
								badcommand=42;				
							break;
						case 'e':
						case 'E':
							pch=strtok(NULL," ");
							itmp=0;
							data[t--]=ARGUMENT_STATE_QEI_CONFIG;
							if(pch)
							{
								if(!strcmp(pch,"4x"))
									itmp|=QEI4X;
								if(!strcmp(pch,"2x"))
									itmp|=QEI2X;
							}
							else
								badcommand=8;
							
							pch=strtok(NULL," ");
							
							if(pch)
							{
								if(!strcmp(pch,"swap"))
								{
									itmp|=SWAPAB;
									printf("\nswap");
								}
								if(!strcmp(pch,"noswap"))
									itmp|=NOSWAPAB;								
							}
							else
								badcommand=9;
							
							memcpy(data,&itmp,2);
							t=0;
							break;
						case 'a':
						case 'A':
							pch=strtok(NULL," ");
							data[t--]=ARGUMENT_STATE_ACCUMULATOR_LIMIT;
							if(pch)
							{
								ftmp=atof(pch);
								memcpy(data,&ftmp,4);
								t=0;
							}
							else
								badcommand=10;
								
							break;
						case 'Z':
						case 'z':
							pch=strtok(NULL," ");
							data[t--]=ARGUMENT_STATE_DEADZONE;
							if(pch)
							{
								sitmp=(short int)strtol(pch,&pEnd,10);
								memcpy(data,&sitmp,2);
								t=0;
							}
							else
								badcommand=11;
								
							break;
						case 'm':
						case 'M':
							pch=strtok(NULL," ");
							data[t--]=ARGUMENT_STATE_MODE;
							if(pch)
							{
								itmp=(int)strtol(pch,&pEnd,16);
								memcpy(data,&itmp,2);
								t=0;
							}
							else
								badcommand=12;
								
							break;
						case 'c':
						case 'C':
							pch=strtok(NULL," ");
							data[t--]=ARGUMENT_STATE_CAN_SID;
							if(pch)
							{
								itmp=(int)strtol(pch,&pEnd,16);
								memcpy(data,&itmp,2);
								t=0;
							}
							else
								badcommand=12;
								
							break;								
						default:
							badcommand=13;
							break;
					}	
				}
				else
					badcommand=14;
					
				if(badcommand)
					printf("\nSyntax : o [l/z/m] [accumulator limit/deadzone/mode]");
				
				break;
					
			case 't':
			case 'T':
				pch=strtok(NULL," ");
				data[t--]=COMMAND_SET_TARGET;
				if(pch)
				{
					ftmp=atof(pch);
					memcpy(data,&ftmp,4);
					t=0;
				}
				else
					badcommand=15;
				
				if(badcommand)
					printf("\ntarget set Syntax : t [target position]");
				break;
			case 'f':
			case 'F':
				printf("\nFinding attached nodes..");
				data[t--]=COMMAND_QUERY_ID;
				badcommand=-1;
				break;				
			case 'q':
			case 'Q':
				printf("\nQuitting\n");
				badcommand=2;
				break;
			case 'r':
			case 'R':
				data[t--]=COMMAND_RESET;
				break;
			case 'E':
			case 'e':
				data[t--]=COMMAND_ENABLE;
				break;
			case 'D':
			case 'd':
				data[t--]=COMMAND_DISABLE;
				break;
			case 'S':
			case 's':
				data[t--]=COMMAND_SAVE;
				break;
			case 'Z':
			case 'z':
				data[t--]=COMMAND_RESTORE;
				break;
			default:
				badcommand=16;
				break;
		}
	}

	return(badcommand);
}

