/*! Linked List Node for Command List
* Node object for linked list of CAN commands sent to MCB nodes.
* \file CommandNode.h
* 
*/

#ifndef _COMMANDNODE_H_
#define _COMMANDNODE_H_

//! Object that represents a node in the CAN Command list
/*! This object is the basic node in the linked list structure
* that stores the commands that go out on the CAN bus.  CAN
* bus messages are stored in nodes with a tag of the absolute
* time at which they are to be transmitted.
* \class CCommandNode
* \ingroup InternalCommunicationStructures
*/
class CCommandNode {
	public:
		CCommandNode();
		CCommandNode(unsigned char *d,int t,double dt);
		~CCommandNode();
		
		void InsertInTime(CCommandNode *n);
		void InsertAfter(CCommandNode *n);

		//! 8 byte packet to transmit
		unsigned char data[8];
		
		//! target address to send command to
		// in non-esid mode, only lowest 11 bits will be used as the CAN SID
		int target;
		
		//! Absolute time to send the command out at
		double time;
		
		//! pointer to the next node in the list
		CCommandNode *next;
};

/*
struct commandnode {
	struct commandnode * next;
	unsigned char data[8];
	int target;
	double time;
};

typedef struct commandnode cn;

//void InsertInTime(cn ** first, cn *new);
void InsertInTime(cn * first, cn *new);
void InsertAfter(cn *first, cn *new);
cn * CreateNew(char *data,int target,double time);
void Destroy(cn *first);
*/

#endif
