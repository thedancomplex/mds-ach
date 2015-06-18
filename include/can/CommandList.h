/*! Linked List for MCB Commands
* Linked list object to handle collection of command nodes
* \brief Linked list for MCB commands declarations
* \file CommandList.h
* 
*/
#include "CommandNode.h"
#include "pthread.h"

#ifndef _COMMANDLIST_H_
#define _COMMANDLIST_H_

//! Object that represents a CAN Command list
/*! This object represents a linked list of CCommandNodes.  It handles creation and management of the 
* list as well as threadsafe access to it.
* \sa CCommandNode
* \class CCommandList
* \ingroup InternalCommunicationStructures
*/
class CCommandList {
	public:
		CCommandList();
		~CCommandList();

		int IsEmpty(void);

		void InsertInTime(CCommandNode *p);
		
		//! Unlinks and returns first node of the queue
		/*!
		*/
		CCommandNode * PopHead(void);

	private:
		void Insert(CCommandNode *p);
	
		//! pointer to the head of the linked list
		CCommandNode *head;
		
		//! pointer to mutex for locking routines that modify the command list structure
		pthread_mutex_t mut;

		

};

#endif

