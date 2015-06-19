/*! Common CAN Functions
* Basic functions used for accessing and initializing the CAN bus
* \brief Common can fuction declaration
* \file CANCommon.h
* \ingroup CommonHelperFunctions
*/

#ifndef _CANCOMMON_H_
#define _CANCOMMON_H_

//! Success
#define CANCOMMON_OK				0

//! Error opening channel
#define CANCOMMON_ERROR_OPENCHANNEL		-1

//! Error setting bus parameters (likely bad parameters)
#define CANCOMMON_ERROR_SETBUSPARAMETERS	-2

//! error encountered getting on the bus
#define CANCOMMON_ERROR_BUSON			-3

//! error encountered while setting the bus output control (not sure exacly why this would happen)
#define CANCOMMON_ERROR_SETBUSOUTPUTCONTROL	-4

//! Nothing to read
#define CANCOMMON_READ_EMPTY	-1

//! generic write error
#define CANCOMMON_WRITE_ERROR -1

//! Generic CAN Interface Class
/* This pure virtual class defines the generic CAN interface required for the API to function.  Support for different
* CAN hardware and drivers is accomplished by deriving a generic CAN object for each one.
* \class CCANCommon
*/
class CCANCommon {
    public:
        CCANCommon(void);

        ~CCANCommon(void);

	//! Reads a message from the CAN Bus
	/*! Reads a message from the can bus (up to 8 bytes) and writes it to the provided buffer, providing a stamp of when the message was
	* taken off the bus
	 * \param *target the address the message was sent to
	* \param *d pointer to the buffer to write the message
	* \param *timestamp pointer to the variable that store the timestamp (in ms)
	*/
        virtual int read(long int *target, unsigned char *d, 
			unsigned long int *timestamp)=0;

	//! Writes a message to the CAN bus
	/*! Writes an 8 byte message to the CAN bus
	* \param target the target address the message should go to
	* \param *d pointer to buffer containing body of message
	*/
        virtual int write(long int target, unsigned char *d)=0;

	//! Initialize CAN Bus
	/*! Initializes the CAN bus hardware and drivers.  Called before use of the bus
	* \param id if there are multiple channels available in a system, id is used to differentiate them
	*/
        virtual int Initialize(int id)=0;

	//! Closes the CAN bus
	/*! Closes the can bus and frees up any allocated memeory
	*/
	virtual int CloseBus(void)=0;

	//! Checks if the last message was good
	/*! Some CAN hardware returns a message read even if it wasn't good (result of colision, etc.)  after a read, the state of the status
	* of the read should be stored and returned by this function
	*/
	virtual int goodmsg(void)=0;

    private:

};
#endif
