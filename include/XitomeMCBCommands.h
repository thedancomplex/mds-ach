// Xitome MCB Command and response Bytes

// communcation packets to/from the MCB are structured as follows

// commands:
// [command byte] [7 bytes for arguments]
// response
// [response byte] [7 bytes for contents]

#ifndef _XITOMEMCBCOMMANDS_H_
#define _XITOMEMCBCOMMANDS_H_

#define FCY	24000000

//! Maximum Number of AutoUpdate Variable Subscriptions
#define MAXAUTOUPDATEVARIABLES	10

// ** Trajectory Buffer Parameters

//! Size of the trajectory target point ring buffer
#define MAXTARGETBUFFERSIZE	100

//! Default Size of the TrajectoryBuffer
#define DEFAULT_TRAJECTORY_BUFFERSIZE	40

#define TRAJECTORY_PERIOD_10MS		0x03A9
#define TRAJECTORY_PERIOD_5MS		0x01D4
#define TRAJECTORY_PERIOD_1MS		0x005D
#define TRAJECTORY_PERIOD_19PT53125MS	0x0727

//! Trajectory Modes
#define	TRAJECTORY_MODE_NEXTPOINT	0x00
#define TRAJECTORY_MODE_MOVETO		0x01
#define TRAJECTORY_MODE_HOME		0x02

//! Homing States
#define HOME_STATE_IDLE			0x00
#define HOME_STATE_START		0x01
#define	HOME_STATE_FIND_FIRST		0x02
#define	HOME_STATE_FIND_SECOND		0x03
#define HOME_STATE_FIND_FINAL		0x04
#define HOME_STATE_DONE			0x05

#define TRAJECTORY_BUFFER_EMPTY_ARM_THRESHOLD 2

// Max Encoder Frequency
#define ENCODER_7PT812_KHZ	0x7
#define ENCODER_15PT625_KHZ	0x6
#define ENCODER_31PT25_KHZ	0x5
#define ENCODER_62PT5_KHZ	0x4
#define ENCODER_125_KHZ		0x3
#define ENCODER_500_KHZ		0x2
#define ENCODER_1000_KHZ	0x1
#define ENCODER_2000_KHZ	0x0

// LED Mode
#define LED_OFF			0x00
#define LED_TRAJECTORY_BLINK	0x01
#define LED_HEARTBEAT_BLINK	0x02

// Drive Mode
#define OFF			0x0000
#define BLDC			0x0001	// hall inputs connected to QEI
#define BDC			0x0002	// bdc on p1 and p2
#define ALT_HALL		0x0004	// hall inputs on h2 & h3
#define REVERSE_DRIVE		0x0008	// for whatever drive mode selected, change the direction
#define PWM_DRIVE		0x0010	// drive all PWMs simultaneously, useful for controlling LEDs, etc.

// Feedback Mode
#define	ENCODER_FEEDBACK	0x0100	// feedback from optical encoder
#define	ANALOG_FEEDBACK		0x0200	// feedback from analog input (pot)
#define VELOCITY_FEEDBACK	0x0400	// feedback from velocity calculation
#define CURRENT_FEEDBACK	0x0800	// **todo** feedback from current measurement
#define NO_FEEDBACK		0x1000	// no feedback (open loop operation)
#define REVERSE_FEEDBACK	0x2000	// **only applies to analog feedback**
#define ENCODER16		0x4000	// if doing encoder feedback, only use a 16 bit measurement

// Soft Limit Mode
#define NOLIMITS		0x0000
#define ENCODER_UPPER_LIMIT	0x0100
#define POT_UPPER_LIMIT		0x0200
#define ENCODER_LOWER_LIMIT	0x0001
#define POT_LOWER_LIMIT		0x0002

// QEI Mode
#define SWAPAB			0x01
#define NOSWAPAB		0x02
#define QEI4X			0x04
#define QEI2X			0x08


// CAN Commands
// Command packets over the CAN bus are of the following format
// byte [0]   [1]        [2] [3]     [4] [5] [6] [7]
//      [CMD] [argument] [address]   [data         ]

// Commands to set parameters and values on the controller
#define COMMAND_SET_TARGET				0x12
#define COMMAND_SET_GAIN				0x13
#define COMMAND_RESET_ACCUMULATOR_AND_REFERENCE		0x16
#define COMMAND_RESET					0x1F
#define COMMAND_SAVE					0x21
#define COMMAND_RESTORE					0x22
#define COMMAND_SET					0x25
#define COMMAND_GET_CHECKSUM				0x26
#define COMMAND_ENABLE_WDT				0x27
#define COMMAND_DISABLE_WDT				0x28
#define COMMAND_LOCK_CONFIGURATION			0x29
#define COMMAND_UNLOCK_CONFIGURATION			0x2A
#define COMMAND_ZERO_ENCODER				0x2B
#define COMMAND_ENABLE					0x19
#define COMMAND_DISABLE					0x1A

// Commands to query state of things
#define COMMAND_QUERY_STATE				0x17
#define COMMAND_QUERY_MODE				0x1C
#define COMMAND_QUERY_ID				0x1D
#define COMMAND_QUERY_GAIN				0x1E
#define COMMAND_QUERY_VERSION				0x20
#define COMMAND_QUERY_RESET				0x23

// command argument defines
#define ARGUMENT_PGAIN					0x01
#define ARGUMENT_IGAIN					0x02
#define ARGUMENT_DGAIN					0x03

// arguments for the state query and set command. byte 7 of packet
#define FIRST_STATE_ARGUMENT				0x04
#define ARGUMENT_STATE_ACTUAL_POSITION			0x04
#define	ARGUMENT_STATE_DESIRED_POSITION			0x05
#define	ARGUMENT_STATE_COMMAND_SIGNAL			0x06
#define	ARGUMENT_STATE_VELOCITY				0x07
#define	ARGUMENT_STATE_CURRENT				0x08
#define ARGUMENT_STATE_POT				0x09
#define ARGUMENT_STATE_RESET				0x0A
#define ARGUMENT_STATE_MODE				0x0B
#define ARGUMENT_STATE_CAN_SID				0x0C
#define ARGUMENT_STATE_DEADZONE				0x0D
#define ARGUMENT_STATE_ACCUMULATOR_LIMIT		0x0E
#define ARGUMENT_STATE_QEI_CONFIG			0x0F
#define ARGUMENT_STATE_ENCODER				0x10
#define ARGUMENT_STATE_LIMITFLAGS			0x11
#define ARGUMENT_STATE_UPPERLIMIT			0x12
#define ARGUMENT_STATE_LOWERLIMIT			0x13
#define ARGUMENT_STATE_MAX_PWM_DC			0x14
#define ARGUMENT_STATE_TRAJECTORY_PERIOD		0x15
#define ARGUMENT_STATE_AUTO_UPDATE_VALUE		0x16
#define ARGUMENT_STATE_TRAJECTORY_BUFFER_SIZE		0x17
#define ARGUMENT_STATE_MAX_ENCODER_FREQUENCY		0x18
#define ARGUMENT_STATE_PGAIN				0x19
#define ARGUMENT_STATE_IGAIN				0x1A
#define ARGUMENT_STATE_DGAIN				0x1B
#define ARGUMENT_STATE_CAN_ERROR_COUNT			0x1C
#define ARGUMENT_STATE_HALL_VALUE			0x1D
#define ARGUMENT_STATE_PWM1				0x1E
#define ARGUMENT_STATE_PWM2				0x1F
#define ARGUMENT_STATE_PWM3				0x20
#define ARGUMENT_STATE_ISTRAJECTORYRUNNING_RO		0x21
#define ARGUMENT_STATE_TRAJECTORY_MODE			0x22
#define ARGUMENT_STATE_ISENABLED_RO			0x23
#define ARGUMENT_STATE_TRAJECTORY_ACCELERATION		0x24
#define ARGUMENT_STATE_TRAJECTORY_MAX_VELOCITY		0x25
#define ARGUMENT_STATE_TRAJECTORY_ERROR_THRESHOLD	0x26
#define ARGUMENT_STATE_MOVETO_TARGET			0x27
#define ARGUMENT_STATE_TRAJECTORY_TIME_RO		0x28
#define ARGUMENT_STATE_TRAJECTORY_POSITION_RO		0x29
#define ARGUMENT_STATE_TRAJECTORY_VELOCITY_RO		0x2A
#define ARGUMENT_STATE_TRAJECTORY_TIMESTEP		0x2B
#define ARGUMENT_STATE_TRACKING_ERROR			0x2C
#define ARGUMENT_STATE_HOME_STATE_STOP			0x2D
#define LAST_STATE_ARGUMENT				0x2D

#define ARGUMENT_NEXT_BUFFER_POINT			0x30

#define ARGUMENT_ERROR_TRAJECTORY_BUFFER_OVERFLOW	0x50
#define ARGUMENT_ERROR_CANRX_BUFFER_OVERFLOW		0x51
#define ARGUMENT_ERROR_CANTX_BUFFER_OVERFLOW		0x52
#define ARGUMENT_ERROR_CANT_SET_TRAJECTORYBUFFERSIZE	0x53
#define ARGUMENT_ERROR_BAD_ENCODER_FREQUENCY		0x54
#define ARGUMENT_ERROR_CAN				0x55
#define ARGUMENT_ERROR_TRAJECTORY_BUFFER_EMPTY		0x56
#define ARGUMENT_ERROR_UNLOCK				0x57
#define ARGUMENT_ERROR_UNLOCK_FAILURE			0x58
#define ARGUMENT_ERROR_EEPROM_VERIFY			0x59
#define ARGUMENT_ERROR_EEPROM_WRITE			0x5A
#define ARGUMENT_ERROR_EEPROM_READ			0x5B
#define ARGUMENT_ERROR_CANRX_BUFFER_OVERRUN		0x5C
#define ARGUMENT_ERROR_TRAJECTORY_CALCULATION		0x5D
#define ARGUMENT_ERROR_TRACKING_ERROR			0x5E

// bootloader command bytes
#define BOOTLOADER_COMMAND_ERASE			0x01
#define BOOTLOADER_COMMAND_WRITE			0x02
#define BOOTLOADER_COMMAND_READ				0x03
#define BOOTLOADER_COMMAND_READID			0x04
#define BOOTLOADER_COMMAND_EXIT				0x05

// Packets that come FROM MCB nodes begin (byte 7) with a RESPONSE byte,
// followed by arguments (byte 6 - byte 0)

// Debug packet
#define RESPONSE_DEBUG			0x00

// Response Bytes
#define RESPONSE_ID			0x01			// node ID (response to a query ID command)
#define RESPONSE_PGM_ECHO		0x02
#define RESPONSE_GAIN			0x03
#define RESPONSE_VERSION		0x04
#define RESPONSE_STATE			0x05
#define RESPONSE_ACK			0x06
#define RESPONSE_ERROR			0x07
#define RESPONSE_FILLBUFFER		0x08
#define RESPONSE_CHECKSUM		0x09
#define RESPONSE_WARNING		0x0A

#define BOOTLOADER_RESPONSE_PAGE_WRITE	0x0F
#define BOOTLOADER_RESPONSE_ACK		0xFF

#endif

