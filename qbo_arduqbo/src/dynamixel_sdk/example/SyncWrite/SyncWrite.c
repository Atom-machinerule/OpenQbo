//##########################################################
//##                      R O B O T I S                   ##
//##         SyncWrite Example code for Dynamixel.        ##
//##                                           2009.11.10 ##
//##########################################################
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <termio.h>

#include <dynamixel.h>

#define PI	3.141592f
#define NUM_ACTUATOR		3

// Control table address
#define P_GOAL_POSITION_L	30
#define P_GOAL_POSITION_H	31
#define P_GOAL_SPEED_L		32
#define P_GOAL_SPEED_H		33

// Defulat setting
#define DEFAULT_BAUDNUM		1 // 1Mbps
#define NUM_ACTUATOR		3 // Number of actuator
#define STEP_THETA			(PI / 100.0f) // Large value is more fast
#define CONTROL_PERIOD		(10000) // usec (Large value is more slow) 

void PrintCommStatus(int CommStatus);
void PrintErrorCode(void);

int main()
{
	int id[NUM_ACTUATOR];
	int baudnum = 1;
	int deviceIndex = 0;
	float phase[NUM_ACTUATOR];
	float theta = 0;
	int AmpPos = 512;
	//int AmpPos = 2048; // for EX series
	int GoalPos;
	int i;
	int CommStatus;
	printf( "\n\nSyncWrite example for Linux\n\n" );

	// Initialize id and phase
	for( i=0; i<NUM_ACTUATOR; i++ )
	{
		id[i] = i+1;
		phase[i] = 2*PI * (float)i / (float)NUM_ACTUATOR;
	}

	///////// Open USB2Dynamixel ////////////
	if( dxl_initialize(deviceIndex, baudnum) == 0 )
	{
		printf( "Failed to open USB2Dynamixel!\n" );
		printf( "Press Enter key to terminate...\n" );
		getchar();
		return 0;
	}
	else
		printf( "Succeed to open USB2Dynamixel!\n" );
	
	// Set goal speed
	dxl_write_word( BROADCAST_ID, P_GOAL_SPEED_L, 0 );
	// Set goal position
	dxl_write_word( BROADCAST_ID, P_GOAL_POSITION_L, AmpPos );

	while(1)
	{
		printf( "Press Enter key to continue!(press ESC and Enter to quit)\n" );
		if(getchar() == 0x1b)
			break;

		theta = 0;
		do
		{
			// Make syncwrite packet
			dxl_set_txpacket_id(BROADCAST_ID);
			dxl_set_txpacket_instruction(INST_SYNC_WRITE);
			dxl_set_txpacket_parameter(0, P_GOAL_POSITION_L);
			dxl_set_txpacket_parameter(1, 2);
			for( i=0; i<NUM_ACTUATOR; i++ )
			{
				dxl_set_txpacket_parameter(2+3*i, id[i]);
				GoalPos = (int)((sin(theta+phase[i]) + 1.0) * (double)AmpPos);
				printf( "%d  ", GoalPos );
				dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(GoalPos));
				dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(GoalPos));
			}
			dxl_set_txpacket_length((2+1)*NUM_ACTUATOR+4);
			

			printf( "\n" );
			
			dxl_txrx_packet();
			CommStatus = dxl_get_result();
			if( CommStatus == COMM_RXSUCCESS )
			{
				PrintErrorCode();
			}
			else
			{
				PrintCommStatus(CommStatus);
				break;
			}
			
			theta += STEP_THETA;
			usleep(CONTROL_PERIOD);

		}while(theta < 2*PI);
	}

	dxl_terminate();
	printf( "Press Enter key to terminate...\n" );
	getchar();

	return 0;
}

// Print communication result
void PrintCommStatus(int CommStatus)
{
	switch(CommStatus)
	{
	case COMM_TXFAIL:
		printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
		break;

	case COMM_TXERROR:
		printf("COMM_TXERROR: Incorrect instruction packet!\n");
		break;

	case COMM_RXFAIL:
		printf("COMM_RXFAIL: Failed get status packet from device!\n");
		break;

	case COMM_RXWAITING:
		printf("COMM_RXWAITING: Now recieving status packet!\n");
		break;

	case COMM_RXTIMEOUT:
		printf("COMM_RXTIMEOUT: There is no status packet!\n");
		break;

	case COMM_RXCORRUPT:
		printf("COMM_RXCORRUPT: Incorrect status packet!\n");
		break;

	default:
		printf("This is unknown error code!\n");
		break;
	}
}

// Print error bit of status packet
void PrintErrorCode()
{
	if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
		printf("Input voltage error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
		printf("Angle limit error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
		printf("Overheat error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
		printf("Out of range error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
		printf("Checksum error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
		printf("Overload error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
		printf("Instruction code error!\n");
}
