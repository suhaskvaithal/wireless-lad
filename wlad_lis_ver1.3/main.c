/* Wireless LAD version 2.8
 *
 * This version covers the following features:
 * 1. Linear Output
 * 2. Send 254 and 255 for \n and \r respectively
 * 3. Grouping feature has been added
 * 4. Scene selection feature has been added
 * 5. UART interrupt modification
 */

#include <msp430g2553.h>

/* Seat occupancy definitions */
#define MAX_DUTY 						3333
#define NULL							0
#define NO								0
#define false							0
#define YES								1
#define ON								1
#define OFF								0
#define true							1
#define UNITY							1
#define MAX_CHAR						8
#define MAX_PIR_COUNT           		1
#define LAST_CHAR						6
#define NO_OF_COMMANDS					38
#define SEND_MSG_LENGTH					2
#define BROADCAST_ADDRESS				254
#define SET_HOST_ADDRESS				0
#define RESET_HOST_ADDRESS				1
#define POLLING_HOST_ADDRESS			2
#define SET_SENSING_FREQ				3
#define SET_TIMEOUT						4
#define SET_POWER_ON_LEVEL				5
#define SET_PERCENTAGE_LEVEL			6
#define SET_FADE_RATE_VALUE				7
#define SET_FADE_DELAY_VALUE			8
#define SET_GROUP_NUMBER				9
#define SET_SCENE_NUMBER				10
#define GET_SCENE_NUMBER				11
#define STORE_LIS_GROUP_NUMBER			12
#define GOTO_SCENE_NUMBER				13
#define GET_GROUP_NUMBER				14
#define CLEAR_SCENE						15
#define IDENTIFY_DEVICE					16
#define FLASH_WRITE						17
#define SET_COMMISSIONING_FLAG			18
#define RESET_COMMISSIONING_FLAG		19
#define GET_SENSOR_DATA					20
#define CLEAR_COUNT						21
#define RELAY_ON						22
#define RELAY_OFF						23
#define DISABLE_OS						24
#define ENABLE_OS						25
#define GET_SENSING_FREQ				26
#define FACTORY_RESET					27
#define GET_SENSOR_SETTINGS				28
#define ENABLE_REQ_MODE					29
#define GET_TIMEOUT_VAL					30
#define GET_PERCENTAGE_LEVEL			31
#define GET_NO_OF_GROUPS				32
#define CLEAR_GROUP						33
#define GET_LIS_GROUP					34
#define CLEAR_LIS_GROUP					35
#define GET_HEALTH_STATUS				36
#define RECEIVE_OWN_ADDRESS				37

#define INPUT_VAL_COMMANDS				12
#define LSB								6
#define MSB								5
#define ASCII_1							49
#define ASCII_0							48											// ASCII value for '0'
#define ASCII_UP_CASE					55
#define ASCII_LOW_CASE					87
#define TEN								10
//#define _0P3_MILLI_SECOND				2500
#define LOWER_BAUD						0x46
#define HIGHER_BAUD						0x00
#define UART_TIMEOUT					8000
#define TIMER1CCR1						2
#define DEVICE_TYPE						8
#define SET								1
#define RESET							0

#define FLASH_ADDRESS_SENSING_FREQ		0x1040										// Segment C
#define FLASH_ADDRESS_TIME_OUT			0x1042										// Segment C
#define FLASH_HOST_ADDRESS				0x1044										// Segment C
#define FLASH_HOST_ADDRESS_1			0x1044
#define FLASH_HOST_ADDRESS_2			0x1045
#define FLASH_HOST_ADDRESS_3			0x1046
#define FLASH_HOST_ADDRESS_4			0x1047
#define FLASH_ADDRESS_COM_FLAG			0x1048										// Segment C
#define FLASH_FADE_RATE_VALUE			0x105A										// Segment C
#define FLASH_FADE_DELAY_VALUE			0x105C										// Segment C
#define FLASH_FADE_DELAY_VALUE_1		0x105C
#define FLASH_FADE_DELAY_VALUE_2		0x105D
#define FLASH_PERCENTAGE_VALUE			0x1060										// Segment C
#define FLASH_SCENE_ONE					0x1062										// Segment C
#define FLASH_SCENE_TWO					0x1064										// Segment C
#define FLASH_SCENE_THREE				0x1066										// Segment C
#define FLASH_SCENE_FOUR				0x1068										// Segment C
#define FLASH_SCENE_FIVE				0x106A										// Segment C
#define FLASH_GROUP_ONE					0x106C										// Segment C
#define FLASH_GROUP_TWO					0x106E										// Segment C
#define FLASH_GROUP_THREE				0x1070										// Segment C
#define FLASH_GROUP_FOUR				0x1072										// Segment C
#define FLASH_GROUP_FIVE				0x1074										// Segment C
#define FLASH_LIS_MODE					0x1076										// Segment C
#define FLASH_LIS_GROUP_NUMBER			0x1078										// Segment C
#define FLASH_POLLING_HOST_ADDRESS		0x107A										// Segment C
#define FLASH_POLLING_HOST_ADDRESS_1	0x107A
#define FLASH_POLLING_HOST_ADDRESS_2	0x107B
#define FLASH_POLLING_HOST_ADDRESS_3	0x107C
#define FLASH_POLLING_HOST_ADDRESS_4	0x107D
#define MAX_FLASH_VAL 					60

/* Occupancy sensor definitions */
#define CALIB_CONST_ERASE    			0xFF										// default flash value after erasing
#define INITIAL_DELAY 					30											// initial delay in mins for sensor stability
#define _15_MIN 						270000										// timer count for 15 mins delay
//#define _15_MIN 						18000										// timer count for 1 mins delay
#define _30_MIN 						540000										// timer count for 30 mins delay
#define _45_MIN 						810000										// timer count for 45 mins delay
#define _60_MIN 						1080000										// timer count for 60 mins delay
#define _15_MIN_VAL						15
#define _30_MIN_VAL						30
#define _45_MIN_VAL						45
#define _60_MIN_VAL						60
#define PWM_WIDTH  						3333										// CCR0 period timer value
#define TIMERCCR1						2
#define _1_MIN							18000										// count for 1 min

void UART_INIT();
void CLOCK_INIT();
void TIMER_INIT();
void TIMER_DISABLE();
void REQUEST_MODE_TIMER_INIT();
void REQUEST_MODE_TIMER_DISABLE();
void FLASH_INIT();
void PORT_INIT();
void PORT_OUT_INIT();
void PIR_INIT();
void SYS_INIT();
void print_val1(unsigned int Value, unsigned int polling_host);
void print_address(unsigned int isHostAddress);
void print_char(unsigned char character);
void print_setting();
void print_command(int lis_value);
void flash_write();
void flash_erase();
unsigned char flash_read(unsigned int address);
void set_duty_cycle(unsigned int Percentage_val);
void get_ble_address();

int i = NULL;
unsigned int j = NULL;
static const char msg_arr[NO_OF_COMMANDS][MAX_CHAR] = {
	"ADHxxxxx", "ADRxxxxx", "ADPxxxxx", "ADSSFxxx", "ADSTOxxx", "ADOPLxxx", "ADSPLxxx",	"ADFDRxxx", "ADFDDxxx", "ADSGPxxx", "ADSSNxxx", "ADGSNxxx",
	"ADSTGxxx", "ADGTSNxx", "ADGGPNxx", "ADCLRSxx", "ADIDDEVx", "ADWRFLSx", "ADCMSETx", "ADCMRSTx", "ADGSD00x", "ADCLROSx", "ADLADONx", "ADLADOFx",
	"ADDISOSx", "ADENAOSx", "ADGSF00x", "ADFTRSTx", "ADGSSETx", "ADERQOSx", "ADGTOOSx", "ADGPLADx",	"ADGNOGPx", "ADCLRGPx", "ADGGPLSx", "ADCRGPLx",
	"ADGSTATx", "ADDROADx" };

static const char send_msg[SEND_MSG_LENGTH] = {"D "};
static const char LIS_command[5] = {'A', 'D', 'S', 'P', 'L'};
unsigned char character_count = NULL;
unsigned int checksum;
unsigned char msb = NULL, lsb = NULL, input_val = NULL;
unsigned int timer_count = NULL;
unsigned char BLE_address[4];
unsigned int BLE_address_encrypted[2];
unsigned char HOST_address[4] = {'1','2','3','4'};
unsigned int HOST_address_ASCII[4];
unsigned int HOST_address_encrypted[2];
unsigned char POLLING_HOST_address[4] = {'4','3','2','1'};
unsigned int POLLING_HOST_address_ASCII[4];
unsigned int POLLING_HOST_address_encrypted[2];

unsigned char iteration = 3;
unsigned char iteration1 = 5;
unsigned char pir_flag = 0;
unsigned char pir_count = 0;
unsigned char received_char = 0;

/* Flags */
unsigned char request_mode_flag = 1;
unsigned char ping_flag;
unsigned char timer_flag;
unsigned int sensing_freq = 15;
unsigned int present_percentage;
unsigned int present_count = 0;
unsigned int timer_count_1 = NULL;
unsigned long sensing_freq_val;
unsigned char commissioning_flag = 0;
unsigned char no_of_groups = 0;
unsigned char group_array[5] = {0xff, 0xff, 0xff, 0xff, 0xff};
unsigned char group_match = false;
unsigned char group_array_index = 0;
unsigned int group_address_encrypted[3];
unsigned char scene_number = NULL;
unsigned char go_to_scene = 0xff;
unsigned char get_scene_number = 0;
unsigned char scene_one = 99;
unsigned char scene_two = 99;
unsigned char scene_three = 99;
unsigned char scene_four = 99;
unsigned char scene_five = 99;
unsigned char store_group_value;
unsigned char lis_on_send_flag = false;
unsigned char lis_mode = OFF;

/* Occupancy Sensor variables */
unsigned long timer_int_count = NULL;										// timer interrupt entry count
unsigned char time_out_flag = NO;
unsigned long max_timer_count = _15_MIN;									// timer count for 15 min default
unsigned char received_val[MAX_CHAR];										// received chararcter array through UART
unsigned long time_out = NULL;
unsigned char temp_delay[2] = {3, 0};
unsigned long delay_value = 300;
unsigned long long_delay;
unsigned char sensor_there = false;

/* PWM generation variables */
unsigned int current_duty = NULL;
unsigned int target_duty = NULL;
int fade_rate_val = 1;
int flash_fade_rate_val;
int percentage_val = 99;
unsigned int percent_normalize;
unsigned int case_value;
unsigned char power_on_value = 99;
unsigned char isOff = false;
unsigned char pirOff = true;
unsigned char char_change_flag = UNITY;
unsigned int command_index_match = 255;
unsigned char junk = NULL;

void main(void)
{
	WDTCTL = WDTPW | WDTHOLD;												// Stop watchdog timer
	CLOCK_INIT();
	UART_INIT();
	FLASH_INIT();
	TIMER_INIT();															// Initialise timer for occupancy sensor operation
	PORT_OUT_INIT();
	SYS_INIT();

	/* Dim down */
	if (target_duty > current_duty)
	{
		for (i = current_duty; i <= target_duty; i += fade_rate_val)
		{
			CCR1 += fade_rate_val;
			if(CCR1 >= target_duty + 1)
			{
				CCR1   = target_duty;
				if(isOff == true)
				{
					P2OUT &= ~BIT2;
				}
				break;
			}
			for(j=0; j<delay_value; j++)
			{
				__delay_cycles(1);
			}
		}
	}
	current_duty = target_duty;

	for(i=0; i<5; i++)
	{
		__delay_cycles(8000000);											// Wait for 15 seconds for sensor to initaialize
	}

	get_ble_address();

	PORT_INIT();																// Initialise the I/O peripherals
	REQUEST_MODE_TIMER_INIT();													// Initialise timer for uart mode and request mode operation

	if(lis_mode == ON)
	{
		sensor_there		= true;
		PIR_INIT();
	}
	else
	{
//		P1IE 				&= ~BIT5;											// Disable the PIR sensor interrupt
		TIMER_DISABLE();														// Disable occupancy sensor timer
		REQUEST_MODE_TIMER_DISABLE();											// Disable request mode timer
	}

	while(true)
	{
		__bis_SR_register(LPM0_bits + GIE);

		if(lis_on_send_flag == true)
		{
			lis_on_send_flag = false;
			print_command(true);
		}

		/* Timer indication of 15s timeout to detect the motion for analytics*/
		if(request_mode_flag == YES)
		{
			if (timer_flag == YES)
			{
				timer_flag = NO;
				if (pir_flag == YES)											// Check whether motion detected
				{
					present_count++;
					if (present_count >= 255)
						present_count = NULL;
					pir_flag = NO;												// Clear PIR flag
				}
			}
		}

		if(time_out_flag == YES)										// Timeout indication for occupancy sensor to turn off the load
		{
			P2OUT &= ~BIT2;												// switch off all the lights
			pirOff = 1;
//			__delay_cycles(_0_5_SEC_COUNT);
			P1IFG &= ~BIT5;												// clear interrupt flag to avoid flase trigger because of relay noise
			time_out_flag = NO;
			TIMER_DISABLE();
			print_command(false);
		}

		if(character_count == MAX_CHAR)													// Processes the UART command request
		{
			character_count = NULL;
			IE2 &= ~UCA0RXIE;
			for(i=NULL; i<5; i++)
			{
				if((received_val[7] == BROADCAST_ADDRESS) || (received_val[7] == group_array[i]))
				{
					group_match = true;
					break;
				}
			}
			if(group_match == true)
			{
				timer_count = NULL;

				for(j=NULL; j<NO_OF_COMMANDS; j++)
				{
					for(i=NULL; i<(MAX_CHAR-1); i++)
					{
						if(received_val[i] == msg_arr[j][i])
						{
							if((i == LAST_CHAR) && (j == IDENTIFY_DEVICE) && (command_index_match != IDENTIFY_DEVICE))
							{
								/* ADIDDEV */
								print_char('s');															// Send acknowledgement
								P2OUT 				|= BIT2;												// Switch ON load
								__delay_cycles(8000000);													// Delay of 1 second
								P2OUT 				&= ~BIT2;												// Switch OFF load
								__delay_cycles(8000000);													// Delay of 1 second
								P2OUT 				|= BIT2;												// Switch ON load
								P1IFG 				&= ~BIT5;												// Clear the interrupt flag
								command_index_match = IDENTIFY_DEVICE;
								j 					= NO_OF_COMMANDS;										// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i == LAST_CHAR) && (j == FLASH_WRITE) && (command_index_match != FLASH_WRITE))
							{
								/* ADWRFLS */
								print_char('s');												// Send acknowledgement
								flash_write();																// Write to flash memory
								command_index_match = FLASH_WRITE;
								j 					= NO_OF_COMMANDS;												// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i == LAST_CHAR) && (j == SET_COMMISSIONING_FLAG) && (command_index_match != SET_COMMISSIONING_FLAG))
							{
								/* ADCMSET */
								print_char('s');												// Send acknowledgement
								commissioning_flag = YES;										// Set the commissioning flag
								command_index_match = SET_COMMISSIONING_FLAG;
								j 					= NO_OF_COMMANDS;												// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i == LAST_CHAR) && (j == RESET_COMMISSIONING_FLAG) && (command_index_match != RESET_COMMISSIONING_FLAG))
							{
								/* ADCMRST */
								print_char('s');												// Send acknowledgement
								commissioning_flag = NO;										// Reset the commissioning flag
								command_index_match = RESET_COMMISSIONING_FLAG;
								j 					= NO_OF_COMMANDS;												// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i == LAST_CHAR) && (j == GET_SENSOR_DATA) && (command_index_match != GET_SENSOR_DATA))
							{
								/* ADGSD00 */
								print_val1(present_count+1,2);									// Send the count of movement
								command_index_match = GET_SENSOR_DATA;
								j 					= NO_OF_COMMANDS;												// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i == LAST_CHAR) && (j == CLEAR_COUNT) && (command_index_match != CLEAR_COUNT))
							{
								/* ADCLROS */
								print_char('s');												// Send acknowledgement
								present_count 		= NULL;											// Clear the count value
								command_index_match = CLEAR_COUNT;
								j 					= NO_OF_COMMANDS;												// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i == LAST_CHAR) && (j == RELAY_ON) && (command_index_match != RELAY_ON))
							{
								/* ADLADON */
								print_char('s');												// Send acknowledgement
								P2OUT 				|= BIT2;													// Switch on all lights
								P1IE 				&= ~BIT5;													// Disable PIR sensor interrupt
								timer_int_count 	= NULL;											// Clear occupancy sensor timer count
								timer_count_1 		= NULL;											// Clear sensing frequecy timer count
								isOff 				= false;
								target_duty 		= NULL;
								CCR1 				= NULL;
								sensor_there		= false;
								TIMER_DISABLE();												// Disable occupancy sensor timer
								REQUEST_MODE_TIMER_DISABLE();									// Disable request mode timer
								command_index_match = RELAY_ON;
								j 					= NO_OF_COMMANDS;												// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i == LAST_CHAR) && (j == RELAY_OFF) && (command_index_match != RELAY_OFF))
							{
								/* ADLADOF */
								print_char('s');												// Send acknowledgement
								P2OUT 				&= ~BIT2;													// switch off all lights
								P1IE 				&= ~BIT5;													// Disable PIR sensor interrupt
								timer_int_count 	= NULL;											// Clear occupancy sensor timer count
								timer_count_1 		= NULL;											// Clear sensing frequecy timer count
								isOff 				= true;
								target_duty 		= 3300;
								CCR1 				= 3300;
								sensor_there 		= false;
								TIMER_DISABLE();												// Disable occupancy sensor timer
								REQUEST_MODE_TIMER_DISABLE();									// Disable request mode timer
								command_index_match = RELAY_OFF;
								j 					= NO_OF_COMMANDS;												// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i == LAST_CHAR) && (j == DISABLE_OS) && (command_index_match != DISABLE_OS))
							{
								/* ADDISOS */
								print_char('s');												// Send acknowledgement
								P1IE 				&= ~BIT5;													// Disable the PIR sensor interrupt
								sensor_there 		= false;
								lis_mode			= OFF;
								TIMER_DISABLE();												// Disable occupancy sensor timer
								REQUEST_MODE_TIMER_DISABLE();									// Disable request mode timer
								command_index_match = DISABLE_OS;
								j 					= NO_OF_COMMANDS;												// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i == LAST_CHAR) && (j == ENABLE_OS) && (command_index_match != ENABLE_OS))
							{
								/* ADENAOS */
								print_char('s');												// Send acknowledgement
								PIR_INIT();
								sensor_there		= true;
								lis_mode			= ON;
								TIMER_INIT();													// Enable occupancy sensor timer
								REQUEST_MODE_TIMER_INIT();										// Enable request mode timer
								command_index_match = ENABLE_OS;
								j 					= NO_OF_COMMANDS;												// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i == LAST_CHAR) && (j == GET_SENSING_FREQ) && (command_index_match != GET_SENSING_FREQ))
							{
								/* ADGSF00 */
								print_val1(sensing_freq,2);										// Send the sensing frequency value
								command_index_match = GET_SENSING_FREQ;
								j 					= NO_OF_COMMANDS;												// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i == LAST_CHAR) && (j == FACTORY_RESET) && (command_index_match != FACTORY_RESET))
							{
								/* ADFTRST */
								print_char('s');												// Send acknowledgement
								P1IE 				&= ~BIT5;													// Disable the PIR sensor interrupt
								sensor_there 		= false;
								lis_mode			= false;
								pir_flag 			= NO;													// Clear the PIR flag
								present_count 		= NULL;											// Clear the present count
								sensing_freq 		= 15;
								sensing_freq_val 	= (15 * 1000000)/8192;							// Set the sensing frequency to 15s
								timer_int_count 	= NULL;											// Clear occupancy sensor timer count
								timer_count_1 		= NULL;											// Clear sensing frequecy timer count
								max_timer_count 	= _15_MIN;										// Set max_timer_count value to 15 mins
								time_out 			= max_timer_count / _1_MIN;							// Calculate the occupancy sensor time out value
								P2OUT 			   	|= BIT2;													// Switch ON the load
								target_duty 		= NULL;
								CCR1 				= RESET;
								isOff 				= false;
								pirOff 				= true;
								power_on_value 		= 99;
								delay_value 		= 300;
								temp_delay[0] 		= 3;
								temp_delay[1] 		= 0;
								fade_rate_val 		= 1;
								commissioning_flag 	= RESET;

								for(i=0;i<5;i++)
								{
									group_array[i]  = 0xff;
								}

								scene_one			= 99;
								scene_two			= 99;
								scene_three			= 99;
								scene_four			= 99;
								scene_five			= 99;
								scene_number		= 0;
								TIMER_INIT();													// Enable occupancy sensor timer & set pwm output mode
								TIMER_DISABLE();												// Disable occupancy sensor timer
								REQUEST_MODE_TIMER_DISABLE();									// Disable request mode timer
								flash_erase();													// Write into flash memory
								command_index_match = FACTORY_RESET;
								j 					= NO_OF_COMMANDS;												// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i == LAST_CHAR) && (j == GET_SENSOR_SETTINGS) && (command_index_match != GET_SENSOR_SETTINGS))
							{
								/* ADGSSET */
								print_setting();												// Send sensing frequency and time out value
								command_index_match = GET_SENSOR_SETTINGS;
								j 					= NO_OF_COMMANDS;												// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i == LAST_CHAR) && (j == ENABLE_REQ_MODE) && (command_index_match != ENABLE_REQ_MODE))
							{
								/* ADERQOS */
								print_char('s');												// Send acknowledgement
								request_mode_flag 	= YES;
								PIR_INIT();
								pir_flag 			= NO;													// Clear pir_flag
								P2OUT 				|= BIT2;													// Switch ON load
								target_duty 		= NULL;
								isOff 				= false;
								power_on_value 		= 99;
								delay_value 		= 300;
								temp_delay[0] 		= 3;
								temp_delay[1] 		= 0;
								fade_rate_val 		= 1;
								sensor_there 		= true;
								lis_mode			= ON;

								TIMER_INIT();													// Start 15s and occupancy sensor timer
								REQUEST_MODE_TIMER_INIT();										// Start 15s request mode timer
								command_index_match = ENABLE_REQ_MODE;
								j 					= NO_OF_COMMANDS;												// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i == LAST_CHAR) && (j == GET_TIMEOUT_VAL) && (command_index_match != GET_TIMEOUT_VAL))
							{
								/* ADGTOOS */
								time_out 			= max_timer_count / _1_MIN;							// Calculate the time out value
								print_val1(time_out,2);											// Send time out value
								command_index_match = GET_TIMEOUT_VAL;
								j 					= NO_OF_COMMANDS;												// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i == LAST_CHAR) && (j == GET_PERCENTAGE_LEVEL) && (command_index_match != GET_PERCENTAGE_LEVEL))
							{
								/* ADGPLAD */
								print_val1(percentage_val,2);												// Send acknowledgement
								command_index_match = GET_PERCENTAGE_LEVEL;
								j 					= NO_OF_COMMANDS;												// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i == LAST_CHAR) && (j == GET_NO_OF_GROUPS) && (command_index_match != GET_NO_OF_GROUPS))
							{
								/* ADGNOGP */
								no_of_groups = 0;
								for(i=0; i<5; i++)
								{
									if(group_array[i] != 0xff)
									{
										no_of_groups++;
									}
								}
								print_val1(no_of_groups,2);
								command_index_match = GET_NO_OF_GROUPS;
								j 					= NO_OF_COMMANDS;  											// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i == LAST_CHAR) && (j == CLEAR_GROUP) && (command_index_match != CLEAR_GROUP))
							{
								/* ADCLRGP */
								print_char('s');
								for(i=NULL; i<5; i++)
								{
									if(received_val[7] == group_array[i])
									{
										group_array[i] = 0xff;
									}
								}
								command_index_match = CLEAR_GROUP;
								j 					= NO_OF_COMMANDS;  											// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i == LAST_CHAR) && (j == GET_LIS_GROUP) && (command_index_match != GET_LIS_GROUP))
							{
								/* ADGGPLS */
								print_val1(store_group_value,2);
								command_index_match = GET_LIS_GROUP;
								j 					= NO_OF_COMMANDS;  											// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i == LAST_CHAR) && (j == CLEAR_LIS_GROUP) && (command_index_match != CLEAR_LIS_GROUP))
							{
								/* ADCRGPL */
								print_char('s');
								store_group_value 	= 0xff;
								command_index_match = CLEAR_LIS_GROUP;
								j 					= NO_OF_COMMANDS;  											// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i == LAST_CHAR) && (j == GET_HEALTH_STATUS) && (command_index_match != GET_HEALTH_STATUS))
							{
								/* ADGSTAT */
								print_val1(percentage_val,1);												// Send acknowledgement
								command_index_match = GET_HEALTH_STATUS;
								j 					= NO_OF_COMMANDS;  											// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i == LAST_CHAR) && (j == RECEIVE_OWN_ADDRESS) && (command_index_match != RECEIVE_OWN_ADDRESS))
							{
								/* ADDROAD */
								get_ble_address();
								print_address(2);
								command_index_match = RECEIVE_OWN_ADDRESS;
								j 					= NO_OF_COMMANDS;  											// To break out of both the for loops
								i 					= MAX_CHAR;
							}
						}
						else
						{
							if((i == 3) && (j == SET_HOST_ADDRESS) && (commissioning_flag == 0) && (command_index_match != SET_HOST_ADDRESS))		// Change host address if sensor is not commissioned
							{
								/* ADHxxxx */
								for(i=NULL; i<4; i++)
								{
									HOST_address[i] 	  = received_val[iteration++];				// Store the address in the HOST_address variable
								}
								for(i=0; i<4; i++)
								{
									if (HOST_address[i] >= '0' && HOST_address[i] <= '9')
									{
										HOST_address_ASCII[i] = HOST_address[i] - ASCII_0;
									}
									else if (HOST_address[i] >= 'A' && HOST_address[i] <= 'F' )
									{
										HOST_address_ASCII[i] = HOST_address[i] - ASCII_UP_CASE;
									}
									else if (HOST_address[i] >= 'a' && HOST_address[i] <= 'f' )
									{
										HOST_address_ASCII[i] = HOST_address[i] - ASCII_LOW_CASE;
									}
								}
								HOST_address_encrypted[0] = (HOST_address_ASCII[0] << 4) + HOST_address_ASCII[1];
								HOST_address_encrypted[1] = (HOST_address_ASCII[2] << 4) + HOST_address_ASCII[3];
								iteration 				  = 3;
								print_address(2);
								command_index_match 	  = SET_HOST_ADDRESS;
								j 						  = NO_OF_COMMANDS;
								i 						  = MAX_CHAR;
							}
							else if((i == 3) && (j == RESET_HOST_ADDRESS) && (command_index_match != RESET_HOST_ADDRESS))
							{
								/* ADRxxxx */
								for(i=NULL; i<4; i++)
								{
									HOST_address[i] 	= received_val[iteration++];
								}
								for(i=0; i<4; i++)
								{
									if (HOST_address[i] >= '0' && HOST_address[i] <= '9')
									{
										HOST_address_ASCII[i] = HOST_address[i] - ASCII_0;
									}
									else if (HOST_address[i] >= 'A' && HOST_address[i] <= 'F' )
									{
										HOST_address_ASCII[i] = HOST_address[i] - ASCII_UP_CASE;
									}
									else if (HOST_address[i] >= 'a' && HOST_address[i] <= 'f' )
									{
										HOST_address_ASCII[i] = HOST_address[i] - ASCII_LOW_CASE;
									}
								}
								HOST_address_encrypted[0] = (HOST_address_ASCII[0] << 4) + HOST_address_ASCII[1];
								HOST_address_encrypted[1] = (HOST_address_ASCII[2] << 4) + HOST_address_ASCII[3];
								iteration 				= 3;
								print_address(2);
								command_index_match 	= RESET_HOST_ADDRESS;
								j 						= NO_OF_COMMANDS;
								i 						= MAX_CHAR;
							}
							else if((i == 3) && (j == POLLING_HOST_ADDRESS) && (command_index_match != POLLING_HOST_ADDRESS))
							{
								/* ADPxxxx */
								for(i=NULL; i<4; i++)
								{
									POLLING_HOST_address[i] 	= received_val[iteration++];
								}
								for(i=0; i<4; i++)
								{
									if (POLLING_HOST_address[i] >= '0' && POLLING_HOST_address[i] <= '9')
									{
										POLLING_HOST_address_ASCII[i] = POLLING_HOST_address[i] - ASCII_0;
									}
									else if (POLLING_HOST_address[i] >= 'A' && POLLING_HOST_address[i] <= 'F' )
									{
										POLLING_HOST_address_ASCII[i] = POLLING_HOST_address[i] - ASCII_UP_CASE;
									}
									else if (POLLING_HOST_address[i] >= 'a' && POLLING_HOST_address[i] <= 'f' )
									{
										POLLING_HOST_address_ASCII[i] = POLLING_HOST_address[i] - ASCII_LOW_CASE;
									}
								}
								POLLING_HOST_address_encrypted[0] = (POLLING_HOST_address_ASCII[0] << 4) + POLLING_HOST_address_ASCII[1];
								POLLING_HOST_address_encrypted[1] = (POLLING_HOST_address_ASCII[2] << 4) + POLLING_HOST_address_ASCII[3];
								iteration 				= 3;
								print_address(1);
								command_index_match 	= POLLING_HOST_ADDRESS;
								j 						= NO_OF_COMMANDS;
								i 						= MAX_CHAR;
							}
							else if((i == 5) && (j == SET_FADE_DELAY_VALUE) && (command_index_match != SET_FADE_DELAY_VALUE))
							{
								/* ADFDDxx */
								print_char('s');
								for(i=NULL; i<2; i++)
								{
									temp_delay[i] 		= received_val[iteration1++];
									temp_delay[i] 		= temp_delay[i] - ASCII_0;
								}
								iteration1 				= 5;
								delay_value 			= ((temp_delay[0] * 10) + (temp_delay[1] * 1)) * 10;
								command_index_match 	= SET_FADE_DELAY_VALUE;
								j			 			= NO_OF_COMMANDS;									// Break out of both the for loops
								i 						= MAX_CHAR;
							}
							else if((i == 5) && (j <= INPUT_VAL_COMMANDS))
							{
								msb 					= received_val[MSB];								// retreive MSB from the command
								lsb 					= received_val[LSB];								// retreive LSB from the command
								lsb 					= lsb - ASCII_0;									// get the actual value after subtracting the ascii 0
								msb 					= msb - ASCII_0;
								input_val 				= (msb * TEN) + lsb;							// get the actual number
								if((j == SET_TIMEOUT) && (command_index_match != SET_TIMEOUT))
								{
									/* ADSTOxx */
									print_char('s');
									time_out 			= input_val;								// Store the number in time_out variable
									timer_int_count 	= NULL;
									max_timer_count 	= time_out * _1_MIN;				// Convert the number into count value
									command_index_match = SET_TIMEOUT;
									j 					= NO_OF_COMMANDS;									// Break out of both the for loops
									i 					= MAX_CHAR;
								}
								else if((j == SET_SENSING_FREQ) && (command_index_match != SET_SENSING_FREQ))
								{
									/* ADSSFxx */
									print_char('s');
									sensing_freq 		= input_val;							// Store the number in sensing_freq variable
									sensing_freq_val 	= ((sensing_freq * 1000000)/8192);		// Convert the number into sensing frequency value count
									timer_count_1 		= NULL;								// Clear the sensing frequency timer count
									timer_flag 			= NO;									// Reset the sensing frequency timer flag
									command_index_match = SET_SENSING_FREQ;
									j = NO_OF_COMMANDS;									// Break out of both the for loops
									i = MAX_CHAR;
								}
								else if((j == SET_POWER_ON_LEVEL) && (command_index_match != SET_POWER_ON_LEVEL))
								{
									/* ADOPLxx Power On Level*/
									print_char('s');
									power_on_value 		= input_val;
									command_index_match = SET_POWER_ON_LEVEL;
									j 					= NO_OF_COMMANDS;									// Break out of both the for loops
									i 					= MAX_CHAR;
								}
								else if((j == SET_PERCENTAGE_LEVEL) && (command_index_match != SET_PERCENTAGE_LEVEL))
								{
									/* ADSPLxx */
									print_char('s');
									percentage_val 		= input_val;
									set_duty_cycle(percentage_val);
									command_index_match = SET_PERCENTAGE_LEVEL;
									j 					= NO_OF_COMMANDS;									// Break out of both the for loops
									i 					= MAX_CHAR;
								}
								else if((j == SET_FADE_RATE_VALUE) && (command_index_match != SET_FADE_RATE_VALUE))
								{
									/* ADFDRxx */
									print_char('s');
									fade_rate_val 		= input_val;
									command_index_match = SET_FADE_RATE_VALUE;
									j					= NO_OF_COMMANDS;									// Break out of both the for loops
									i 					= MAX_CHAR;
								}
								else if((j == SET_GROUP_NUMBER) && (command_index_match != SET_GROUP_NUMBER))
								{
									/* ADSGPxx */
									print_char('s');
									group_array[group_array_index]	= input_val;
									group_array_index++;
									if(group_array_index >= 5)
									{
										group_array_index = 0;
									}
									command_index_match = SET_GROUP_NUMBER;
									j					= NO_OF_COMMANDS;									// Break out of both the for loops
									i 					= MAX_CHAR;
								}
								else if((j == SET_SCENE_NUMBER) && (command_index_match != SET_SCENE_NUMBER))
								{
									/* ADSSNxx */
									print_char('s');
									scene_number = input_val;
									switch(scene_number)
									{
									case 1:
										scene_one 	= percentage_val;
										break;
									case 2:
										scene_two 	= percentage_val;
										break;
									case 3:
										scene_three = percentage_val;
										break;
									case 4:
										scene_four 	= percentage_val;
										break;
									case 5:
										scene_five 	= percentage_val;
										break;
									default:
										break;
									}
									scene_number = 0;
									command_index_match = SET_SCENE_NUMBER;
									j					= NO_OF_COMMANDS;									// Break out of both the for loops
									i 					= MAX_CHAR;
								}
								else if((j == GET_SCENE_NUMBER) && (command_index_match != GET_SCENE_NUMBER))
								{
									/* ADGSNxx */
									get_scene_number 	= input_val;
									switch(get_scene_number)
									{
									case 1:
										print_val1(scene_one,2);
										break;
									case 2:
										print_val1(scene_two,2);
										break;
									case 3:
										print_val1(scene_three,2);
										break;
									case 4:
										print_val1(scene_four,2);
										break;
									case 5:
										print_val1(scene_five,2);
										break;
									default:
										break;
									}
									command_index_match = GET_SCENE_NUMBER;
									j 					= NO_OF_COMMANDS;									// Break out of both the for loops
									i 					= MAX_CHAR;
								}
								else if((j == STORE_LIS_GROUP_NUMBER) && (command_index_match != STORE_LIS_GROUP_NUMBER))
								{
									/* ADSTGxx */
									print_char('s');
									store_group_value   = input_val;
									command_index_match = STORE_LIS_GROUP_NUMBER;
									j					= NO_OF_COMMANDS;									// Break out of both the for loops
									i 					= MAX_CHAR;
								}
							}
							else if((i==6) && (j == GOTO_SCENE_NUMBER) && (command_index_match != GOTO_SCENE_NUMBER))
							{
								/* ADGTSNx */
								print_char('s');
								go_to_scene		 	= received_val[6] - ASCII_0;
								switch(go_to_scene)
								{
								case 1:
									percentage_val  = scene_one;
									set_duty_cycle(percentage_val);
									break;
								case 2:
									percentage_val  = scene_two;
									set_duty_cycle(percentage_val);
									break;
								case 3:
									percentage_val  = scene_three;
									set_duty_cycle(percentage_val);
									break;
								case 4:
									percentage_val  = scene_four;
									set_duty_cycle(percentage_val);
									break;
								case 5:
									percentage_val  = scene_five;
									set_duty_cycle(percentage_val);
									break;
								default:
									break;
								}
								command_index_match = GOTO_SCENE_NUMBER;
								j					= NO_OF_COMMANDS;									// Break out of both the for loops
								i 					= MAX_CHAR;
							}
							else if((i==6) && (j==GET_GROUP_NUMBER) && (command_index_match != GET_GROUP_NUMBER))
							{
								/* ADGGPNx */
								unsigned int group_array_val = received_val[6] - ASCII_1;
								print_val1(group_array[group_array_val],2);
   								command_index_match = GET_GROUP_NUMBER;
								j					= NO_OF_COMMANDS;
								i 					= MAX_CHAR;
							}
							else if((i == 6) && (j == CLEAR_SCENE) && (command_index_match != CLEAR_SCENE))
							{
								/* ADCLRSx */
								print_char('s');
								scene_number = received_val[6] - ASCII_0;
								switch(scene_number)
								{
								case 1:
									scene_one = 99;
									break;
								case 2:
									scene_two = 99;
									break;
								case 3:
									scene_three = 99;
									break;
								case 4:
									scene_four = 99;
									break;
								case 5:
									scene_five = 99;
									break;
								default:
									break;
								}
								command_index_match = CLEAR_SCENE;
								j 					= NO_OF_COMMANDS;  											// To break out of both the for loops
								i 					= MAX_CHAR;
							}
							break;														// Break out of the for loops
						}
					}
				}
				group_match = false;
			}
			for (i = 0; i<10 ; i++)
			{
				junk = UCA0RXBUF;
			}
			IFG2 &= ~UCA0RXIFG;
			IE2 |= UCA0RXIE;
		}
		/* Dim down */
		if (target_duty > current_duty)
		{
			P2OUT |= BIT2;
			for (i = current_duty; i <= target_duty; i += fade_rate_val)		// Dim down the intensity level
			{
				CCR1 	+= fade_rate_val;
				if(CCR1 >= target_duty + 1)
				{
					CCR1 = target_duty;
					if(isOff == true)
					{
						P2OUT &= ~BIT2;
					}
					break;
				}
				for(j=0; j<delay_value; j++)
				{
					__delay_cycles(1);
				}
			}
		}
		/* Dim up */
		else if (target_duty < current_duty)
		{
			P2OUT 	|= BIT2;
			for (i = current_duty; i >= target_duty; i -= fade_rate_val)
			{
				CCR1 	-= fade_rate_val;
				if(CCR1 <= target_duty - 1)
				{
					CCR1 = target_duty;
					break;
				}
				for(j=0; j<delay_value; j++)
				{
					__delay_cycles(1);
				}
			}
		}
		current_duty = target_duty;
	}
}

void get_ble_address()
{
	ping_flag = YES;														// Set the ping_flag to receive the BLE address
	IFG2 &= ~UCA0RXIFG;
	IE2 |= UCA0RXIE;
	for (i = 0; i<10 ; i++)
	{
		junk = UCA0RXBUF;
	}
	while (!(IFG2&UCA0TXIFG));                								// USCI_A0 TX buffer ready?
	UCA0TXBUF = 'p';									                    // TX -> RXed character
	while (!(IFG2&UCA0TXIFG));                								// USCI_A0 TX buffer ready?
	UCA0TXBUF = '\r';									                    // TX -> RXed character
	__bis_SR_register(LPM0_bits + GIE);										// Enter low power mode to receive the response from the ble module
	character_count = NULL;													// Initialize the count of the number of characters received
	for(i=3; i>=0; i--)														// Iterate untill all 4 characters are received
	{
		BLE_address[i] = received_val[i];									// Store the received values in a variable array
		if (BLE_address[i] >= '0' && BLE_address[i] <= '9')					// To convert the ASCII characters to numbers
		{
			BLE_address[i] -= ASCII_0;										// Subtract ASCII value of '0' from characters if '0' to '9' are received
		}
		else if (BLE_address[i] >= 'A' && BLE_address[i] <= 'F' )
		{
			BLE_address[i] -= ASCII_UP_CASE;								// Subtract 55 from characters if 'A' to 'F' are received
		}
		else if (BLE_address[i] >= 'a' && BLE_address[i] <= 'f' )
		{
			BLE_address[i] -= ASCII_LOW_CASE;								// Subtract 87 from charaters if 'a' to 'f' are received
		}
	}
	ping_flag = NO;															// Reset the ping_flag indicating BLE address are received

	BLE_address_encrypted[0] = (BLE_address[0] << 4) + BLE_address[1];
	BLE_address_encrypted[1] = (BLE_address[2] << 4) + BLE_address[3];
}


void set_duty_cycle(unsigned int Percentage_val)
{
	percent_normalize 	= 100 - percentage_val;			// To make 0% as min and 100% as max
														// (Otherwise 0% would be max and 100% would be min)
	case_value 			= MAX_DUTY * 0.01;						// To convert the value into percentage
	/* Use the equation y = m*x + c */
	if(percentage_val >= 0 && percentage_val <= 10)
	{
		/* Use c = 3300 */
		target_duty = (-24 * percentage_val) + 3300;
	}
	else if(percentage_val >= 15 && percentage_val <= 66)
	{
		/* Use c = 3070 */
		target_duty = (-24 * (percentage_val - 15)) + 3070;
	}
	else if(percentage_val >= 67 && percentage_val < 99)
	{
		/* Use c = 1780 */
		target_duty = (-24 * (percentage_val - 67) + 1780);
	}
	else if(percentage_val == 99)
	{
		target_duty = 0;
	}
	else if(percentage_val == 11)
	{
		target_duty = 3085;
	}
	else if(percentage_val == 12)
	{
		target_duty = 3084;
	}
	else if(percentage_val == 13)
	{
		target_duty = 3080;
	}
	else if(percentage_val == 14)
	{
		target_duty = 3075;
	}
	if(target_duty >= 3300)								// Manually turn off
	{
		isOff = true;
	}
	else
	{
		isOff = false;
	}
	if(sensor_there == true)
	{
		TA0CTL 			|= (TASSEL_2 + MC_1 + ID_3);					// Use clock at 1 MHz
		TA0CCTL0 		|= CCIE;									// Enable Occupancy sensor timer interrupt
		TA0CCR0 		= PWM_WIDTH;
		timer_int_count = NULL;
	}
}

void flash_erase()
{
	char *Flash_ptr;                          										// Flash pointer - sensing freq
	unsigned int address_1 = FLASH_ADDRESS_SENSING_FREQ;

	Flash_ptr = (char *) address_1;

	FCTL1 = FWKEY + ERASE;                    										// Set Erase bit
	FCTL3 = FWKEY;                            										// Clear Lock bit
	*Flash_ptr = 0;                           										// Dummy write to erase Flash segment

	FCTL1 = FWKEY + WRT;                      										// Set WRT bit for write operation

	FCTL1 = FWKEY;                            										// Clear WRT bit
	FCTL3 = FWKEY + LOCK;                     										// Set LOCK bits
}

/*********************************************************************************************************************************
 * Function name			: flash_write()
 * Date         			: 21/6/2017
 * Passing parameters 		: None
 * Returning parameters 	: None
 * Author 					: Suhas K V
 * Description 				: This function writes all critical values like sensing freqency, occupancy sensor time out, host address,
 * 							  commissioning flag status into flash memory. This does not take any parameter nor return any.
 *********************************************************************************************************************************/
void flash_write()
{
	char *Flash_ptr_1;                          									// Flash pointer - sensing freq
	char *Flash_ptr_2;																// Flash pointer - occupancy sensor time-out
	char *Flash_ptr_3;																// Flash pointer - host address
	char *Flash_ptr_6;																// Flash pointer - commissioning flag
	char *Flash_ptr_7;																// Flash pointer - fade rate value
	char *Flash_ptr_8;																// Flash pointer - fade delay value
	char *Flash_ptr_9;																// Flash pointer - power on value
	char *Flash_ptr_10;																// Flash pointer - group one
	char *Flash_ptr_11;																// Flash pointer - group two
	char *Flash_ptr_12;																// Flash pointer - group three
	char *Flash_ptr_13;																// Flash pointer - group four
	char *Flash_ptr_14;																// Flash pointer - group five
	char *Flash_ptr_15;																// Flash pointer - scene number one
	char *Flash_ptr_16;																// Flash pointer - scene number two
	char *Flash_ptr_17;																// Flash pointer - scene number three
	char *Flash_ptr_18;																// Flash pointer - scene number four
	char *Flash_ptr_19;																// Flash pointer - scene number five
	char *Flash_ptr_20;																// Flash pointer - LIS mode
	char *Flash_ptr_21;																// Flash pointer - LIS group number
	char *Flash_ptr_22;																// Flash pointer - polling host address

	unsigned int address_1 = FLASH_ADDRESS_SENSING_FREQ;
	unsigned int address_2 = FLASH_ADDRESS_TIME_OUT;
	unsigned int address_3 = FLASH_HOST_ADDRESS;
	unsigned int address_6 = FLASH_ADDRESS_COM_FLAG;
	unsigned int address_7 = FLASH_FADE_RATE_VALUE;
	unsigned int address_8 = FLASH_FADE_DELAY_VALUE;
	unsigned int address_9 = FLASH_PERCENTAGE_VALUE;
	unsigned int address_10= FLASH_GROUP_ONE;
	unsigned int address_11= FLASH_GROUP_TWO;
	unsigned int address_12= FLASH_GROUP_THREE;
	unsigned int address_13= FLASH_GROUP_FOUR;
	unsigned int address_14= FLASH_GROUP_FIVE;
	unsigned int address_15= FLASH_SCENE_ONE;
	unsigned int address_16= FLASH_SCENE_TWO;
	unsigned int address_17= FLASH_SCENE_THREE;
	unsigned int address_18= FLASH_SCENE_FOUR;
	unsigned int address_19= FLASH_SCENE_FIVE;
	unsigned int address_20= FLASH_LIS_MODE;
	unsigned int address_21= FLASH_LIS_GROUP_NUMBER;
	unsigned int address_22= FLASH_POLLING_HOST_ADDRESS;

	Flash_ptr_1 = (char *) address_1;              									// Initialize Flash pointer
	Flash_ptr_2 = (char *) address_2;
	Flash_ptr_3 = (char *) address_3;
	Flash_ptr_6 = (char *) address_6;
	Flash_ptr_7 = (char *) address_7;
	Flash_ptr_8 = (char *) address_8;
	Flash_ptr_9 = (char *) address_9;
	Flash_ptr_10= (char *) address_10;
	Flash_ptr_11= (char *) address_11;
	Flash_ptr_12= (char *) address_12;
	Flash_ptr_13= (char *) address_13;
	Flash_ptr_14= (char *) address_14;
	Flash_ptr_15= (char *) address_15;
	Flash_ptr_16= (char *) address_16;
	Flash_ptr_17= (char *) address_17;
	Flash_ptr_18= (char *) address_18;
	Flash_ptr_19= (char *) address_19;
	Flash_ptr_20= (char *) address_20;
	Flash_ptr_21= (char *) address_21;
	Flash_ptr_22= (char *) address_22;

	FCTL1 = FWKEY + ERASE;                    										// Set Erase bit
	FCTL3 = FWKEY;                            										// Clear Lock bit
	*Flash_ptr_1 = 0;                           									// Dummy write to erase Flash segment

	FCTL1 = FWKEY + WRT;                      										// Set WRT bit for write operation

	*Flash_ptr_1 = sensing_freq;                   									// Write value to flash
	*Flash_ptr_2 = time_out;
	*Flash_ptr_6 = commissioning_flag;
	*Flash_ptr_7 = fade_rate_val;
	*Flash_ptr_9 = power_on_value;
	*Flash_ptr_10= group_array[0];
	*Flash_ptr_11= group_array[1];
	*Flash_ptr_12= group_array[2];
	*Flash_ptr_13= group_array[3];
	*Flash_ptr_14= group_array[4];

	for(i=NULL; i<4; i++)
	{
		*Flash_ptr_3++ = HOST_address[i];                   						// Write value to flash
	}

	for(i=NULL; i<4; i++)
	{
		*Flash_ptr_22++ = POLLING_HOST_address[i];                   						// Write value to flash
	}

	for(i=NULL; i<2; i++)
	{
		*Flash_ptr_8++ = temp_delay[i];                   							// Write value to flash
	}

	*Flash_ptr_15 = scene_one;
	*Flash_ptr_16 = scene_two;
	*Flash_ptr_17 = scene_three;
	*Flash_ptr_18 = scene_four;
	*Flash_ptr_19 = scene_five;
	*Flash_ptr_20 = lis_mode;
	*Flash_ptr_21 = store_group_value;

	FCTL1 = FWKEY;                            										// Clear WRT bit
	FCTL3 = FWKEY + LOCK;                     										// Set LOCK bits
}

/*********************************************************************************************************************************
 * Function name			: flash_read(unsigned int address)
 * Date         			: 21/6/2017
 * Passing parameters 		: unsigned int address
 * Returning parameters 	: return_val
 * Author 					: Suhas K V
 * Description 				: This function reads all critical values like sensing freqency, occupancy sensor time out, host address,
 * 							  commissioning flag status into flash memory. This does not take any parameter nor return any.
 *********************************************************************************************************************************/
unsigned char flash_read(unsigned int address)
{
	char *Flash_ptr;                          										// Flash pointer
	unsigned char return_val;
	Flash_ptr = (char *) address;              										// Initialize Flash pointer
	return_val = *Flash_ptr;
	return return_val;
}


/*********************************************************************************************************************************
 * Function name			: print_val1(unsigned int Value, unsigned int polling_host)
 * Date         			: 21/6/2017
 * Passing parameters 		: unsigned int Value
 * Returning parameters 	: None
 * Author 					: Subrahmanya K S and Suhas K V
 * Description 				: This function will accept the value to be sent out through BLE and sends it to the destination address.
 **********************************************************************************************************************************/
void print_val1(unsigned int Value, unsigned int polling_host)
{
	for(i=0; i<2; i++)
	{
		while (!(IFG2&UCA0TXIFG));                									// USCI_A0 TX buffer ready?
		UCA0TXBUF = send_msg[i];                    								// TX -> RXed character
	}

	if(polling_host==1)
	{
		for(i=NULL; i<4; i++)
		{
			while (!(IFG2&UCA0TXIFG));                									// USCI_A0 TX buffer ready?
			UCA0TXBUF = POLLING_HOST_address[i];                    							// TX -> RXed character
		}
	}
	else if(polling_host==2)
	{
		for(i=NULL; i<4; i++)
		{
			while (!(IFG2&UCA0TXIFG));                									// USCI_A0 TX buffer ready?
			UCA0TXBUF = HOST_address[i];                    							// TX -> RXed character
		}
	}

	while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
	UCA0TXBUF = ' ';                    											// TX -> RXed character

	for(i=NULL; i<2; i++)
	{
		if(BLE_address_encrypted[i] == 10)
		{
			while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
			UCA0TXBUF = 254;                    											// TX -> RXed character
			char_change_flag |= (0x80 - (i * 0x40));
		}
		else if(BLE_address_encrypted[i] == 13)
		{
			while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
			UCA0TXBUF = 255;                    											// TX -> RXed character
			char_change_flag |= (0x80 - (i * 0x40));
		}
		else
		{
			while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
			UCA0TXBUF = BLE_address_encrypted[i];                    											// TX -> RXed character
		}
	}

	if(Value == 10)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 254;                    											// TX -> RXed character
		char_change_flag |= 0x20;
	}
	else if(Value == 13)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 255;                    											// TX -> RXed character
		char_change_flag |= 0x20;
	}
	else
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = Value;                    											// TX -> RXed character
	}

	checksum = BLE_address_encrypted[0] + BLE_address_encrypted[1] + Value;
	if(checksum == 10)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 254;                    											// TX -> RXed character
		char_change_flag |= 0x10;
	}
	else if(checksum == 13)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 255;                    											// TX -> RXed character
		char_change_flag |= 0x10;
	}
	else
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = checksum;                    											// TX -> RXed character
	}

	checksum = checksum >> 8;
	if(checksum == 10)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 254;                    											// TX -> RXed character
		char_change_flag |= 0x08;
	}
	else if(checksum == 13)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 255;                    											// TX -> RXed character
		char_change_flag |= 0x08;
	}
	else
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = checksum;                    											// TX -> RXed character
	}

	while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
	UCA0TXBUF = char_change_flag;                    											// TX -> RXed character

	while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
	UCA0TXBUF = '\n';
	while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
	UCA0TXBUF = '\r';
	__delay_cycles(1);
	Value = NULL;
	char_change_flag = UNITY;
}

/*********************************************************************************************************************************
 * Function name			: print_val1(unsigned int Value)
 * Date         			: 21/6/2017
 * Passing parameters 		: None
 * Returning parameters 	: None
 * Author 					: Subrahmanya K S and Suhas K V
 * Description 				: This function will send the sensing frequency and occupnacy sensor time out out through BLE and sends
 * 							  it to the destination address.
 **********************************************************************************************************************************/
void print_setting()
{
	for(i=0; i<2; i++)
	{
		while (!(IFG2&UCA0TXIFG));                									// USCI_A0 TX buffer ready?
		UCA0TXBUF = send_msg[i];                    								// TX -> RXed character
	}

	for(i=NULL; i<4; i++)
	{
		while (!(IFG2&UCA0TXIFG));                									// USCI_A0 TX buffer ready?
		UCA0TXBUF = HOST_address[i];                    							// TX -> RXed character
	}

	while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
	UCA0TXBUF = ' ';                    											// TX -> RXed character

	for(i=NULL; i<2; i++)
	{
		if(BLE_address_encrypted[i] == 10)
		{
			while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
			UCA0TXBUF = 254;                    											// TX -> RXed character
			char_change_flag |= (0x80 - (i * 0x40));
		}
		else if(BLE_address_encrypted[i] == 13)
		{
			while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
			UCA0TXBUF = 255;                    											// TX -> RXed character
			char_change_flag |= (0x80 - (i * 0x40));
		}
		else
		{
			while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
			UCA0TXBUF = BLE_address_encrypted[i];                    											// TX -> RXed character
		}
	}

	time_out = max_timer_count / _1_MIN;											// Calculate the time out value

	if(time_out == 10)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 254;                    											// TX -> RXed character
		char_change_flag |= 0x20;
	}
	else if(time_out == 13)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 255;                    											// TX -> RXed character
		char_change_flag |= 0x20;
	}
	else
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = time_out;                    											// TX -> RXed character
	}

	if(power_on_value == 10)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 254;                    											// TX -> RXed character
		char_change_flag |= 0x10;
	}
	else if(power_on_value == 13)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 255;                    											// TX -> RXed character
		char_change_flag |= 0x10;
	}
	else
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = power_on_value;                    											// TX -> RXed character
	}

	while (!(IFG2&UCA0TXIFG));                											// USCI_A0 TX buffer ready?
	UCA0TXBUF = fade_rate_val;                    										// TX -> RXed character

	for(i=NULL; i<2; i++)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = temp_delay[i];                    									// TX -> RXed character
	}

	checksum = BLE_address_encrypted[0] +
					BLE_address_encrypted[1] +
						time_out + power_on_value + fade_rate_val +
							temp_delay[0] + temp_delay[1];
	if(checksum == 10)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 254;                    											// TX -> RXed character
		char_change_flag |= 0x08;
	}
	else if(checksum == 13)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 255;                    											// TX -> RXed character
		char_change_flag |= 0x08;
	}
	else
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = checksum;                    											// TX -> RXed character
	}

	checksum = checksum >> 8;
	if(checksum == 10)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 254;                    											// TX -> RXed character
		char_change_flag |= 0x04;
	}
	else if(checksum == 13)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 255;                    											// TX -> RXed character
		char_change_flag |= 0x04;
	}
	else
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = checksum;                    											// TX -> RXed character
	}

	while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
	UCA0TXBUF = char_change_flag;                    											// TX -> RXed character

	while (!(IFG2&UCA0TXIFG));                											// USCI_A0 TX buffer ready?
	UCA0TXBUF = '\n';
	while (!(IFG2&UCA0TXIFG));                											// USCI_A0 TX buffer ready?
	UCA0TXBUF = '\r';
	__delay_cycles(1);
	char_change_flag = UNITY;
}

/*********************************************************************************************************************************
 * Function name			: print_address(unsigned int isHostAddress)
 * Date         			: 21/6/2017
 * Passing parameters 		: unsigned int isHostAddress
 * Returning parameters 	: None
 * Author 					: Subrahmanya K S and Suhas K V
 * Description 				: This function will accept the address parameter. Based on the parameter it will send the required
 * 							  address through BLE.
 **********************************************************************************************************************************/
void print_address(unsigned int isHostAddress)
{
	for(i=0; i<2; i++)
	{
		while (!(IFG2&UCA0TXIFG));                									// USCI_A0 TX buffer ready?
		UCA0TXBUF = send_msg[i];                    								// TX -> RXed character
	}

	if(isHostAddress == 1)
	{
		for(i=NULL; i<4; i++)
		{
			while (!(IFG2&UCA0TXIFG));                									// USCI_A0 TX buffer ready?
			UCA0TXBUF = POLLING_HOST_address[i];                    							// TX -> RXed character
		}
	}
	else if(isHostAddress == 2)
	{
		for(i=NULL; i<4; i++)
		{
			while (!(IFG2&UCA0TXIFG));                									// USCI_A0 TX buffer ready?
			UCA0TXBUF = HOST_address[i];                    							// TX -> RXed character
		}
	}

	while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
	UCA0TXBUF = ' ';                    											// TX -> RXed character

	for(i=NULL; i<2; i++)
	{
		if(BLE_address_encrypted[i] == 10)
		{
			while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
			UCA0TXBUF = 254;                    											// TX -> RXed character
			char_change_flag |= (0x80 - (i * 0x40));
		}
		else if(BLE_address_encrypted[i] == 13)
		{
			while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
			UCA0TXBUF = 255;                    											// TX -> RXed character
			char_change_flag |= (0x80 - (i * 0x40));
		}
		else
		{
			while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
			UCA0TXBUF = BLE_address_encrypted[i];                    											// TX -> RXed character
		}
	}

	while (!(IFG2&UCA0TXIFG));              	 				 				// USCI_A0 TX buffer ready?
	UCA0TXBUF = DEVICE_TYPE;				                    				// TX -> RXed character

	checksum = BLE_address_encrypted[0]
				+ BLE_address_encrypted[1] + DEVICE_TYPE;
	if(checksum == 10)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 254;                    											// TX -> RXed character
		char_change_flag |= 0x10;
	}
	else if(checksum == 13)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 255;                    											// TX -> RXed character
		char_change_flag |= 0x10;
	}
	else
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = checksum;                    											// TX -> RXed character
	}
	checksum = checksum >> 8;
	if(checksum == 10)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 254;                    											// TX -> RXed character
		char_change_flag |= 0x08;
	}
	else if(checksum == 13)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 255;                    											// TX -> RXed character
		char_change_flag |= 0x08;
	}
	else
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = checksum;                    											// TX -> RXed character
	}

	while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
	UCA0TXBUF = char_change_flag;                    											// TX -> RXed character
	while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
	UCA0TXBUF = '\n';
	while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
	UCA0TXBUF = '\r';
	char_change_flag = UNITY;
}

/*********************************************************************************************************************************
 * Function name			: print_char(unsigned int character)
 * Date         			: 21/6/2017
 * Passing parameters 		: unsigned int character
 * Returning parameters 	: None
 * Author 					: Subrahmanya K S and Suhas K V
 * Description 				: This function will accept the address parameter. Based on the parameter it will send the required
 * 							  address through BLE.
 **********************************************************************************************************************************/
void print_char(unsigned char character)
{
	for(i=0; i<2; i++)
	{
		while (!(IFG2&UCA0TXIFG));               			 						// USCI_A0 TX buffer ready?
		UCA0TXBUF = send_msg[i];                    								// TX -> RXed character
	}

	for(i=NULL; i<4; i++)
	{
		while (!(IFG2&UCA0TXIFG));                									// USCI_A0 TX buffer ready?
		UCA0TXBUF = HOST_address[i];                    							// TX -> RXed character
	}

	while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
	UCA0TXBUF = ' ';                    											// TX -> RXed character

	for(i=NULL; i<2; i++)
	{
		if(BLE_address_encrypted[i] == 10)
		{
			while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
			UCA0TXBUF = 254;                    											// TX -> RXed character
			char_change_flag |= (0x80 - (i * 0x40));
		}
		else if(BLE_address_encrypted[i] == 13)
		{
			while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
			UCA0TXBUF = 255;                    											// TX -> RXed character
			char_change_flag |= (0x80 - (i * 0x40));
		}
		else
		{
			while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
			UCA0TXBUF = BLE_address_encrypted[i];                    											// TX -> RXed character
		}
	}

	while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
	UCA0TXBUF = character;                    										// TX -> RXed character

	checksum = BLE_address_encrypted[0] + BLE_address_encrypted[1] + character;
	if(checksum == 10)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 254;                    											// TX -> RXed character
		char_change_flag |= 0x10;
	}
	else if(checksum == 13)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 255;                    											// TX -> RXed character
		char_change_flag |= 0x10;
	}
	else
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = checksum;                    											// TX -> RXed character
	}

	checksum = checksum >> 8;
	if(checksum == 10)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 254;                    											// TX -> RXed character
		char_change_flag |= 0x08;
	}
	else if(checksum == 13)
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = 255;                    											// TX -> RXed character
		char_change_flag |= 0x08;
	}
	else
	{
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = checksum;                    											// TX -> RXed character
	}

	while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
	UCA0TXBUF = char_change_flag;                    											// TX -> RXed character

	while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
	UCA0TXBUF = '\n';
	while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
	UCA0TXBUF = '\r';
	__delay_cycles(1);
	char_change_flag = UNITY;
}

void print_command(int lis_value)
{
	for(i=0; i<2; i++)
	{
		while (!(IFG2&UCA0TXIFG));                									// USCI_A0 TX buffer ready?
		UCA0TXBUF = send_msg[i];                    								// TX -> RXed character
	}

	for(i=NULL; i<4; i++)
	{
		while (!(IFG2&UCA0TXIFG));                									// USCI_A0 TX buffer ready?
		UCA0TXBUF = '0';
	}

	while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
	UCA0TXBUF = ' ';                    											// TX -> RXed character

	if(lis_value == 0)
	{
		for(i=NULL; i<5; i++)
		{
			while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
			UCA0TXBUF = LIS_command[i] ;                    											// TX -> RXed character
		}
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = '0';                    											// TX -> RXed character

		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = '0';                    											// TX -> RXed character

		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = store_group_value;                    											// TX -> RXed character
	}
	else if(lis_value == 1)
	{
		for(i=NULL; i<5; i++)
		{
			while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
			UCA0TXBUF = LIS_command[i] ;                    											// TX -> RXed character
		}
		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = '9';                    											// TX -> RXed character

		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = '9';                    											// TX -> RXed character

		while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
		UCA0TXBUF = store_group_value;                    											// TX -> RXed character
	}

	while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
	UCA0TXBUF = '\n';
	while (!(IFG2&UCA0TXIFG));                										// USCI_A0 TX buffer ready?
	UCA0TXBUF = '\r';                    											// TX -> RXed character

}

/*********************************************************************************************************************************
 * Function name			: CLOCK_INIT()
 * Date         			: 19/4/2017
 * Passing parameters 		: None
 * Returning parameters 	: None
 * Author 					: Suhas K V
 * Description 				: This function is used to intialise BCS clock and DCO clock. Frequency of clock is set to 8MHz.
 **********************************************************************************************************************************/
void CLOCK_INIT()
{
	if (CALBC1_8MHZ==0xFF)															// If calibration constant erasedd
	{
		while(true);                               									// do not load, trap CPU!!
	}
	DCOCTL = 0;                               										// Select lowest DCOx and MODx settings
	BCSCTL1 = CALBC1_8MHZ;                    										// Set DCO
	DCOCTL = CALDCO_8MHZ;
}

/*********************************************************************************************************************************
 * Function name			: UART_INIT()
 * Date         			: 21/6/2017
 * Passing parameters 		: None
 * Returning parameters 	: None
 * Author 					: Suhas K V
 * Description 				: This function is used to intialise UART peripherals of the microcontroller. Baud rate is set to 115200.
 * 							  Also UART interrupt will be enabled.
 **********************************************************************************************************************************/
void UART_INIT()
{
	P1SEL |= (BIT1 + BIT2) ;                     									// P1.1 = RXD, P1.2=TXD
	P1SEL2 |= (BIT1 + BIT2) ;                    									// P1.1 = RXD, P1.2=TXD
	UCA0CTL1 |= UCSSEL_2;                     										// SMCLK
	UCA0BR0 = LOWER_BAUD;                            								// 8MHz 115200
	UCA0BR1 = HIGHER_BAUD;                              							// 8MHz 115200
	UCA0MCTL = UCBRS0;                        										// Modulation UCBRSx = 1
	UCA0CTL1 &= ~UCSWRST;                     										// **Initialize USCI state machine**
	IE2 |= UCA0RXIE;                          										// Enable USCI_A0 RX interrupt
}

/*********************************************************************************************************************************
 * Function name			: TIMER_INIT()
 * Date         			: 21/6/2017
 * Passing parameters 		: None
 * Returning parameters 	: None
 * Author 					: Suhas K V
 * Description 				: This function is used to intialise TIMER0 peripherals of the microcontroller. Timer for occupancy
 * 							  sensor is initialised and interrupt is enabled.
 **********************************************************************************************************************************/
void TIMER_INIT()
{
	/* Occupancy sensor timer */
	TA0CTL |= (TASSEL_2 + MC_1 + ID_3);												// Use clock at 1 MHz
	TA0CCTL0 |= CCIE;																// Enable Occupancy sensor timer interrupt
	TA0CCR0 = PWM_WIDTH;															// Value for 300 Hz PWM frequency
	TA0CCR1 = NULL;
	TA0CCTL1 = OUTMOD_3;
}

/*********************************************************************************************************************************
 * Function name			: TIMER_DISABLE()
 * Date         			: 21/6/2017
 * Passing parameters 		: None
 * Returning parameters 	: None
 * Author 					: Suhas K V
 * Description 				: This function is used to disable TIMER0 peripherals of the microcontroller. Disables the occupancy sensor
 * 							  timer.
 **********************************************************************************************************************************/
void TIMER_DISABLE()
{
	TA0CTL &= ~MC_2;																// Stop Timer 0 clock
	TA0CCTL0 &= ~CCIE;																// Disable Occupancy sensor timer interrupt
}

/*********************************************************************************************************************************
 * Function name			: REQUEST_MODE_TIMER_INIT()
 * Date         			: 21/6/2017
 * Passing parameters 		: None
 * Returning parameters 	: None
 * Author 					: Suhas K V
 * Description 				: This function is used to intialise TIMER1 peripherals of the microcontroller. Timer for
 * 							  Request mode is initialised and interrupt is enabled.
 **********************************************************************************************************************************/
void REQUEST_MODE_TIMER_INIT()
{
	TA1CTL 	|= (TASSEL_2 + MC_2);													// Use clock at 8 MHz
	TA1CCTL1 |= CCIE;																// Enable request mode timer interrupt
	TA1CCR1 = 0xFFFF;
}

/* Disable request mode timer */
/*********************************************************************************************************************************
 * Function name			: TIMER_INIT()
 * Date         			: 19/4/2017
 * Passing parameters 		: None
 * Returning parameters 	: None
 * Author 					: Suhas K V
 * Description 				: This function is used to disable TIMER1 peripherals of the microcontroller. Request mode timer is disabled
 **********************************************************************************************************************************/
void REQUEST_MODE_TIMER_DISABLE()
{
	TA1CCTL1 &= ~CCIE;																// Disable request mode timer interrupt
}

/*********************************************************************************************************************************
 * Function name			: PORT_INIT()
 * Date         			: 21/6/2017
 * Passing parameters 		: None
 * Returning parameters 	: None
 * Author 					: Suhas K V
 * Description 				: This function is used to intialise GPIOs of the microcontroller. This includes Occupancy sensor, switch
 * 							  and relay.
 **********************************************************************************************************************************/
void PORT_INIT()
{
	P2DIR &= ~BIT0;																	// Configure port at input
	P2IE |=  BIT0;                           	 									// P2.0 interrupt enabled
	P2IES |= BIT0;                            										// P2.0 Hi/lo edge
	P2REN |= BIT0;							  										// Enable Pull resistor on SW1 (P2.0)
	P2IFG &= ~BIT0;                           										// P2.0 IFG cleared
	P2OUT |= BIT0;																	// Enable pull up on P2.0
}

void PIR_INIT()
{
	P1DIR &= ~BIT5;																	// Occupancy sensor input
	P1IE |= BIT5;
	P1IFG &= ~BIT5;
}

void PORT_OUT_INIT()
{
	P2DIR |= BIT2;																	// Smart Occupancy Sensor relay output
	P2OUT |= BIT2;																	// Switch ON load

	P2DIR |= BIT6;
	P2SEL &= ~(BIT6 + BIT7);
	P2SEL |= BIT6;
}

void ADC_INIT()
{
	ADC10CTL0 = ADC10SHT_2 + ADC10ON + ADC10IE; // ADC10ON, interrupt enabled
	ADC10CTL1 = INCH_5;                       // input A1
	ADC10AE0 |= BIT5;                         // PA.1 ADC option select
}

void ADC_UNINIT()
{
	ADC10CTL0 &= ~(ADC10SHT_2 + ADC10ON + ADC10IE); // ADC10ON, interrupt enabled
	ADC10CTL1 &= ~INCH_5;                       // input A1
	ADC10AE0 &= ~BIT5;                         // PA.1 ADC option select
}

/*********************************************************************************************************************************
 * Function name			: FLASH_INIT()
 * Date         			: 21/6/2017
 * Passing parameters 		: None
 * Returning parameters 	: None
 * Author 					: Suhas K V
 * Description 				: This function is used to intialise flash memory of microcontroller.
 **********************************************************************************************************************************/
void FLASH_INIT()
{
	FCTL2 = FWKEY + FSSEL_2 + (FN0 + FN1 + FN2 + FN4);             										// MCLK/3 for Flash Timing Generator
}


void SYS_INIT()
{
	if(flash_read(FLASH_GROUP_ONE) != 0xff)
	{
		group_array[0] = flash_read(FLASH_GROUP_ONE);
	}
	if(flash_read(FLASH_GROUP_TWO) != 0xff)
	{
		group_array[1] = flash_read(FLASH_GROUP_TWO);
	}
	if(flash_read(FLASH_GROUP_THREE) != 0xff)
	{
		group_array[2] = flash_read(FLASH_GROUP_THREE);
	}
	if(flash_read(FLASH_GROUP_FOUR) != 0xff)
	{
		group_array[3] = flash_read(FLASH_GROUP_FOUR);
	}
	if(flash_read(FLASH_GROUP_FIVE) != 0xff)
	{
		group_array[4] = flash_read(FLASH_GROUP_FIVE);
	}

	if(flash_read(FLASH_HOST_ADDRESS) != 0xff)								// Read the host address from the flash memory if something is written in it
	{
		HOST_address[0] = flash_read(FLASH_HOST_ADDRESS_1);
		HOST_address[1] = flash_read(FLASH_HOST_ADDRESS_2);
		HOST_address[2] = flash_read(FLASH_HOST_ADDRESS_3);
		HOST_address[3] = flash_read(FLASH_HOST_ADDRESS_4);
	}

	if(flash_read(FLASH_POLLING_HOST_ADDRESS) != 0xff)								// Read the host address from the flash memory if something is written in it
	{
		POLLING_HOST_address[0] = flash_read(FLASH_POLLING_HOST_ADDRESS_1);
		POLLING_HOST_address[1] = flash_read(FLASH_POLLING_HOST_ADDRESS_2);
		POLLING_HOST_address[2] = flash_read(FLASH_POLLING_HOST_ADDRESS_3);
		POLLING_HOST_address[3] = flash_read(FLASH_POLLING_HOST_ADDRESS_4);
	}

	if(flash_read(FLASH_ADDRESS_SENSING_FREQ) != 0xff)						// Read the sensing frequency from the flash memory if something is written in it
	{
		sensing_freq = flash_read(FLASH_ADDRESS_SENSING_FREQ);
	}
	sensing_freq_val = (sensing_freq * 1000000)/8192;							// Calculate sensing frequency


	if(((flash_read(FLASH_ADDRESS_TIME_OUT)) != 0xff) 						// Read the time out value of the occupancy sensor from the flash memory is something is
			&& ((flash_read(FLASH_ADDRESS_TIME_OUT)) != 0x00))				// written in it and if that value is not zero
	{
		time_out = flash_read(FLASH_ADDRESS_TIME_OUT);
		max_timer_count = time_out * _1_MIN;							// Convert the value in minutes
	}

	if(flash_read(FLASH_ADDRESS_COM_FLAG) != 0xff)							// Check if the sensor has been commissioned, if it is then the commissioning flag will be 1
	{
		commissioning_flag = flash_read(FLASH_ADDRESS_COM_FLAG);
	}

	if(((flash_read(FLASH_FADE_RATE_VALUE) != 0xff))					// Something's written in the flash memory
			&& ((flash_read(FLASH_FADE_RATE_VALUE)) != 0x00))
	{
		fade_rate_val = flash_read(FLASH_FADE_RATE_VALUE);
	}

	if((flash_read(FLASH_PERCENTAGE_VALUE)) != 0xff)			// Something's written in the flash memory
	{
		power_on_value = flash_read(FLASH_PERCENTAGE_VALUE);
		percentage_val = power_on_value;
		percent_normalize = 100 - power_on_value;			// To make 0% as min and 100% as max
		// (Otherwise 0% would be max and 100% would be min)
		case_value = MAX_DUTY * 0.01;						// To convert the value into percentage
		target_duty = case_value * percent_normalize;
		if(target_duty >= 3300)
		{
			isOff = 1;
		}
	}

	if(flash_read(FLASH_FADE_DELAY_VALUE) != 0xff)								// Read the host address from the flash memory if something is written in it
	{
		temp_delay[0] = flash_read(FLASH_FADE_DELAY_VALUE_1);
		temp_delay[1] = flash_read(FLASH_FADE_DELAY_VALUE_2);

		delay_value = ((temp_delay[0] * 10) + (temp_delay[1] * 1)) * 10;
		if(delay_value == 0)
		{
			delay_value = 300;
			temp_delay[0] = 3;
			temp_delay[1] = 0;
		}
	}

	/* Group values and scene numbers are to be read from flash here */
	if(flash_read(FLASH_SCENE_ONE) != 0xff)
	{
		scene_one = flash_read(FLASH_SCENE_ONE);
	}
	if(flash_read(FLASH_SCENE_TWO) != 0xff)
	{
		scene_two = flash_read(FLASH_SCENE_TWO);
	}
	if(flash_read(FLASH_SCENE_THREE) != 0xff)
	{
		scene_three = flash_read(FLASH_SCENE_THREE);
	}
	if(flash_read(FLASH_SCENE_FOUR) != 0xff)
	{
		scene_four = flash_read(FLASH_SCENE_FOUR);
	}
	if(flash_read(FLASH_SCENE_FIVE) != 0xff)
	{
		scene_five = flash_read(FLASH_SCENE_FIVE);
	}

	if(flash_read(FLASH_LIS_GROUP_NUMBER) != 0xff)
	{
		store_group_value = flash_read(FLASH_LIS_GROUP_NUMBER);
	}

	if(flash_read(FLASH_LIS_MODE) != 0xff)
	{
		lis_mode 			= flash_read(FLASH_LIS_MODE);
	}
}

/*********************************************************************************************************************************
 * Interrupt name			: USCI0RX_ISR()
 * Date         			: 21//62017
 * Passing parameters 		: None
 * Returning parameters 	: None
 * Author 					: Suhas K V
 * Description 				: This interrupt service routine is invoked when THE BLE module sends the data through UART. Once it
 * 							  reads the command it wakes up the CPU.
 **********************************************************************************************************************************/
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
	received_char = UCA0RXBUF;														// Store the received character in a variable
	if (received_char == 0 || received_char == 13 || received_char == 10			// Filter the junk and invalid characters
													|| received_char == 255 )
	{
		IFG2 &= ~UCA0RXIFG;
		return;
	}
	else
	{
		received_val[character_count] = received_char;								// Put all the characters in an array
		IFG2 &= ~UCA0RXIFG;															// Clear the receive interrupt flag
		character_count++;															// Increase the character count index
	}
	if(character_count == UNITY && ping_flag == 0) {								// Start UART timer to receive BLE address
		TA1CTL 	|= (TASSEL_2 + MC_2);													// Use clock at 8 MHz
		TA1CCTL0 |= CCIE;
		TA1CCR0 = 0xFFFF;
	}
	if(ping_flag == 1 && character_count == 4)										// If all 4 characters of BLE address are received then go to main program
	{
		__bic_SR_register_on_exit(LPM0_bits);
	}
	else if (character_count >= MAX_CHAR)											// If all 7 characters are received, then go to main program
	{
		__bic_SR_register_on_exit(LPM0_bits);
	}
}

/*********************************************************************************************************************************
 * Interrupt name			: Timer_A ()
 * Date         			: 21/6/2017
 * Passing parameters 		: None
 * Returning parameters 	: None
 * Author 					: Suhas K V
 * Description 				: This interrupt service routine is invoked when the timer timeout happens. It will check whether the
 * 							  occupancy time out is over. If yes, wake up the CPU.
 **********************************************************************************************************************************/
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
	timer_int_count++;																// increment timer entry count
	if (timer_int_count >= max_timer_count)
	{
		time_out_flag = YES;														// if count is more than the time out value, set the flag
 		timer_int_count = NULL;
		__bic_SR_register_on_exit(LPM0_bits);										// wake up MCU from sleep mode
	}
}

/*********************************************************************************************************************************
 * Interrupt name			: Timer_A2 ()
 * Date         			: 21/6/2017
 * Passing parameters 		: None
 * Returning parameters 	: None
 * Author 					: Suhas K V
 * Description 				: This interrupt service routine is invoked when there is UART command is received. It checks for the
 * 							  corrctness of UART data.
 **********************************************************************************************************************************/
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer_A2 (void)
{
	timer_count++;
	if(timer_count > 122)																// If no character is received after 2s then clear the character count
	{
		TA1CCTL0 &= ~CCIE;															// Clear the timer interrupt
		timer_count = NULL;															// Clear the timer count
		character_count = NULL;														// Clear the character count
		command_index_match = 255;
		for (i = 0; i<10 ; i++)
		{
			junk = UCA0RXBUF;
		}
		IFG2 &= ~UCA0RXIFG;
		IE2 |= UCA0RXIE;
	}
}

/*********************************************************************************************************************************
 * Interrupt name			: Timer_A ()
 * Date         			: 21/6/2017
 * Passing parameters 		: None
 * Returning parameters 	: None
 * Author 					: Suhas K V
 * Description 				: This interrupt service routine is invoked when the timer timeout happens. It will check whether the
 * 							  sensing time is over. If yes, wake up the CPU.
 **********************************************************************************************************************************/
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer_A1 (void)
{
	switch(TA1IV)
	{
	case 2:
		timer_count_1++;
		if(timer_count_1 > sensing_freq_val)										// Upon completition of 15 seconds
		{
			timer_flag = YES;														// Set the timer flag
			timer_count_1 = NULL;													// Clear the timer count
			__bic_SR_register_on_exit(LPM0_bits);
		}
		break;

	default:
		break;
	}
}

/*********************************************************************************************************************************
 * Interrupt name			: PORT_1 ()
 * Date         			: 21/6/2017
 * Passing parameters 		: None
 * Returning parameters 	: None
 * Author 					: Suhas K V
 * Description 				: This interrupt service routine is invoked when the occupancy is detected.
 **********************************************************************************************************************************/
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
	__delay_cycles(8000000);														// 1 second debounce delay

	if (P1IFG & BIT5)																// if Motion detected
	{
		pir_count++;																// increment count
		if (pir_count >= MAX_PIR_COUNT)												//check for spikes - false triggering
		{
			if(isOff == false)														// if load is not turned off manually
			{
				P2OUT |= BIT2;
				if(pirOff == true)
				{
					lis_on_send_flag = true;
					pirOff = false;
				}
			}
			/* Initialise occupancy sensor timer */
			TA0CTL |= (TASSEL_2 + MC_1 + ID_3);										// Use clock at 1 MHz
			TA0CCTL0 |= CCIE;														// Enable Occupancy sensor timer interrupt
			TA0CCR0 = PWM_WIDTH;
			timer_int_count = NULL;
			pir_flag = YES;
			pir_count = NULL;														// Clear counter
		}
	}
	P2IFG &= ~BIT0;
	P1IFG &= ~BIT5;
	__bic_SR_register_on_exit(LPM0_bits);											// Clear LPM0 on exit
}
