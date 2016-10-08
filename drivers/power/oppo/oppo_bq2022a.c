/*******************************************************************************
* Copyright (c)  2014- 2014  Guangdong OPPO Mobile Telecommunications Corp., Ltd
* CONFIG_MACH_OPPO
* Description: Source file for CBufferList.
*           To allocate and free memory block safely.
* Version   : 0.0
* Date      : 2014-07-30
* Author    : Lijiada @Bsp.charge
* ---------------------------------- Revision History: -------------------------
* <version>           <date>          < author >              <desc>
* Revision 0.0        2014-07-30      Lijiada @Bsp.charge
* Modified to be suitable to the new coding rules in all functions.
*******************************************************************************/

#define OPPO_BQ2022A_PAR
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/qpnp/qpnp-adc.h>
//#include <mach/oppo_boot_mode.h>
//#include <linux/boot_mode.h>
#include <soc/oppo/boot_mode.h>
#include <soc/oppo/device_info.h>
#include "oppo_inc.h"


#define DEVICE_BATTERY_ID_VERSION		"1.0"
#define DEVICE_BATTERY_ID_TYPE_SONY		"SONY"
#define DEVICE_BATTERY_ID_TYPE_ATL		"ATL"
#define DEVICE_BATTERY_ID_TYPE_SDI		"SDI"
#define DEVICE_BATTERY_ID_TYPE_LG		"LG"


#define OPPO_BATTERY_ENCRPTION
static bool oppo_high_battery_status = 1;
static int oppo_check_ID_status = 0;
static bool oppo_battery_status_init_flag = 0;
//int Gpio_BatId_Init(void);
void CheckIDCompare(void);

#define DEBUG_BQ2202A
#define READ_PAGE_BQ2202A

//#define BQ2202A_GPIO				53
static int bq2202a_gpio = 0;
#define GPIO_DIR_OUT_1				1
#define GPIO_DIR_OUT_0				0



#define READ_ID_CMD					0x33            // read ROM
#define SKIP_ROM_CMD               	0xCC           // skip ROM
#define WRITE_EPROM_CMD        		0x0F           // write EPROM
#define READ_PAGE_ID_CMD        	0xF0          // read EPROM  PAGE
#define READ_FIELD_ID_CMD       	0xC3          // read EPROM  FIELD

#ifdef READ_PAGE_BQ2202A
#define AddressLow                   		0x20        // EPROM start address LOW
#define AddressHigh                  		0x00        // EPROM start address  HIGH
#define BQ2022_MANUFACTURE_ADDR_LOW			0x40
#define BQ2022_MANUFACTURE_ADDR_HIGH		0x00
#else
#define AddressLow                   	0x00        // EPROM start address LOW
#define AddressHigh                  	0x00        // EPROM start address  HIGH
#define BQ2022_MANUFACTURE_ADDR_LOW		0x00
#define BQ2022_MANUFACTURE_ADDR_HIGH		0x00
#endif

static  unsigned char ReadIDDataByte[8];     //8*8=64bit            ID ROM
#ifdef READ_PAGE_BQ2202A
static  unsigned char CheckIDDataByte[32];    // 32*8=256bit   EPROM  PAGE1
#else
static  unsigned char CheckIDDataByte[128];    // 128*8=1024bit   EPROM  PAGE1
#endif



static DEFINE_MUTEX(bq2202a_access);
/**********************************************************************/
/* 		void wait_us(int usec)										  */
/*																      */
/*	Description :   Creates a delay of approximately (usec + 5us) 	  */
/*				  		when usec is greater.						  */
/* 	Arguments : 		None										  */
/*	Global Variables:	None   										  */
/*  Returns: 			None								          */
/**********************************************************************/
#if 0
void wait_us(int usec)
{
    while (usec--);
}
#else
#define wait_us(n) udelay(n)
#define wait_ms(n) mdelay(n)
#endif

static int Gpio_BatId_Init(void)
{
	int rc = 0;
	if(gpio_is_valid(bq2202a_gpio))
	{
		rc = gpio_request(bq2202a_gpio,"batid_bq2202a");
		//pr_err("Gpio_BatId_Init,gpio_request batid_bq2202a\r\n");
		if(rc)
		{
			pr_err("unable to request gpio batid_bq2202a\r\n");
			return rc;
		}
	}
	else
	{
		pr_err("Gpio_BatId_Init,bq2202a_gpio is not valid\r\n");
		rc = -1;
	}
	return rc;
}
/**********************************************************************/
/* 	static void SendReset(void)										  */
/*																      */
/*	Description : 		Creates the Reset signal to initiate SDQ 	  */
/*				  		communication.								  */
/* 	Arguments : 		None										  */
/*	Global Variables:	None   										  */
/*  Returns: 			None								          */
/**********************************************************************/
static void SendReset(void)
{

    gpio_direction_output(bq2202a_gpio, GPIO_DIR_OUT_1);	            //Set High
    wait_us(20);													//Allow PWR cap to charge and power IC	~ 25us
    gpio_direction_output(bq2202a_gpio, GPIO_DIR_OUT_0);           	//Set Low
    wait_us(650);								//Reset time greater then 480us
    //mt_set_gpio_out(bq2202a_gpio, 1);            	//Set High
    gpio_direction_input(bq2202a_gpio);		//Set GPIO P9.3 as Input
}

/**********************************************************************/
/* 	static unsigned char TestPresence(void)							  */
/*																      */
/*	Description : 		Detects if a device responds to Reset signal  */
/* 	Arguments : 		PresenceTimer - Sets timeout if no device	  */
/*							present									  */
/*						InputData - Actual state of GPIO			  */
/*						GotPulse - States if a pulse was detected	  */
/*	Global Variables:	None   										  */
/*  Returns: 			GotPulse         							  */
/**********************************************************************/
static unsigned char TestPresence(void)
{
    unsigned int PresenceTimer;
    static volatile unsigned char InputData;
    static volatile unsigned char GotPulse;

    gpio_direction_input(bq2202a_gpio);	        //Set GPIO P9.3 as Input
    PresenceTimer = 300;                                                //Set timeout, enough time to allow presence pulse
    GotPulse = 0;                                                           //Initialize as no pulse detected
	wait_us(60);
    while ((PresenceTimer > 0) && (GotPulse == 0))
    {
        InputData = gpio_get_value(bq2202a_gpio);       //Monitor logic state of GPIO
		/*int j = 0;
		while(j < 10)
		{
			printk("mt_get_gpio_in---------------InputData = %d\r\n", InputData);
			j++;
			wait_us(100);

		}*/
        if (InputData == 0)                                             //If GPIO is Low,
        {
            GotPulse = 1;                                               //it means that device responded
        }
        else                                                                //If GPIO is high
        {
            GotPulse = 0;			                            //it means that device has not responded
            --PresenceTimer;		                            //Decrease timeout until enough time has been allowed for response
        }
    }
    wait_us(200);					                    //Wait some time to continue SDQ communication
    #ifdef DEBUG_BQ2202A
    //printk("PresenceTimer=%d\n",PresenceTimer);
    #endif
    return GotPulse;				                    //Return if device detected or not
}

/**********************************************************************/
/* 	static void WriteOneBit(unsigned char OneZero)					  */
/*																      */
/*	Description : 		This procedure outputs a bit in SDQ to the 	  */
/*				  		slave.								  		  */
/* 	Arguments : 		OneZero - value of bit to be written		  */
/*	Global Variables:	None   										  */
/*  Returns: 			None								          */
/**********************************************************************/
static void WriteOneBit(unsigned char OneZero)
{
	//wait_us(300);
	gpio_direction_output(bq2202a_gpio, GPIO_DIR_OUT_1);			//Set GPIO P9.3 as Output
    //mt_set_gpio_out(bq2202a_gpio, 1);		            //Set High
     gpio_direction_output(bq2202a_gpio, GPIO_DIR_OUT_0);	                //Set Low

     //printk("WriteOneBit----1------OneZero = %d\t\n", OneZero);
    if (OneZero != 0x00)
    {
        wait_us(7);									//approximately 7us	for a Bit 1
        gpio_direction_output(bq2202a_gpio, GPIO_DIR_OUT_1);	            //Set High
        wait_us(65);								//approximately 65us
    }
    else
    {
        wait_us(65);								//approximately 65us for a Bit 0
        gpio_direction_output(bq2202a_gpio, GPIO_DIR_OUT_1);	            //Set High
        wait_us(7);					   				//approximately 7us
    }
    wait_us(5);	  									//approximately 5us
}

/**********************************************************************/
/* 	static void WriteOneByte(unsigned char Data2Send)				  */
/*																      */
/*	Description : 		This procedure calls the WriteOneBit() 		  */
/*				  		function 8 times to send a byte in SDQ.		  */
/* 	Arguments : 		Data2Send - Value of byte to be sent in SDQ	  */
/*	Global Variables:	None   										  */
/*  Returns: 			None								          */
/**********************************************************************/
static void WriteOneByte(unsigned char Data2Send)
{
    unsigned char i;
    unsigned char MaskByte;
    unsigned char Bit2Send;

    MaskByte = 0x01;

    for (i = 0; i < 8; i++)
    {
        Bit2Send = Data2Send & MaskByte;		//Selects the bit to be sent
        WriteOneBit(Bit2Send);					//Writes the selected bit
        MaskByte <<= 1;							//Moves the bit mask to the next most significant position
    }
}

/**********************************************************************/
/* 	static unsigned char ReadOneBit(void)							  */
/*																      */
/*	Description : 		This procedure receives the bit value returned*/
/*				  		by the SDQ slave.							  */
/* 	Arguments : 		InBit - Bit value returned by slave			  */
/*	Global Variables:	None   										  */
/*  Returns: 			InBit								          */
/**********************************************************************/
static unsigned char ReadOneBit(void)
{
    static unsigned char InBit;

    gpio_direction_output(bq2202a_gpio, GPIO_DIR_OUT_1);			//Set GPIO P9.3 as Output
													            //Set High
    gpio_direction_output(bq2202a_gpio, GPIO_DIR_OUT_0);	                //Set Low
	gpio_direction_input(bq2202a_gpio);			//Set GPIO P9.3 as Input
    wait_us(15);		   								//Strobe window	~ 12us
    InBit = gpio_get_value(bq2202a_gpio);		        //This function takes about 3us
													//Between the wait_us and GPIO_ReadBit functions
													//approximately 15us should occur to monitor the
													//GPIO line and determine if bit read is one or zero
    wait_us(65);									//End of Bit
    gpio_direction_output(bq2202a_gpio, GPIO_DIR_OUT_1);			//Set GPIO P9.3 as Output
													            //Set High
    return InBit;									//Return bit value
}

/**********************************************************************/
/* 	static unsigned char ReadOneByte(void)							  */
/*																      */
/*	Description : 		This procedure reads 8 bits on the SDQ line   */
/*				  		and returns the byte value.					  */
/* 	Arguments : 		Databyte - Byte value returned by SDQ slave	  */
/*						MaskByte - Used to seperate each bit	      */
/*						i - Used for 8 time loop					  */
/*	Global Variables:	None   										  */
/*  Returns: 			DataByte							          */
/**********************************************************************/
static unsigned char ReadOneByte(void)
{
    unsigned char i;
    unsigned char DataByte;
    unsigned char MaskByte;

    DataByte = 0x00;			 								//Initialize return value

    for (i = 0; i < 8; i++)                                      //Select one bit at a time
    {
        MaskByte = ReadOneBit();				    		//Read One Bit
        MaskByte <<= i;										//Determine Bit position within the byte
        DataByte = DataByte | MaskByte;					//Keep adding bits to form the byte
    }
    return DataByte;											//Return byte value read
}

/**********************************************************************/
/* 	void ReadBq2202aID(void)               							  */
/*																      */
/*	Description : 		This procedure reads BQ2202A'S ID on the SDQ  */
/*				  		line.                   					  */
/* 	Arguments : 		None                    					  */
/*	Global Variables:	None   										  */
/*  Returns: 			None       							          */
/**********************************************************************/
void ReadBq2202aID(void)
{
    unsigned char i;
    mutex_lock(&bq2202a_access);

    SendReset();
    wait_us(2);
    i = TestPresence();
    #ifdef DEBUG_BQ2202A
    //printk("TestPresence=%d\n",i);
    #endif
	//printk("TestPresence----1------WriteOneByte\t\n");
    //wait_us(600);
	//printk("TestPresence----2------WriteOneByte\t\n");
    WriteOneByte(READ_ID_CMD);                     		 // read rom commond
    for(i = 0;i < 8;i++)
    {
        //ReadIDDataByte[i] = ReadOneByte();     				 // read rom Partition 64bits = 8Bits
        ReadIDDataByte[7-i] = ReadOneByte();      // read rom Partition 64bits = 8Bits
    }

    mutex_unlock(&bq2202a_access);
    //#ifdef DEBUG_BQ2202A
	/*
    for(i=0;i<8;i++)
    {
        printk("ReadBq2202aID[%d]=%d\n",i,ReadIDDataByte[i]);
    }*/
    //#endif
	printk("ReadBq2202aID[0-7]:%03d,%03d,%03d,%03d,%03d,%03d,%03d,%03d\n",ReadIDDataByte[0],ReadIDDataByte[1],ReadIDDataByte[2],ReadIDDataByte[3],ReadIDDataByte[4],ReadIDDataByte[5],ReadIDDataByte[6],ReadIDDataByte[7]);
}
/**********************************************************************/
/* 	void CheckBq2202aID(void)               							  */
/*																      */
/*	Description : 		This procedure reads BQ2202A'S ID on the SDQ  */
/*				  		line.                   					  */
/* 	Arguments : 		None                    					  */
/*	Global Variables:	None   										  */
/*  Returns: 			None       							          */
/**********************************************************************/
void CheckBq2202aID(void)
{
    unsigned char i;
    mutex_lock(&bq2202a_access);

    SendReset();
    wait_us(2);
    i=TestPresence();
#ifdef DEBUG_BQ2202A
    //printk("TestPresence=%d\n",i);
#endif

    WriteOneByte(SKIP_ROM_CMD);              // skip rom commond
    wait_us(60);

#ifdef READ_PAGE_BQ2202A
    WriteOneByte(READ_PAGE_ID_CMD);     // read eprom Partition for page mode
#else
    WriteOneByte(READ_FIELD_ID_CMD);     // read eprom Partition for field mode
#endif
    wait_us(60);
    WriteOneByte(AddressLow);               // read eprom Partition Starting address low
    wait_us(60);
    WriteOneByte(AddressHigh);               // read eprom Partition Starting address high

#ifdef READ_PAGE_BQ2202A
    for(i = 0;i < 32;i++)
    {
        CheckIDDataByte[i] = ReadOneByte();   // read eprom Partition page1  256bits = 32Bits
    }

    mutex_unlock(&bq2202a_access);
	/*
    #ifdef DEBUG_BQ2202A
    for(i=0;i<32;i++)
    {
        printk("CheckBq2202aID[%d]=%d\n",i,CheckIDDataByte[i]);
    }
    #endif
	*/
	printk("CheckBq2202aID[16-23]:%03d,%03d,%03d,%03d,%03d,%03d,%03d,%03d\n",CheckIDDataByte[16],CheckIDDataByte[17],CheckIDDataByte[18],CheckIDDataByte[19],CheckIDDataByte[20],CheckIDDataByte[21],CheckIDDataByte[22],CheckIDDataByte[23]);
	//printk("CheckBq2202aID[24]=%03d,CheckBq2202aID[25]=%03d,CheckBq2202aID[26]=%03d,CheckBq2202aID[27]=%03d,CheckBq2202aID[28]=%03d,CheckBq2202aID[29]=%03d,CheckBq2202aID[30]=%03d,CheckBq2202aID[31]=%03d\n",CheckIDDataByte[24],CheckIDDataByte[25],CheckIDDataByte[26],CheckIDDataByte[27],CheckIDDataByte[28],CheckIDDataByte[29],CheckIDDataByte[30],CheckIDDataByte[31]);
	//printk("CheckBq2202aID[0] =%03d,CheckBq2202aID[1] =%03d,CheckBq2202aID[2] =%03d,CheckBq2202aID[3] =%03d,CheckBq2202aID[4] =%03d,CheckBq2202aID[5] =%03d,CheckBq2202aID[6] =%03d,CheckBq2202aID[7] =%03d\n",CheckIDDataByte[0],CheckIDDataByte[1],CheckIDDataByte[2],CheckIDDataByte[3],CheckIDDataByte[4],CheckIDDataByte[5],CheckIDDataByte[6],CheckIDDataByte[7]);
	//printk("CheckBq2202aID[8] =%03d,CheckBq2202aID[9] =%03d,CheckBq2202aID[10]=%03d,CheckBq2202aID[11]=%03d,CheckBq2202aID[12]=%03d,CheckBq2202aID[13]=%03d,CheckBq2202aID[14]=%03d,CheckBq2202aID[15]=%03d\n",CheckIDDataByte[8],CheckIDDataByte[9],CheckIDDataByte[10],CheckIDDataByte[11],CheckIDDataByte[12],CheckIDDataByte[13],CheckIDDataByte[14],CheckIDDataByte[15]);


#else
    for(i = 0;i < 128;i++)
    {
        CheckIDDataByte[i] = ReadOneByte();   // read eprom Partition field  1024bits = 128Bits
    }
    mutex_unlock(&bq2202a_access);

    #ifdef DEBUG_BQ2202A
    for(i = 0;i < 128;i++)
    {
        printk("CheckBq2202aID[%d]=%d\n",i,CheckIDDataByte[i]);
    }
    #endif
#endif
}
/**********************************************************************/
/* 	void CheckIDCompare(void)               							  */
/*																      */
/*	Description : 		This procedure reads BQ2202A'S ID on the SDQ  */
/*				  		line.                   					  */
/* 	Arguments : 		None                    					  */
/*	Global Variables:	None   										  */
/*  Returns: 			None       							          */
/**********************************************************************/
void CheckIDCompare(void)
{
    unsigned char i,j;
    int IDReadSign=1;

    if(IDReadSign == 1)
    {
        for(i = 0;i < 1;i++)
        {
            ReadBq2202aID();
            CheckBq2202aID();

            oppo_check_ID_status=0;
            if(ReadIDDataByte[7] == 0x09)
            {
                for(j = 1;j < 7;j++)
                {
                    if((ReadIDDataByte[j] == CheckIDDataByte[j + 16]) && (ReadIDDataByte[j] != 0xff)  && (ReadIDDataByte[j] != 0))
                    {
                        oppo_check_ID_status++;
                    }
                }
                if(oppo_check_ID_status > 0)
                {
                    IDReadSign = 0;
                    return;
                }
            }
            else
            {
                continue;
            }
        }
        IDReadSign=0;
    }
}

#define  ATL_VOLATGE  	1800
#define  LG_VOLATGE  	1200
#define  SDI_VOLATGE  	200

int opchg_get_bq2022_manufacture_id(void)
{
    unsigned char i;
	unsigned char manufac_id_buf[7] = {0x0};
	int batt_manufac_id = 0;
	int rc =0;

	if(is_project(OPPO_15109))
	{
		oppo_high_battery_status = 1;
		oppo_battery_status_init_flag = 1;

		if(opchg_chip == NULL)
		{
			return -EPROBE_DEFER;
		}
		else
		{
			bq2202a_gpio = opchg_get_prop_battery_id_voltage(opchg_chip);
			pr_debug("MPP4 battery id volatge is V_battery_id=%d,V_battery=%d\n", bq2202a_gpio,rc);

			if(bq2202a_gpio < 0)
			{
				pr_debug("MPP4 battery id volatge is fail\n");
				return -EPROBE_DEFER;
			}

			if(bq2202a_gpio <= SDI_VOLATGE)
			{
				batt_manufac_id = BATTERY_2420MAH_SDI;
				register_device_proc("battery_id", DEVICE_BATTERY_ID_VERSION, DEVICE_BATTERY_ID_TYPE_SDI);
			}
			else if(bq2202a_gpio <= LG_VOLATGE)
			{
				batt_manufac_id = BATTERY_2420MAH_LG;
				register_device_proc("battery_id", DEVICE_BATTERY_ID_VERSION, DEVICE_BATTERY_ID_TYPE_LG);
			}
			else
			{
				batt_manufac_id = BATTERY_2420MAH_ATL;
				register_device_proc("battery_id", DEVICE_BATTERY_ID_VERSION, DEVICE_BATTERY_ID_TYPE_ATL);
			}
			return batt_manufac_id;
		}
	}

	if(!oppo_battery_status_init_flag)
		return -1;

    mutex_lock(&bq2202a_access);
    SendReset();
    wait_us(2);
    TestPresence();

    WriteOneByte(SKIP_ROM_CMD);              // skip rom commond
    wait_us(60);

#ifdef READ_PAGE_BQ2202A
    WriteOneByte(READ_PAGE_ID_CMD);     // read eprom Partition for page mode
#else
    WriteOneByte(READ_FIELD_ID_CMD);     // read eprom Partition for field mode
#endif
    wait_us(60);
    WriteOneByte(BQ2022_MANUFACTURE_ADDR_LOW);               // read eprom Partition Starting address low
    wait_us(60);
    WriteOneByte(BQ2022_MANUFACTURE_ADDR_HIGH);               // read eprom Partition Starting address high

#ifdef READ_PAGE_BQ2202A
    for(i = 0;i < 7;i++)
    {
        manufac_id_buf[i] = ReadOneByte();   // read eprom Partition page1  256bits = 32Bits
        //printk("manufac_id[0x%x]:0x%x\n",i,manufac_id_buf[i]);
    }
    mutex_unlock(&bq2202a_access);

	printk("manufac_id[0-6]:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",manufac_id_buf[0],manufac_id_buf[1],
		manufac_id_buf[2],manufac_id_buf[3],manufac_id_buf[4],manufac_id_buf[5],manufac_id_buf[6]);
#else
    for(i = 0;i < 128;i++)
    {
        CheckIDDataByte[i] = ReadOneByte();   // read eprom Partition field  1024bits = 128Bits
    }
    mutex_unlock(&bq2202a_access);

    #ifdef DEBUG_BQ2202A
    for(i = 0;i < 128;i++)
    {
        printk("CheckBq2202aID[%d]=%d\n",i,CheckIDDataByte[i]);
    }
    #endif
#endif

	return batt_manufac_id;
}


bool oppo_battery_status_init(int batt_id_gpio)
{
    static int CheckIDSign = 5;

	/****************************************************************
	* Because the 15035 project does not use encryption chip bq2022
	*****************************************************************/
	if(is_project(OPPO_15109))
	{
		oppo_high_battery_status = 1;
		oppo_battery_status_init_flag = 1;
	}
	else
	{
		bq2202a_gpio = batt_id_gpio;
		Gpio_BatId_Init();

		if(!oppo_battery_status_init_flag)
		{
			while(CheckIDSign > 0)
		    {
		        CheckIDCompare();
		        CheckIDSign--;
		        if(oppo_check_ID_status > 0)
		        {
		            oppo_high_battery_status = 1;
		            oppo_check_ID_status = 0;
		            CheckIDSign = 0;
					oppo_battery_status_init_flag = 1;
					break;
		        }
		        else if(CheckIDSign <= 0)
		        {
		            oppo_high_battery_status = 0;
		            oppo_check_ID_status = 0;
					oppo_battery_status_init_flag = 1;
		        }
		    }
		}
	}

	return oppo_high_battery_status;
}
