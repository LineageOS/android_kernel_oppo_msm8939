/*******************************************************************************
* Copyright (c)  2015- 2016  Guangdong OPPO Mobile Telecommunications Corp., Ltd
* CONFIG_MACH_OPPO
* Description: Source file for CBufferList.
*           To allocate and free memory block safely.
* Version   : 0.1
* Date      : 2015-07-20
* Author    : sjc copy from oppo_vooc.c
* ---------------------------------- Revision History: -------------------------
* <version>           <date>          < author >              <desc>
* Revision 0.1        2015-07-20      sjc
* Modified to be suitable to the new coding rules in all functions.
*******************************************************************************/

#define OPPO_VOOC_STM8S_PAR
#include "oppo_inc.h"

//static struct task_struct *pic16f_fw_update_task = NULL;
#define BYTE_OFFSET 			2
#define BYTES_TO_WRITE 		16
#define FW_CHECK_FAIL		0
#define FW_CHECK_SUCCESS		1

#if defined CONFIG_OPPO_DEVICE_FIND7 || defined CONFIG_OPPO_DEVICE_FIND7WX	//FIND7/FIND7WX:pic1503
#define ERASE_COUNT   		96	//0x200-0x7FF usb_uar_control
#define READ_COUNT			95	//192
#else
#define ERASE_COUNT   		224	//0x200-0xFFF
#define READ_COUNT			223	//448
#endif


//static struct i2c_client *pic16F_client;
int st_fw_ver_count_15022 = sizeof(stm8s_firmware_data_15022);
int st_fw_ver_count = sizeof(stm8s_firmware_data_15022);
//static int st_need_to_up_fw = 0;
//static int st_have_updated = 0;
extern bool vooc_hw_init_flag;
extern int vooc_have_updated;
extern int vooc_fw_ver_count;
extern unsigned char *vooc_firmware_data;
extern struct opchg_fast_charger *opchg_fast_charger_chip;
extern int vooc_get_mcu_hw_type(void);
extern int (*vooc_fw_update)(struct opchg_fast_charger *,bool);


static bool stm8s_fw_check(struct opchg_fast_charger *chip)
{
	unsigned char addr_buf[2] = {0x88,0x00};
	unsigned char data_buf[32] = {0x0};
	int rc,i,j,addr;
	int fw_line = 0;

	//first:set address
	rc = i2c_smbus_write_i2c_block_data(chip->client,0x01,2,&addr_buf[0]);
	if(rc < 0){
		pr_err("%s i2c_write 0x01 error\n",__func__);
		goto i2c_err;
	}
	msleep(10);

	for(i = 0;i < READ_COUNT;i++){	//1508:448,1503:192
		i2c_smbus_read_i2c_block_data(chip->client,0x03,16,&data_buf[0]);
		msleep(2);
		i2c_smbus_read_i2c_block_data(chip->client,0x03,16,&data_buf[16]);

		addr = 0x8800 + i * 32;

			if(addr == ((vooc_firmware_data[fw_line * 34 + 1] << 8) | vooc_firmware_data[fw_line * 34]))
			{
				if(data_buf[0] != vooc_firmware_data[fw_line * 34 + 2]){
					pr_err("%s fail,data_buf[0]:0x%x != vooc_firmware_data[%d]:0x%x\n",__func__,
							data_buf[0],(fw_line * 34 + 2),vooc_firmware_data[fw_line * 34 + 2]);
					pr_err("%s addr = 0x%x,%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n",__func__,addr,
							data_buf[0],data_buf[1],data_buf[2],data_buf[3],data_buf[4],data_buf[5],data_buf[6],data_buf[7],
							data_buf[8],data_buf[9],data_buf[10],data_buf[11],data_buf[12],data_buf[13],data_buf[14],
							data_buf[15],data_buf[16],data_buf[17],data_buf[18],data_buf[19],data_buf[20],data_buf[21],data_buf[22],
							data_buf[23],data_buf[24],data_buf[25],data_buf[26],data_buf[27],data_buf[28],data_buf[29],data_buf[30],
							data_buf[31]);
					return FW_CHECK_FAIL;
				}
				fw_line++;
			}
			else
			{
				if(addr == ((vooc_firmware_data[fw_line * 34 + 1] << 8) | vooc_firmware_data[fw_line * 34]))
				{
					for(j = 0;j < 32;j++){
						if(data_buf[j] != vooc_firmware_data[fw_line * 34 + 2 + j]){
							pr_err("%s fail,data_buf[%d]:0x%x != vooc_firmware_data[%d]:0x%x\n",__func__,
								j,data_buf[j],(fw_line * 34 + 2 + j),vooc_firmware_data[fw_line * 34 + 2 + j]);
							return FW_CHECK_FAIL;
						}
					}
					fw_line++;
				} else {
				}
			}
	}
	pr_info("%s success\n",__func__);
	return FW_CHECK_SUCCESS;

i2c_err:
	pr_err("%s is not success\n",__func__);
	return FW_CHECK_FAIL;
}

static int stm8s_fw_write(struct opchg_fast_charger *chip,unsigned char *data_buf,unsigned int offset,unsigned int length)
{
	unsigned int count = 0;
	unsigned char zero_buf[1] = {0};
	unsigned char temp_buf[1] = {0};
	unsigned char addr_buf[2] = {0x88,0x00};
	//unsigned char temp;
	int rc;

	count = offset;
	//write data begin
	while(count < (offset + length))
	{
		addr_buf[0] = data_buf[count + 1];
		addr_buf[1] = data_buf[count];
		//printk("%s write data addr_buf[0]:0x%x,addr_buf[1]:0x%x\n",__func__,addr_buf[0],addr_buf[1]);
		rc = i2c_smbus_write_i2c_block_data(chip->client,0x01,2,&addr_buf[0]);
		if(rc < 0){
			pr_err("%s i2c_write 0x01 error\n",__func__);
			return -1;
		}

		//swap low byte and high byte end
		//write 16 bytes data to pic16F
		i2c_smbus_write_i2c_block_data(chip->client,0x02,BYTES_TO_WRITE,&data_buf[count+BYTE_OFFSET]);
		i2c_smbus_write_i2c_block_data(chip->client,0x05,1,&zero_buf[0]);
		i2c_smbus_read_i2c_block_data(chip->client,0x05,1,&temp_buf[0]);
		//printk("lfc read 0x05,temp_buf[0]:0x%x\n",temp_buf[0]);

		//write 16 bytes data to pic16F again
		i2c_smbus_write_i2c_block_data(chip->client,0x02,BYTES_TO_WRITE,&data_buf[count+BYTE_OFFSET+BYTES_TO_WRITE]);
		i2c_smbus_write_i2c_block_data(chip->client,0x05,1,&zero_buf[0]);
		i2c_smbus_read_i2c_block_data(chip->client,0x05,1,&temp_buf[0]);
		//printk("lfc read again 0x05,temp_buf[0]:0x%x\n",temp_buf[0]);

		count = count + BYTE_OFFSET + 2 * BYTES_TO_WRITE;

		msleep(2);
		//pr_err("%s count:%d,offset:%d,length:%d\n",__func__,count,offset,length);
		if(count > (offset + length - 1)){
			break;
		}
	}
	return 0;
}

int stm8s_fw_update(struct opchg_fast_charger *chip,bool enable)
{
	unsigned char zero_buf[1] = {0};
	unsigned char addr_buf[2] = {0x88,0x00};
	unsigned char temp_buf[1]={0};
	int i,rc=0;
	unsigned int addr = 0x8800;
	int download_again = 0;

	pr_err("%s is start,erase data ing.......\n",__func__);

	if (enable)
		opchg_chip->updating_fw_flag = true;

	if(enable){
		//rc = opchg_set_switch_mode(VOOC_CHARGER_MODE);
		if(rc < 0){
			pr_err("%s pull up switch fail\n",__func__);
			if (enable)
				opchg_chip->updating_fw_flag = false;
			return rc;
		}
		msleep(300);
	}

update_fw:
	//erase address 0x200-0x7FF
	for(i = 0;i < ERASE_COUNT;i++){
		//first:set address
		rc = i2c_smbus_write_i2c_block_data(chip->client,0x01,2,&addr_buf[0]);
		if(rc < 0){
			pr_err("%s stm8s_update_fw,i2c_write 0x01 error, i=%d\n",__func__,i);
			goto update_fw_err;
		}

		//erase data:0x10 words once
		i2c_smbus_write_i2c_block_data(chip->client,0x04,1,&zero_buf[0]);
		msleep(1);
		i2c_smbus_read_i2c_block_data(chip->client,0x04,1,&temp_buf[0]);
		//printk("lfc read 0x04,temp_buf[0]:0x%x\n",temp_buf[0]);

		//erase data:0x10 words once
		addr = addr + 0x10;
		addr_buf[0] = addr >> 8;
		addr_buf[1] = addr & 0xFF;
		//printk("lfc addr_buf[0]:0x%x,addr_buf[1]:0x%x\n",addr_buf[0],addr_buf[1]);
	}
	msleep(10);

	if (is_project(OPPO_15022))
	{
		if (get_PCB_Version() == HW_VERSION__16)
			rc = stm8s_fw_write(chip,vooc_firmware_data,0,vooc_fw_ver_count - 34);
		else
			rc = stm8s_fw_write(chip,vooc_firmware_data,0,vooc_fw_ver_count - 34);
	}
	else
	{
		rc = stm8s_fw_write(chip,vooc_firmware_data,0,vooc_fw_ver_count - 34);
	}
	if (rc < 0) {
		pr_err("%s fw write error\n",__func__);
		goto update_fw_err;
	}

	//fw check begin:read data from pic1503/1508,and compare it with Pic16F_firmware_data[]
	rc = stm8s_fw_check(chip);
	if(rc == FW_CHECK_FAIL){
		download_again++;
		if(download_again > 3){
			goto update_fw_err;
		}

#ifdef OPPO_USE_FAST_CHARGER
		opchg_set_reset_active(bq27541_di);
#endif
		msleep(1000);
		pr_err("%s fw check fail,download fw again\n",__func__);
		goto update_fw;
	}
	//fw check end

	//write 0x7F0~0x7FF(0x7FF = 0x3455)
	if (is_project(OPPO_15022))
	{
		if (get_PCB_Version() == HW_VERSION__16) {
			rc = stm8s_fw_write(chip,vooc_firmware_data,vooc_fw_ver_count - 34,34);
			if (rc < 0) {
				goto update_fw_err;
			}
			msleep(5);
			//stm8s_fw_data_recover(chip,vooc_firmware_data,vooc_fw_ver_count - 34,34);
		}
	}
	else
	{
		rc = stm8s_fw_write(chip,vooc_firmware_data,vooc_fw_ver_count - 34,34);
		if (rc < 0) {
			goto update_fw_err;
		}
		msleep(5);
		//stm8s_fw_data_recover(chip,vooc_firmware_data,vooc_fw_ver_count - 34,34);
	}

	//write 0x7F0~0x7FF end

	msleep(2);
	//jump to app code begin
	i2c_smbus_write_i2c_block_data(chip->client,0x06,1,&zero_buf[0]);
	i2c_smbus_read_i2c_block_data(chip->client,0x06,1,&temp_buf[0]);
	//jump to app code end
	vooc_have_updated = 1;

	//pull down GPIO96 to power off MCU1503/1508
	if(enable) {
		if(rc < 0){
			//pr_err("%s pull down switch fail\n",__func__);
		}
	}
	pr_err("%s stm8s update_fw success\n",__func__);
	if (enable)
		opchg_chip->updating_fw_flag = false;
	return 0;

update_fw_err:
	if(enable){
		if(rc < 0){
			//pr_err("%s pull down switch fail\n",__func__);
		}
	}
	pr_err("%s stm8s update_fw is not success\n",__func__);
	if (enable)
		opchg_chip->updating_fw_flag = false;
	return 1;
}


void stm8s_fw_update_thread(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct opchg_fast_charger *chip = container_of(dwork,
								struct opchg_fast_charger, update_opfastchg_thread_work);
#ifdef OPPO_USE_FAST_CHARGER
	opchg_set_reset_active(bq27541_di);
#endif
	printk(KERN_ERR "%s: update fw after 1s...\n", __func__);
	msleep(1000);
	stm8s_fw_update(chip,1);

}


void opchg_stm8s_fast_charging_works_init(struct opchg_fast_charger *chip)
{
    INIT_DELAYED_WORK(&chip->update_opfastchg_thread_work, stm8s_fw_update_thread);
    schedule_delayed_work(&chip->update_opfastchg_thread_work,
                            round_jiffies_relative(msecs_to_jiffies(OPCHG_THREAD_INTERVAL_INIT)));

}
static int stm8s_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int retval;
    struct opchg_fast_charger *chip;

	/**/
	if (vooc_get_mcu_hw_type() != OPCHG_VOOC_STM8S_ID)
		return 0;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
    if (!chip) {
        dev_err(&client->dev, "Couldn't allocate memory\n");
        return -ENOMEM;
    }
	chip->client = client;	//pic16F_client = client;
    chip->dev = &client->dev;

	/**/
    /* i2c pull up Regulator configuration */
    chip->vcc_i2c = regulator_get(&client->dev, "vcc_i2c_stm8s");
    if (IS_ERR(chip->vcc_i2c)) {
        dev_err(&client->dev, "%s: Failed to get i2c regulator\n", __func__);
        retval = PTR_ERR(chip->vcc_i2c);
        return -1;//retval;
    }
    if (regulator_count_voltages(chip->vcc_i2c) > 0) {
        retval = regulator_set_voltage(chip->vcc_i2c, OPCHARGER_I2C_VTG_MIN_UV, OPCHARGER_I2C_VTG_MAX_UV);
        if (retval) {
            dev_err(&client->dev, "reg set i2c vtg failed retval =%d\n", retval);
            goto err_set_vtg_i2c;
        }
    }

    retval = regulator_enable(chip->vcc_i2c);
    if (retval) {
        dev_err(&client->dev,"Regulator vcc_i2c enable failed " "rc=%d\n", retval);
        return retval;
    }
	/**/
	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s stm8s_probe,i2c_func error\n",__func__);
		goto err_check_functionality_failed;
	}

	/**/
    chip->opchg_fast_driver_id = id->driver_data;
	if (is_project(OPPO_15022)) {
		vooc_fw_ver_count = st_fw_ver_count_15022;
		vooc_firmware_data = stm8s_firmware_data_15022;
	} else {
		vooc_fw_ver_count = st_fw_ver_count;
		vooc_firmware_data = stm8s_firmware_data_15022;
	}
	opchg_fast_charger_chip_stm8s = chip;
	opchg_fast_charger_chip = opchg_fast_charger_chip_stm8s;
	vooc_fw_update = stm8s_fw_update;
    dev_dbg(chip->dev, "opchg_fast_driver_id=%d\n",chip->opchg_fast_driver_id);

	if( chip->opchg_fast_driver_id == OPCHG_VOOC_PIC16F_ID)
	{
		register_device_proc("fastcharger_mcu", DEVICE_FASTCHARGER_MCU_VERSION, DEVICE_FASTCHARGER_MCU_TYPE_PIC16F);
	}
	else if( chip->opchg_fast_driver_id == OPCHG_VOOC_STM8S_ID)
	{
		register_device_proc("fastcharger_mcu", DEVICE_FASTCHARGER_MCU_VERSION, DEVICE_FASTCHARGER_MCU_TYPE_STM8S);
	}
	else
	{
		register_device_proc("fastcharger_mcu", DEVICE_FASTCHARGER_MCU_VERSION, DEVICE_FASTCHARGER_MCU_TYPE_UNKOWN);
	}

	/**/
	//rc=opchg_pic16f_parse_dt(chip);

	/**/
	opchg_stm8s_fast_charging_works_init(chip);

	return 0;

err_set_vtg_i2c:
		if (regulator_count_voltages(chip->vcc_i2c) > 0) {
			regulator_set_voltage(chip->vcc_i2c, 0, OPCHARGER_I2C_VTG_MAX_UV);
		}

err_check_functionality_failed:
	pr_err("%s is fail\n",__func__);
	return 0;
}


static int stm8s_remove(struct i2c_client *client)
{
    return 0;
}

static const struct of_device_id stm8s_match[] = {
	{ .compatible = "st,stm8s_fastcg" },
	{ },
};

static const struct i2c_device_id stm8s_id[] = {
	{ "stm8s_fastcg", OPCHG_VOOC_STM8S_ID },
	{},
};
MODULE_DEVICE_TABLE(i2c, stm8s_id);


static struct i2c_driver stm8s_fastcg_driver = {
	.driver		= {
		.name = "stm8s_fastcg",
		.owner	= THIS_MODULE,
		.of_match_table = stm8s_match,
	},
	.probe		= stm8s_probe,
	.remove		= stm8s_remove,
	.id_table	= stm8s_id,
};
module_i2c_driver(stm8s_fastcg_driver);

MODULE_DESCRIPTION("OPCHARGER Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:opcharger-charger");
