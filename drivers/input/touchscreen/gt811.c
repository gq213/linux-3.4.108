#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <plat/gpio-cfg.h>
#include <linux/interrupt.h>
#include <linux/slab.h>


struct goodix_data{
	struct input_dev *input_dev;
	
	struct i2c_client *client;
	
	uint16_t abs_x_max;
	uint16_t abs_y_max;

	unsigned int int_pin;		
	unsigned int rst_pin;
	
	struct workqueue_struct *wq;
	struct work_struct work;
	
	unsigned int int_irq;
};

struct goodix_data *ts = NULL;


static int i2c_read_bytes(struct i2c_client *client, uint8_t *buf, int len)
{
	struct i2c_msg msgs[2];
	
	msgs[0].flags=!I2C_M_RD;
	msgs[0].addr=client->addr;
	msgs[0].len=2;
	msgs[0].buf=&buf[0];

	msgs[1].flags=I2C_M_RD;
	msgs[1].addr=client->addr;
	msgs[1].len=len-2;
	msgs[1].buf=&buf[2];
	
	return i2c_transfer(client->adapter, msgs, 2);
}

static int i2c_write_bytes(struct i2c_client *client, uint8_t *data, int len)
{
	struct i2c_msg msg;
	
	msg.flags=!I2C_M_RD;
	msg.addr=client->addr;
	msg.len=len;
	msg.buf=data;	
	
	return i2c_transfer(client->adapter, &msg, 1);
}

#define MD_SWITCH		0x0C
#define TOUCH_N			0x05
#define X_OU_MAX_L		(480 & 0xff)
#define X_OU_MAX_H		(480 >> 8)
#define Y_OU_MAX_L		(800 & 0xff)
#define Y_OU_MAX_H		(800 >> 8)

static int ts_init_panel(struct i2c_client *client)
{
	int ret;
	uint8_t config_info[] = {
		0x06,0xA2,
		0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x02,0x22,0x12,0x22,0x22,0x22,
		0x32,0x22,0x42,0x22,0x52,0x22,0x62,0x22,0x72,0x22,0x82,0x22,0x92,0x22,0xA2,0x22,
		0xB2,0x22,0xC2,0x22,0xD2,0x22,0xE2,0x22,0xF2,0x22,0x1B,0x03,0x60,0x60,0x60,0x16,
		0x16,0x16,0x0F,0x0F,0x0A,0x40,0x30,MD_SWITCH,0x03,0x63,
		TOUCH_N,
		X_OU_MAX_L,X_OU_MAX_H,Y_OU_MAX_L,Y_OU_MAX_H,
		0x00,0x00,0x38,0x33,0x35,0x30,0x00,0x00,0x05,0x19,0x02,0x08,0x00,0x00,0x00,0x00,0x00,
		0x14,0x10,0x30,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01
	};
	
	ret = i2c_write_bytes(client, config_info, sizeof(config_info)/sizeof(config_info[0]));
	if (ret <= 0) {
		printk("Warnning: GT811 Send config failed!\n");
		return -1;
	}
	
	msleep(10);
	
	return 0;
}


static void ts_work_func(struct work_struct* work)
{
	uint8_t point_data[36] = {0x07, 0x21, 0};
	uint8_t point_index;
	uint8_t point_tmp;
	uint8_t track_id[TOUCH_N];
	uint8_t touch_num = 0;
	uint8_t check_sum = 0;
	int sum_pos;
	uint16_t input_x = 0;
	uint16_t input_y = 0;
	uint8_t  input_w = 0;
	int ret;
	int i;
	
	ret = i2c_read_bytes(ts->client, point_data, sizeof(point_data)/sizeof(point_data[0]));
	if (ret <= 0) {
		printk("ERROR: I2C transfer error. Number:%d\n", ret);
		goto reirq_enable;
	}
	
	if ((point_data[2] & 0x20) && (point_data[3] == 0xF0)) {
		printk("ERROR: gt811 need reinit!\n");
		goto reirq_enable;
	}
	
	point_index = point_data[2] & 0x1f;
    point_tmp = point_index;
	for (i=0; (i<TOUCH_N)&&point_tmp; i++) {
		if (point_tmp & 0x01) {
			track_id[touch_num++] = i;
		}
		
		point_tmp >>= 1;
	}
	//printk("touch_num:%d\n", touch_num);
	
	if (touch_num) {
		switch (point_index) {
			case 0:
				sum_pos = 3;
				break;
			case 1:
				for (i=2; i<9; i++) {
					check_sum += (int)point_data[i];
				}
				sum_pos = 9;
				break;
			case 2:
			case 3:
				for (i=2; i<14; i++) {
					check_sum += (int)point_data[i];
				}
				sum_pos = 14;
				break;
			default:
				for (i=2; i<35; i++) {
					check_sum += (int)point_data[i];
				}
				sum_pos = 35;
				break;
		}
		if (check_sum != point_data[sum_pos]) {
			printk("ERROR: Cal_chksum:%d, Read_chksum:%d\n", check_sum, point_data[sum_pos]);
			goto reirq_enable;
		}
	
		for (i=0; i<touch_num; i++) {
			point_index = track_id[i];
			
            if (point_index == 3) {
                input_x = (uint16_t)(point_data[19] << 8) + (uint16_t)point_data[26];
                input_y = (uint16_t)(point_data[27] << 8) + (uint16_t)point_data[28];
                input_w = point_data[29];
            } else {
				int offset;
				
                if (point_index < 3) {
                    offset = 4 + (point_index * 5);
                } else {
                    offset = 30;
                }
				
                input_x = (uint16_t)(point_data[offset] << 8) + (uint16_t)point_data[offset+1];
                input_y = (uint16_t)(point_data[offset+2] << 8) + (uint16_t)point_data[offset+3];
                input_w = point_data[offset+4];
            }
			
			if ((input_x > ts->abs_x_max) || (input_y > ts->abs_y_max)) {
				 continue;
			}
			
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);			
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, point_index);
			input_mt_sync(ts->input_dev);
			
			//printk("ID=%d, X=%d, Y=%d, W=%d\n", point_index, input_x, input_y, input_w);
		}
    } else {
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_mt_sync(ts->input_dev);
		
		//printk("Touch Release!\n");
    }
	
    input_report_key(ts->input_dev, BTN_TOUCH, touch_num);
    input_sync(ts->input_dev);
	
reirq_enable:
	enable_irq(ts->int_irq);
}

static irqreturn_t ts_irq_handler(int irq, void *dev_id)
{
	disable_irq_nosync(ts->int_irq);
	
	//printk("%s\n", __func__);
	
	queue_work(ts->wq, &ts->work);
	
	return IRQ_RETVAL(IRQ_HANDLED);
}


static int init_input_dev(void)
{
	int ret;
	uint16_t input_dev_x_max = ts->abs_x_max;
    uint16_t input_dev_y_max = ts->abs_y_max;
	
	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		printk("ERROR: failed to allocate input device for key & vol !!\n");
		goto fail;
	}
	
	ts->input_dev->name = "tq210-ts";
	ts->input_dev->phys = "input/ts";
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor  =0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427;	//screen firmware version	
	
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);
	
	input_set_abs_params(ts->input_dev, ABS_X, 0, input_dev_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, input_dev_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, input_dev_x_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, input_dev_y_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);	
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, TOUCH_N, 0, 0);
	
	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk("ERROR: Unable register %s input device!\n", ts->input_dev->name);
		goto fail1;
	}
	
	return 0;

fail1:
	input_free_device(ts->input_dev);
fail:
	return -1;
}

static void uninit_input_dev(void)
{
	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
}

static int ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	int retry;
	char test_data = 1;
	int int_trigger_type;

	ts = (struct goodix_data *)kzalloc(sizeof(struct goodix_data), GFP_KERNEL);
	if (ts == NULL) {
		printk("ERROR: Allocate ts failed!\n");
		ret = -ENOMEM;
		goto exit;
	}

	ts->client = client;
    ts->abs_x_max = 480;
    ts->abs_y_max = 800;
	
	ret = init_input_dev();
	if (ret < 0) {
		printk("ERROR: no tsint pin available !!\n");
		ret = -ENOMEM;
		goto free_ts;
	}

	ts->int_pin = S5PV210_GPH1(6);
	ts->rst_pin = S5PV210_GPD0(3);
	
	ret = gpio_request(ts->int_pin, "tsint");
	if (ret < 0) {
		printk("ERROR: no tsint pin available !!\n");
		ret = -1;
		goto free_input_dev;
	}
	
	gpio_direction_input(ts->int_pin);
	s3c_gpio_setpull(ts->int_pin, S3C_GPIO_PULL_NONE);
	
	ret = gpio_request(ts->rst_pin, "reset");
	if (ret < 0) {
		printk("ERROR: no reset pin available !!\n");
		ret = -1;
		goto free_int_pin;
	}
	
	gpio_direction_input(ts->rst_pin);
	s3c_gpio_setpull(ts->rst_pin, S3C_GPIO_PULL_NONE);
	
	gpio_direction_output(ts->rst_pin, 0);
	msleep(20);
	
	gpio_direction_input(ts->rst_pin);
	s3c_gpio_setpull(ts->rst_pin, S3C_GPIO_PULL_NONE);
	msleep(50);
	
	for (retry=0; retry<5; retry++) {
	
		ret = i2c_write_bytes(client, &test_data, 1);	//Test I2C connection.
		if (ret > 0) {
			break;
		}
		
		printk("Warnning: GT811 I2C TEST FAILED!Please check the HARDWARE connect\n");
		msleep(10);
	}
	if (ret <= 0) {
		printk("ERROR: I2C communication might be ERROR!\n");
		ret = -ENODEV;
		goto free_rst_pin;
	}

	for (retry=0; retry<5; retry++) {
		ret = ts_init_panel(client);
		if(ret == 0)
			break;
	}
	if (ret != 0) {
		printk("ERROR: GT811 Configue failed!\n");
		ret = -ENODEV;
		goto free_rst_pin;
	}
	
	ts->wq = create_singlethread_workqueue("ts_handle_thread");
	if (ts->wq == NULL) {
		printk("ERROR: Allocate work queue faile!\n");
		ret = -1;
		goto free_rst_pin;
	}
	INIT_WORK(&ts->work, ts_work_func);
	
	ts->int_irq = gpio_to_irq(ts->int_pin);
	if (ts->int_irq < 0) {
		printk("ERROR: Can't get irq.\n");
		ret = -1;
		goto destroy_wq;
	}
	
	int_trigger_type = MD_SWITCH & 0x08;
	printk("+%s: TRIG_MODE=%s\n", __func__, int_trigger_type ? "RISING EDGE" : "FALLING EDGE");
	
	ret = request_irq(ts->int_irq, ts_irq_handler, 
		(int_trigger_type ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING) | IRQF_DISABLED | IRQF_NO_SUSPEND,   
		"gt811-int", NULL);
	if (ret < 0) {
		printk("ERROR: Can't request %d.\n", ts->int_irq);
		ret = -1;
		goto destroy_wq;
	}
	
	printk("+%s: gt811 init done.\n", __func__);
	
	return 0;
	
destroy_wq:
	destroy_workqueue(ts->wq);
free_rst_pin:
	gpio_direction_input(ts->rst_pin);
	s3c_gpio_setpull(ts->rst_pin, S3C_GPIO_PULL_NONE);
	gpio_free(ts->rst_pin);
free_int_pin:
	gpio_direction_input(ts->int_pin);
	s3c_gpio_setpull(ts->int_pin, S3C_GPIO_PULL_NONE);
	gpio_free(ts->int_pin);
free_input_dev:
	uninit_input_dev();
free_ts:
	kfree(ts);
exit:
	return ret;
}

static int ts_remove(struct i2c_client *client){
	disable_irq(ts->int_irq);
	free_irq(ts->int_irq, NULL);
	
	destroy_workqueue(ts->wq);
	
	gpio_direction_input(ts->rst_pin);
	s3c_gpio_setpull(ts->rst_pin, S3C_GPIO_PULL_NONE);
	gpio_free(ts->rst_pin);
	
	gpio_direction_input(ts->int_pin);
	s3c_gpio_setpull(ts->int_pin, S3C_GPIO_PULL_NONE);
	gpio_free(ts->int_pin);
	
	uninit_input_dev();
	
	kfree(ts);
	
	return 0;
}

static const struct i2c_device_id ts_id[] = {
	{ "tq210-ts", 0 },
	{ }
};

static struct i2c_driver ts_driver = {
	.probe    = ts_probe,
	.remove   = ts_remove,
	.id_table = ts_id,

	.driver = {
		.name = "tq210-ts",
		.owner = THIS_MODULE,
	},
};

static int ts_init(void)
{
	int ret;
	
	ret = i2c_add_driver(&ts_driver);
	if (ret < 0) {
		printk("ERROR: i2c_add_driver fail!\n");
		return -1;
	}
	
	return 0;
}

static void ts_exit(void)
{
	i2c_del_driver(&ts_driver);
}

module_init(ts_init);
module_exit(ts_exit);
MODULE_LICENSE("GPL");