/*
 * rpi-interface.c
 *
 *  Created on: Dec 27, 2023
 *      Author: SP5WWP
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdarg.h>

#include <fcntl.h> 
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <time.h>
#include <signal.h>

#include <zmq.h>

//rpi-interface commands
#include "interface_cmds.h"
#include "term.h" //colored terminal font

#define DEBUG_HALT				while(1)

//config stuff
struct config_t
{
	uint8_t uart[64];
	uint32_t uart_rate;
	//GPIO
	uint16_t pa_en;
	uint16_t boot0;
	uint16_t nrst;
    //ZMQ
    uint16_t dl_port;
    uint16_t ul_port;
    uint16_t ctrl_port;
} config;

int uart_byte_cnt=0;                //how many bytes are available on UART
int zmq_byte_cnt=0;                 //how many bytes are available over ZMQ

uint8_t uart_buff[1000];
uint8_t zmq_buff[10000];

uint8_t reset_only=0;

//debug printf
void dbg_print(const char* color_code, const char* fmt, ...)
{
	char str[200];
	va_list ap;

	va_start(ap, fmt);
	vsprintf(str, fmt, ap);
	va_end(ap);

	if(color_code!=NULL)
	{
		printf(color_code);
		printf(str);
		printf(TERM_DEFAULT);
	}
	else
	{
		printf(str);
	}
}

//UART magic
int fd; //UART handle

int get_baud(uint32_t baud)
{
    switch(baud)
	{
		case 9600:
			return B9600;
		case 19200:
			return B19200;
		case 38400:
			return B38400;
		case 57600:
			return B57600;
		case 115200:
			return B115200;
		case 230400:
			return B230400;
		case 460800:
			return B460800;
		case 500000:
			return B500000;
		case 576000:
			return B576000;
		case 921600:
			return B921600;
		case 1000000:
			return B1000000;
		case 1152000:
			return B1152000;
		case 1500000:
			return B1500000;
		case 2000000:
			return B2000000;
		case 2500000:
			return B2500000;
		case 3000000:
			return B3000000;
		case 3500000:
			return B3500000;
		case 4000000:
			return B4000000;
		default: 
			return -1;
    }
}

int set_interface_attribs(int fd, uint32_t speed, int parity)
{
	struct termios tty;
	if (tcgetattr (fd, &tty) != 0)
	{
		//error_message ("error %d from tcgetattr", errno);
		return -1;
 	}

	cfsetospeed(&tty, get_baud(speed));
	cfsetispeed(&tty, get_baud(speed));

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;			// disable break processing
	tty.c_lflag = 0;				// no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 0;            // 0.0 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if(tcsetattr(fd, TCSANOW, &tty)!=0)
	{		
		dbg_print(TERM_RED, " Error from tcsetattr\n");
		return -1;
	}
	
	return 0;
}

void set_blocking(int fd, int should_block)
{
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if(tcgetattr(fd, &tty)!=0)
	{
		dbg_print(TERM_YELLOW, " Error from tggetattr\n");
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if(tcsetattr(fd, TCSANOW, &tty)!=0)
	{
		dbg_print(TERM_YELLOW, " Error setting UART attributes\n");
	}
}

//GPIO - library-less, guerrilla style - we assume that only 3 GPIOs will be used
void gpio_init(void)
{
	FILE* fp;
	char tmp[256];
	
	//enable
	fp=fopen("/sys/class/gpio/export", "wb");
	if(fp!=NULL)
	{
		sprintf(tmp, "%d", config.pa_en);
		fwrite(tmp, strlen(tmp), 1, fp);
		fclose(fp);
	}
	else
	{
		dbg_print(TERM_RED, " can not initialize PA_EN (GPIO%d)\nExiting\n", config.pa_en);
		exit(1);
	}

	fp=fopen("/sys/class/gpio/export", "wb");
	if(fp!=NULL)
	{
		sprintf(tmp, "%d", config.boot0);
		fwrite(tmp, strlen(tmp), 1, fp);
		fclose(fp);
	}
	else
	{
		dbg_print(TERM_RED, " can not initialize BOOT0 (GPIO%d)\nExiting\n", config.boot0);
		exit(1);
	}
	
	fp=fopen("/sys/class/gpio/export", "wb");
	if(fp!=NULL)
	{
		sprintf(tmp, "%d", config.nrst);
		fwrite(tmp, strlen(tmp), 1, fp);
		fclose(fp);
	}
	else
	{
		dbg_print(TERM_RED, " can not initialize nRST (GPIO%d)\nExiting\n", config.nrst);
		exit(1);
	}

	usleep(250000U); //give it 250ms

	//set as output, default value is logic low
	sprintf(tmp, "/sys/class/gpio/gpio%d/direction", config.pa_en);
	fp=fopen(tmp, "wb");
	if(fp!=NULL)
	{
		fwrite("out", 3, 1, fp);
		fclose(fp);
	}
	else
	{
		dbg_print(TERM_RED, " can not initialize PA_EN (GPIO%d)\nExiting\n");
		exit(1);
	}

	sprintf(tmp, "/sys/class/gpio/gpio%d/direction", config.boot0);
	fp=fopen(tmp, "wb");
	if(fp!=NULL)
	{
		fwrite("out", 3, 1, fp);
		fclose(fp);
	}
	else
	{
		dbg_print(TERM_RED, " can not initialize BOOT0 (GPIO%d)\nExiting\n");
		exit(1);
	}

	sprintf(tmp, "/sys/class/gpio/gpio%d/direction", config.nrst);
	fp=fopen(tmp, "wb");
	if(fp!=NULL)
	{
		fwrite("out", 3, 1, fp);
		fclose(fp);
	}
	else
	{
		dbg_print(TERM_RED, " can not initialize nRST (GPIO%d)\nExiting\n");
		exit(1);
	}
}

uint8_t gpio_set(uint16_t gpio, uint8_t state)
{
	FILE* fp=NULL;
	char tmp[256];

	sprintf(tmp, "/sys/class/gpio/gpio%d/value", gpio);
	fp=fopen(tmp, "wb");

	if(fp!=NULL)
	{
		if(state)
		{
			fwrite("1", 1, 1, fp);
		}
		else
		{
			fwrite("0", 1, 1, fp);
		}

		fclose(fp);
		return 0;
	}
	else
	{
		dbg_print(TERM_YELLOW, " Error - can not set GPIO%d value\n", gpio);
		return 1;
	}
}

//commands and funcs
void dev_ping(void)
{
	uint8_t cmd[3]={CMD_PING, 3, 0}; //PING
	write(fd, cmd, 3);
}

void dev_set_rx_freq(uint64_t freq)
{
	uint8_t cmd[11];
	cmd[0]=CMD_SET_RX_FREQ;		//RX freq
	*((uint16_t*)&cmd[1])=11;
	*((uint64_t*)&cmd[3])=freq;
	write(fd, cmd, 11);
	
	//wait for device's response
	do
	{
		ioctl(fd, FIONREAD, &uart_byte_cnt);
	}
	while(uart_byte_cnt!=4);
	uint8_t resp[4]={0};
	read(fd, resp, 4);
	
	if(resp[0]==CMD_SET_RX_FREQ && *((uint16_t*)&resp[1])==4)
	{
        if(resp[3]==0)
		    dbg_print(0, "RX frequency: %lu Hz\n", freq); //OK
        else
            dbg_print(TERM_YELLOW, "Error %d setting RX frequency: %lu Hz\n", resp[3], freq); //error
	}
	else
	{
		dbg_print(TERM_YELLOW, "Malformed reply\n"); //error
	}
}

void dev_set_tx_freq(uint64_t freq)
{
	uint8_t cmd[11];
	cmd[0]=CMD_SET_TX_FREQ;		//RX freq
	*((uint16_t*)&cmd[1])=11;
	*((uint64_t*)&cmd[3])=freq;
	write(fd, cmd, 11);
	
	//wait for device's response
	do
	{
		ioctl(fd, FIONREAD, &uart_byte_cnt);
	}
	while(uart_byte_cnt!=4);
	uint8_t resp[4]={0};
	read(fd, resp, 4);
	
	if(resp[0]==CMD_SET_TX_FREQ && *((uint16_t*)&resp[1])==4)
	{
        if(resp[3]==0)
		    dbg_print(0, "TX frequency: %lu Hz\n", freq); //OK
        else
            dbg_print(TERM_YELLOW, "Error %d setting TX frequency: %lu Hz\n", resp[3], freq); //error
	}
	else
	{
		dbg_print(TERM_YELLOW, "Malformed reply\n"); //error
	}
}

void dev_set_rx_freq_corr(float corr)
{
	uint8_t cmd[7];
	cmd[0]=CMD_SET_RX_FREQ_CORR;	//freq correction
	*((uint16_t*)&cmd[1])=7;
	*((float*)&cmd[3])=corr;
	write(fd, cmd, 7);

	//wait for device's response
	do
	{
		ioctl(fd, FIONREAD, &uart_byte_cnt);
	}
	while(uart_byte_cnt!=4);
	uint8_t resp[4]={0};
	read(fd, resp, 4);
	
	if(resp[0]==CMD_SET_RX_FREQ_CORR && *((uint16_t*)&resp[1])==4)
	{
        if(resp[3]==0)
		    dbg_print(0, "RX frequency correction: %2.1f\n", corr); //OK
        else
            dbg_print(TERM_YELLOW, "Error %d setting RX frequency correction: %2.1f\n", resp[3], corr); //error
	}
	else
	{
		dbg_print(TERM_YELLOW, "Malformed reply\n"); //error
	}
}

void dev_set_tx_freq_corr(float corr)
{
	uint8_t cmd[7];
	cmd[0]=CMD_SET_TX_FREQ_CORR;	//freq correction
	*((uint16_t*)&cmd[1])=7;
	*((float*)&cmd[3])=corr;
	write(fd, cmd, 7);

	//wait for device's response
	do
	{
		ioctl(fd, FIONREAD, &uart_byte_cnt);
	}
	while(uart_byte_cnt!=4);
	uint8_t resp[4]={0};
	read(fd, resp, 4);
	
	if(resp[0]==CMD_SET_TX_FREQ_CORR && *((uint16_t*)&resp[1])==4)
	{
        if(resp[3]==0)
		    dbg_print(0, "TX frequency correction: %2.1f\n", corr); //OK
        else
            dbg_print(TERM_YELLOW, "Error %d setting TX frequency correction: %2.1f\n", resp[3], corr); //error
	}
	else
	{
		dbg_print(TERM_YELLOW, "Malformed reply\n"); //error
	}
}

void dev_set_afc(uint8_t en)
{
	uint8_t cmd[4];
	cmd[0]=CMD_SET_AFC;
	*((uint16_t*)&cmd[1])=4;
	cmd[3]=en?1:0;

	write(fd, cmd, 4);

	//wait for device's response
	do
	{
		ioctl(fd, FIONREAD, &uart_byte_cnt);
	}
	while(uart_byte_cnt!=4);
	uint8_t resp[4]={0};
	read(fd, resp, 4);
	
	if(resp[0]==CMD_SET_AFC && *((uint16_t*)&resp[1])==4)
	{
        if(resp[3]==0)
        {
            if(en)
                dbg_print(0, "AFC enabled"); 
            else
                dbg_print(0, "AFC disabled"); 
            dbg_print(TERM_GREEN, " OK\n"); //OK
        }
        else
        {
            dbg_print(TERM_YELLOW, "Error %d setting AFC\n", resp[3]); //error
        }
	}
	else
	{
		dbg_print(TERM_YELLOW, "Malformed reply\n"); //error
	}
}

void dev_set_tx_power(float power) //powr in dBm
{
	uint8_t cmd[4];
	cmd[0]=CMD_SET_TX_POWER;	//transmit power
	*((uint16_t*)&cmd[1])=4;
	cmd[3]=roundf(power*4.0f);
	write(fd, cmd, 4);

	//wait for device's response
	do
	{
		ioctl(fd, FIONREAD, &uart_byte_cnt);
	}
	while(uart_byte_cnt!=4);
	uint8_t resp[4]={0};
	read(fd, resp, 4);
	
    if(resp[0]==CMD_SET_TX_POWER && *((uint16_t*)&resp[1])==4)
    {
        if(resp[3]==0)
        {
            dbg_print(0, "TX power: %2.2f dBm\n", power); //OK
        }
        else
        {
            dbg_print(TERM_YELLOW, "Error %d setting TX power: %2.2f dBm\n", resp[3], power); //error
        }
    }
    else
    {
        dbg_print(TERM_YELLOW, "Malformed reply\n"); //error
    }
}

void dev_start_rx(void)
{
	uint8_t cmd[4]={CMD_SET_RX, 4, 0, 1}; //start reception
	write(fd, cmd, 4);
}

void dev_stop_rx(void)
{
	uint8_t cmd[4]={CMD_SET_RX, 4, 0, 0}; //stop reception
	write(fd, cmd, 4);
}

int main(int argc, char *argv[])
{
    //we need 3 gpio pin numbers and uart params (device, speed)
    if(argc>=1+6+4)
    {
        //load some ridiculous init values so we can tell, if they have been overwritten
        config.dl_port=0;
        config.ul_port=0;
        config.ctrl_port=0;

        for(int i=0; i<argc; i++)
        {
            if(argv[i][0]=='-')
            {
                //long commands
                //PA_EN GPIO pin number
                if(strstr(argv[i], "pa"))
                {
                    config.pa_en=atoi(argv[i+1]);
                }

                //RST GPIO pin number
                else if(strstr(argv[i], "nrst"))
                {
                    config.nrst=atoi(argv[i+1]);
                    i++;
                }

                //BOOT0 GPIO pin number
                else if(strstr(argv[i], "boot"))
                {
                    config.boot0=atoi(argv[i+1]);
                    i++;
                }

                //uplink port
                /*else if(strstr(argv[i], "ul"))
                {
                    config.ul_port=atoi(argv[i+1]);
                    i++;
                }*/

                //downlink port
                else if(strstr(argv[i], "dl"))
                {
                    config.dl_port=atoi(argv[i+1]);
                    i++;
                }

                //control port
                else if(strstr(argv[i], "ctrl"))
                {
                    config.ctrl_port=atoi(argv[i+1]);
                    i++;
                }

                //short commands
                //reset only
                else if(argv[i][1]=='r')
                {
                    reset_only=1;
                }

                //uart device
                else if(argv[i][1]=='d')
                {
                    char *addr=argv[i+1];
                    if(strlen(addr)>5) //meaningful length?
                    {
                        memcpy(config.uart, addr, strlen(addr));
                        //dbg_print(0, "Setting UART device to %s\n", config.uart);
                        i++;
                    }
                    else
                    {
                        dbg_print(TERM_RED, "UART device address too short.\nExiting.\n");
                        return 1;
                    }
                }

                //uart speed
                else if(argv[i][1]=='s')
                {
                    uint32_t spd=atoi(argv[i+1]);
                    if(spd>=115200U && spd<=1000000U) //meaningful value?
                    {
                        config.uart_rate=spd;
                        //dbg_print(0, "Setting UART speed to %d\n", config.uart_rate);
                        i++;
                    }
                    else
                    {
                        dbg_print(TERM_RED, "Invalid UART speed.\nExiting.\n");
                        return 1;
                    }
                }
            }
        }

        //init GPIOs
        dbg_print(0, "GPIO init");
        uint8_t gpio_err=0;
        gpio_init();
        gpio_err|=gpio_set(config.boot0, 0); //all pins should be at logic low already, but better be safe than sorry
        gpio_err|=gpio_set(config.pa_en, 0);
        gpio_err|=gpio_set(config.nrst, 0);

        if(gpio_err)
        {
            dbg_print(TERM_RED, " error.\nExiting.\n");
            return (int)gpio_err;
        }
        dbg_print(TERM_GREEN, " OK\n");

        usleep(50000U); //50ms
        dbg_print(0, "Device reset");
        if(gpio_set(config.nrst, 1))
        {
            dbg_print(TERM_RED, " error\n");
            return (int)gpio_err;
        }
        else
            dbg_print(TERM_GREEN, " OK\n");

        //init UART
        dbg_print(0, "Initializing device %s at %d", (char*)config.uart, config.uart_rate);
        fd=open((char*)config.uart, O_RDWR | O_NOCTTY | O_SYNC);
        if(fd==0)
        {
            dbg_print(TERM_RED, " error\nExiting.\n");
            return 1;
        }
        
        set_blocking(fd, 0);
        set_interface_attribs(fd, config.uart_rate, 0);
        dbg_print(TERM_GREEN, " OK\n");

        //PING-PONG test
        usleep(500000U); //0.5s for RRU boot-up
        dbg_print(0, "Device's reply to PING");

        dev_ping();
        do
        {
            ioctl(fd, FIONREAD, &uart_byte_cnt);
        }
        while(uart_byte_cnt!=7);
        uint8_t ping_test[7]={0};
        read(fd, ping_test, 7);

        uint32_t dev_err=*((uint32_t*)&ping_test[3]);
        if(ping_test[0]==CMD_PING && *((uint16_t*)&ping_test[1])==7 && dev_err==0)
            dbg_print(TERM_GREEN, " OK\n");
        else
        {
            dbg_print(TERM_YELLOW, " error code: 0x%04X\n", dev_err);
            return 1;
        }

        //if required - exit after reset
        if(reset_only)
            return 0;

        //set PA_EN=1
        gpio_set(config.pa_en, 1);

        //ZMQ stuff
        void *zmq_ctx = zmq_ctx_new();
        void *zmq_dlink = zmq_socket(zmq_ctx, ZMQ_PUB);
        void *zmq_ctrl = zmq_socket(zmq_ctx, ZMQ_REP);

        char tmp[128];
        if(config.dl_port!=0)
        {
            sprintf(tmp, "tcp://*:%d", config.dl_port);
            dbg_print(0, "ZeroMQ downlink: %s", tmp);
            if(zmq_bind(zmq_dlink, tmp)==0)
                dbg_print(TERM_GREEN, " OK\n");
            else
                dbg_print(TERM_RED, " ERROR\n");
        }
        else
        {
            dbg_print(TERM_RED, "ZeroMQ downlink port setting missing.\nExiting.\n", tmp);
            return 1;
        }
        
        if(config.ctrl_port!=0)
        {
            sprintf(tmp, "tcp://*:%d", config.ctrl_port);
            dbg_print(0, "ZeroMQ control: %s", tmp);
            if(zmq_bind(zmq_ctrl, tmp)==0)
                dbg_print(TERM_GREEN, " OK\n");
            else
                dbg_print(TERM_RED, " ERROR\n");
        }
        else
        {
            dbg_print(TERM_RED, "ZeroMQ control port setting missing.\nExiting.\n", tmp);
            return 1;
        }

        //do actual work
        dbg_print(0, "Listening for CARI commands...\n");
        while(1)
        {
            //check UART for data
            ioctl(fd, FIONREAD, &uart_byte_cnt);
            if(uart_byte_cnt>0)
            {
                static uint32_t total_uart_bytes=0;

                if(total_uart_bytes+uart_byte_cnt>sizeof(uart_buff))
                {
                    dbg_print(TERM_RED, "UART buffer overflow.\nExiting.\n");
                    return 1;
                }

                read(fd, &uart_buff[total_uart_bytes], uart_byte_cnt);
                total_uart_bytes+=uart_byte_cnt;

                if(*((uint16_t*)&uart_buff[1])==total_uart_bytes && total_uart_bytes>=3)
                {
                    //process commands
                    //baseband data chunk
                    if(uart_buff[0]==CMD_STREAM_DATA)
                    {
                        //rip off the header
                        zmq_send(zmq_dlink, &uart_buff[3], total_uart_bytes-3, ZMQ_DONTWAIT);
                    }

                    //something else
                    else if(uart_buff[0]>=CMD_SET_RX_FREQ && uart_buff[0]<=CMD_SUB_CONNECT)
                    {
                        dbg_print(0, "REP %02X for CMD %d\n", uart_buff[3], uart_buff[0]);
                        zmq_send(zmq_ctrl, uart_buff, 4, ZMQ_DONTWAIT);
                    }

                    memset(uart_buff, 0, sizeof(uart_buff));
                    total_uart_bytes=0;
                }
            }

            //check ZMQ REP/REQ
            zmq_byte_cnt=zmq_recv(zmq_ctrl, zmq_buff, sizeof(zmq_buff), ZMQ_DONTWAIT);
            if(zmq_byte_cnt>0)
            {
                if(*((uint16_t*)&zmq_buff[1])==zmq_byte_cnt)
                {
                    if(zmq_buff[0]==CMD_SET_RX_FREQ)
                    {
                        dev_set_rx_freq(*((uint64_t*)&zmq_buff[3]));
                    }

                    else if(zmq_buff[0]==CMD_SET_TX_FREQ)
                    {
                        dev_set_tx_freq(*((uint64_t*)&zmq_buff[3]));
                    }

                    else if(zmq_buff[0]==CMD_SET_RX)
                    {
                        if(zmq_buff[3])
                        {
                            dev_start_rx();
                            uint8_t rep[4]={CMD_SET_RX, 4, 0, ERR_OK};
                            dbg_print(0, "REP %02X for CMD %d\n", ERR_OK, CMD_SET_RX);
                            zmq_send(zmq_ctrl, rep, 4, ZMQ_DONTWAIT);
                        }
                        else
                            dev_stop_rx();
                    }
                }
            }

            //check ZMQ PUB for data
            //zmq_byte_cnt=zmq_recv(zmq_uplink, zmq_buff, sizeof(zmq_buff), ZMQ_DONTWAIT);
            if(zmq_byte_cnt>0)
            {
                ;
            }
        }
    }
    else
    {
        dbg_print(TERM_RED, "Not enough parameters.\nExiting.\n");
        return 1;
    }

    return 0;
}
