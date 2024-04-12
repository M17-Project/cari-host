/*
 * cari-host.c
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

//CARI commands
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
	uint16_t me_port;
    uint16_t ctrl_port;
} config;

int uart_byte_cnt=0;                //how many bytes are available on UART
int zmq_byte_cnt=0;                 //how many bytes are available over ZMQ

uint8_t uart_buff[1000];
uint8_t zmq_buff[10000];

uint8_t reset_only=0;
uint8_t ulink_connected=0;			//uplink ZMQ connected?

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
}

void dev_set_tx_freq(uint64_t freq)
{
	uint8_t cmd[11];
	cmd[0]=CMD_SET_TX_FREQ;		//RX freq
	*((uint16_t*)&cmd[1])=11;
	*((uint64_t*)&cmd[3])=freq;
	write(fd, cmd, 11);
}

void dev_set_rx_freq_corr(float corr)
{
	uint8_t cmd[7];
	cmd[0]=CMD_SET_RX_FREQ_CORR;	//freq correction
	*((uint16_t*)&cmd[1])=7;
	*((float*)&cmd[3])=corr;
	write(fd, cmd, 7);
}

void dev_set_tx_freq_corr(float corr)
{
	uint8_t cmd[7];
	cmd[0]=CMD_SET_TX_FREQ_CORR;	//freq correction
	*((uint16_t*)&cmd[1])=7;
	*((float*)&cmd[3])=corr;
	write(fd, cmd, 7);
}

void dev_set_afc(uint8_t en)
{
	uint8_t cmd[4];
	cmd[0]=CMD_SET_AFC;
	*((uint16_t*)&cmd[1])=4;
	cmd[3]=en?1:0;
	write(fd, cmd, 4);
}

void dev_set_tx_power(uint8_t power) //powr in dBm
{
	uint8_t cmd[4];
	cmd[0]=CMD_SET_TX_POWER;	//transmit power
	*((uint16_t*)&cmd[1])=4;
	cmd[3]=power;
	write(fd, cmd, 4);
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

void dev_send_baseband(const uint8_t *samples, uint16_t len)
{
	len+=3;
	uint8_t h_buff[1000]={CMD_STREAM_DATA, len&0xFF, len>>8}; //header
	memcpy(&h_buff[3], samples, len-3);
	write(fd, h_buff, len);
}

void dev_get_meas(void)
{
	uint8_t cmd[3]={CMD_GET_MEAS, 3, 0}; //get measurements
	write(fd, cmd, 3);
}

int main(int argc, char *argv[])
{
    //we need 3 gpio pin numbers and uart params (device, speed)
    if(argc>=1+6+4)
    {
        //load some ridiculous init values so we can tell, if they have been overwritten
        config.dl_port=0;
        config.ul_port=0;
		config.me_port=0;
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

                //downlink port
                else if(strstr(argv[i], "dl"))
                {
                    config.dl_port=atoi(argv[i+1]);
                    i++;
                }

				//uplink port
                /*else if(strstr(argv[i], "ul"))
                {
                    config.ul_port=atoi(argv[i+1]);
                    i++;
                }*/

				//measurements port
				else if(strstr(argv[i], "me"))
                {
                    config.me_port=atoi(argv[i+1]);
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
        
        set_interface_attribs(fd, config.uart_rate, 0); //no parity, no blocking
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
		void *zmq_meas = zmq_socket(zmq_ctx, ZMQ_PUB);
		void *zmq_ulink = zmq_socket(zmq_ctx, ZMQ_SUB);
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

        if(config.me_port!=0)
        {
            sprintf(tmp, "tcp://*:%d", config.me_port);
            dbg_print(0, "ZeroMQ telemetry: %s", tmp);
            if(zmq_bind(zmq_meas, tmp)==0)
                dbg_print(TERM_GREEN, " OK\n");
            else
                dbg_print(TERM_RED, " ERROR\n");
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
                    //basic GET commands
                    if(uart_buff[0]<CMD_SUB_CONNECT)
                    {
                        zmq_send(zmq_ctrl, uart_buff, total_uart_bytes, ZMQ_DONTWAIT);
						if(uart_buff[0]!=CMD_PING)
							dbg_print(0, "<- CMD %02X, REP %02X\n", uart_buff[0], uart_buff[3]);
						else
							dbg_print(0, "<- CMD %02X, REP %08X\n", uart_buff[0], *((uint32_t*)&uart_buff[3]));
                    }

					//baseband data chunk
                    else if(uart_buff[0]==CMD_STREAM_DATA)
                    {
                        //rip off the header and push down the ZMQ
                        zmq_send(zmq_dlink, &uart_buff[3], total_uart_bytes-3, ZMQ_DONTWAIT);
						//send CMD_GET_MEAS after receiving RX baseband data
						//dev_get_meas();
                    }

					//telemetry
					else if(uart_buff[0]==CMD_GET_MEAS)
                    {
						zmq_send(zmq_meas, &uart_buff[3], 3, ZMQ_DONTWAIT);
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
						uint64_t freq=*((uint64_t*)&zmq_buff[3]);
						dev_set_rx_freq(freq);
						//dbg_print(0, "-> CMD %02X, VAL %ld\n", CMD_SET_RX_FREQ, freq);
                    }

                    else if(zmq_buff[0]==CMD_SET_TX_FREQ)
                    {
						uint64_t freq=*((uint64_t*)&zmq_buff[3]);
                        dev_set_tx_freq(freq);
						//dbg_print(0, "-> CMD %02X, VAL %ld\n", CMD_SET_TX_FREQ, freq);
                    }

					else if(zmq_buff[0]==CMD_SET_RX_FREQ_CORR)
                    {
						float corr=*((float*)&zmq_buff[3]);
                        dev_set_rx_freq_corr(corr);
						//dbg_print(0, "-> CMD %02X, VAL %ld\n", CMD_SET_RX_FREQ_CORR, corr);
                    }

					else if(zmq_buff[0]==CMD_SET_TX_FREQ_CORR)
                    {
						float corr=*((float*)&zmq_buff[3]);
                        dev_set_tx_freq_corr(corr);
						//dbg_print(0, "-> CMD %02X, VAL %ld\n", CMD_SET_TX_FREQ_CORR, corr);
                    }

					else if(zmq_buff[0]==CMD_SET_TX_POWER)
                    {
						uint8_t pwr=zmq_buff[3];
                        dev_set_tx_power(pwr);
						//dbg_print(0, "-> CMD %02X, VAL %ld\n", CMD_SET_TX_POWER, pwr);
                    }

					else if(zmq_buff[0]==CMD_SET_AFC)
                    {
						uint8_t afc=zmq_buff[3];
                        dev_set_afc(afc);
						//dbg_print(0, "-> CMD %02X, VAL %ld\n", CMD_SET_AFC, afc);
                    }

                    else if(zmq_buff[0]==CMD_SET_RX)
                    {
                        if(zmq_buff[3])
                        {
                            dev_start_rx();
                            uint8_t rep[4]={CMD_SET_RX, 4, 0, ERR_OK};
							zmq_send(zmq_ctrl, rep, 4, ZMQ_DONTWAIT);
							dbg_print(0, "-> CMD %02X, VAL %02X\n", CMD_SET_RX, zmq_buff[3]);
                            dbg_print(0, "<- CMD %02X, REP %02X\n", CMD_SET_RX, ERR_OK);
                        }
                        else
                            dev_stop_rx();
                    }

					else if(zmq_buff[0]==CMD_SUB_CONNECT)
					{
						ulink_connected=!zmq_connect(zmq_ulink, (char*)&zmq_buff[3]);
						dbg_print(0, "-> CMD %02X, VAL %s\n", CMD_SUB_CONNECT, (char*)&zmq_buff[3]);

						uint8_t rep[4]={CMD_SUB_CONNECT, 4, 0, 0};
						if(ulink_connected)
						{
							zmq_setsockopt(zmq_ulink, ZMQ_SUBSCRIBE, "", 0); //subscribe to everything
							rep[3]=ERR_OK;
							zmq_send(zmq_ctrl, rep, 4, ZMQ_DONTWAIT);
							dbg_print(0, "<- CMD %02X, RET %02X\n", rep[0], rep[3]);
						}
						else
						{
							rep[3]=ERR_ZMQ_CONN;
							zmq_send(zmq_ctrl, rep, 4, ZMQ_DONTWAIT);
							dbg_print(0, "<- CMD %02X, RET %02X\n", rep[0], rep[3]);
						}
					}

					//this command doesnt need to be issued by the CARI controller
					//instead, this should be managed cyclically by the host
					/*else if(zmq_buff[0]==CMD_GET_MEAS)
                    {
						dev_get_meas();
						//dbg_print(0, "-> CMD %02X\n", CMD_GET_MEAS);
                    }*/
                }
            }

            //check ZMQ PUB for data
			//if(ulink_connected)
            zmq_byte_cnt=zmq_recv(zmq_ulink, zmq_buff, sizeof(zmq_buff), ZMQ_DONTWAIT);
            if(zmq_byte_cnt>0)
            {
				//send baseband samples over UART for transmission
                dev_send_baseband(zmq_buff, zmq_byte_cnt);
            }

			//fetch measurements every 5 seconds
			/*static uint8_t measure=1;
			if(time(NULL)%5==0 && measure)
			{
				dev_get_meas();
				measure=0;
			}
			else if(time(NULL)%5!=0)
				measure=1;*/
        }
    }
    else
    {
        dbg_print(TERM_RED, "Not enough parameters.\nExiting.\n");
        return 1;
    }

    return 0;
}
