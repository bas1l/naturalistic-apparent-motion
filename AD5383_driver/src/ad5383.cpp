#include <stdio.h>
#include <time.h>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <bitset>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/timerfd.h>

#include "ad5383.h"
using namespace std;


AD5383::AD5383() : _spi_fd(0) {
    for(int i = 0; i < num_channels; ++i)
    {
        _data_neutral[i] = AD5383_DEFAULT_NEUTRAL;
    }
}


AD5383::~AD5383() 
{
	spi_close();
}


void AD5383::configure() {
    for(int i = 0; i < num_channels; ++i)
    {
        set_channel_properties(i, _data_neutral[i]);
    }
}


void AD5383::set_channel_properties(uint8_t channel_id, uint16_t channel_neutral_value) {
    _data_neutral[channel_id] = channel_neutral_value;

    spi_xfer(AD5383_REG_A, AD5383_WRITE, channel_id, AD5383_REG_OFFSET,  AD5383_DEFAULT_OFFSET);
    spi_xfer(AD5383_REG_A, AD5383_WRITE, channel_id, AD5383_REG_GAIN,	AD5383_DEFAULT_GAIN);
    spi_xfer(AD5383_REG_A, AD5383_WRITE, channel_id, AD5383_REG_DATA,	channel_neutral_value);
}


bool AD5383::spi_open(const char *port_name, uint8_t transfer_mode, uint8_t bit_justification, uint8_t bits_per_word, uint32_t max_speed) {
    
    if(!(_spi_fd = open(port_name, O_RDWR)))
    {
        perror("spi_open/open");
        _spi_fd = 0;
        return false;
    }

    if(ioctl(_spi_fd, SPI_IOC_WR_MODE, &transfer_mode) != 0)
    {
        perror("spi_open/SPI_IOC_WR_MODE");
        _spi_fd = 0;
        return false;
    }

    if(ioctl(_spi_fd, SPI_IOC_WR_LSB_FIRST, &bit_justification) != 0)
    {
        perror("spi_open/SPI_IOC_WR_LSB_FIRST");
        _spi_fd = 0;
        return false;
    }

    if(ioctl(_spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) != 0)
    {
        perror("spi_open/SPI_IOC_WR_BITS_PER_WORD");
        _spi_fd = 0;
        return false;
    }

    if(ioctl(_spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &max_speed) != 0)
    {
        perror("spi_open/SPI_IOC_WR_MAX_SPEED_HZ");
        _spi_fd = 0;
        return false;
    }

    return true;
}


bool AD5383::spi_close() {
    if(_spi_fd && close(_spi_fd) != 0)
	{
		perror("spi_close/close");
		return false;
	}
	_spi_fd=0;
    
    return true;
}


bool AD5383::gpio_open() {
    _gpio_fd = open("/sys/class/gpio/export", O_WRONLY);
    if(!_gpio_fd)
    {
        perror("gpio_open/export");
        _gpio_fd = 0;
        return false;
    }
    write(_gpio_fd,"24",2);
    close(_gpio_fd);

    _gpio_fd = open("/sys/class/gpio/gpio24/direction", O_WRONLY);
    if(!_gpio_fd)
    {
        perror("gpio_open/direction");
        _gpio_fd = 0;
        return false;
    }
    write(_gpio_fd,"out",3);
    close(_gpio_fd);

    _gpio_fd = open("/sys/class/gpio/gpio24/value", O_WRONLY);
    if(!_gpio_fd)
    {
        perror("gpio_open/value");
        _gpio_fd = 0;
        return false;
    }
    write(_gpio_fd,"1",1);
}


bool AD5383::gpio_close() {
    if(_gpio_fd)
    {
        close(_gpio_fd);
    }

    _gpio_fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if(!_gpio_fd)
    {
        perror("gpio_close/unexport");
        _gpio_fd = 0;
        return false;
    }
    write(_gpio_fd,"24",2);
    close(_gpio_fd);
}


int AD5383::execute_trajectory(const std::vector<std::vector<uint16_t> >& values, long period_ns) {
    bool keep_running;
    int ret;
    unsigned long long missed = 0;
    int overruns = 0;
    int value_idx = 0;

    struct timespec ts = {
        .tv_sec = 0,
        .tv_nsec = period_ns
    };

    struct itimerspec its = {
        .it_interval = ts,
        .it_value = ts
    };

    if(values.size() > num_channels)
        throw std::runtime_error("Trajectory vector is bigger than number of channels");

    _timer_fd = timerfd_create(CLOCK_REALTIME, 0);
    if(_timer_fd == -1)
    {
        perror("execute_trajectory/timerfd_create");
        _timer_fd = 0;
        return overruns;
    }
    if(timerfd_settime(_timer_fd, 0, &its, NULL) == -1)
    {
        perror("execute_trajectory/timerfd_settime");
        close(_timer_fd);
        return overruns;
    }

    do
    {
        ret = read(_timer_fd, &missed, sizeof (missed));
        if (ret == -1)
        {
            perror("execute_trajectory/read");
            close(_timer_fd);
            return overruns;
        }
        overruns += missed - 1;

        keep_running = false;
        for(unsigned int channel = 0; channel < values.size(); ++channel)
        {
            if(values[channel].size() > value_idx)
            {
                keep_running = true;
                spi_xfer(AD5383_REG_A,AD5383_WRITE,channel,AD5383_REG_DATA,values[channel][value_idx]);
            }
        }
        ++value_idx;

    } while(keep_running);
    close(_timer_fd);

    return overruns;
}



int AD5383::execute_selective_trajectory(std::multimap<uint8_t,std::vector<uint16_t>> wfLetter, long period_ns)
{   
    if(wfLetter.size() > num_channels)
        throw std::runtime_error("Trajectory vector is bigger than number of channels");
    
    std::multimap<uint8_t, std::vector<uint16_t>>::iterator it = wfLetter.begin();
    bool keep_running;
    int ret;
    unsigned long long missed = 0;
    int overruns = 0;
    int value_idx = 0;
    
    struct timespec ts = {
        .tv_sec = 0,
                .tv_nsec = period_ns
    };

    struct itimerspec its = {
        .it_interval = ts,
                .it_value = ts
    };
    
    
    _timer_fd = timerfd_create(CLOCK_REALTIME, 0);
    if(_timer_fd == -1)
    {
        perror("execute_trajectory/timerfd_create");
        _timer_fd = 0;
        return overruns;
    }
    if(timerfd_settime(_timer_fd, 0, &its, NULL) == -1)
    {
        perror("execute_trajectory/timerfd_settime");
        close(_timer_fd);
        return overruns;
    }

    do
    {
        ret = read(_timer_fd, &missed, sizeof (missed));
        if (ret == -1)
        {
            perror("execute_trajectory/read");
            close(_timer_fd);
            return overruns;
        }
        overruns += missed - 1;

        keep_running = false;
        
        for (it=wfLetter.begin(); it!=wfLetter.end(); ++it)
        {
            if(it->second.size() > value_idx)
            {
                keep_running = true;
                spi_xfer(AD5383_REG_A,AD5383_WRITE,it->first,AD5383_REG_DATA,it->second[value_idx]);
            }
        }
        ++value_idx;

    } while(keep_running);
    struct timespec ts_wait, ts_rem;
    ts_wait.tv_sec = 0;
    ts_wait.tv_nsec = 200 * 1000000;
    
    nanosleep(&ts_wait, &ts_rem);
    
    close(_timer_fd);

    return overruns;
}


void AD5383::execute_single_target(const std::vector<uint16_t> values) {
    if(values.size() > num_channels)
        throw std::runtime_error("Trajectory vector is bigger than number of channels");

    for(unsigned int channel = 0; channel < values.size(); ++channel)
    {
        spi_xfer(AD5383_REG_A,AD5383_WRITE,channel,AD5383_REG_DATA,values[channel]);
    }
}


void AD5383::execute_single_channel(uint16_t value, int channel) {
    spi_xfer(AD5383_REG_A, AD5383_WRITE, channel, AD5383_REG_DATA, value);
}

uint16_t AD5383::read_channel(int channel) {
    // ask AD5383 to send data during the next SPI message
    spi_xfer(AD5383_REG_A, AD5383_READ, channel, AD5383_REG_DATA, 0);
    // nop condition to read the output value
    spi_xfer(0,0,0,0,0);
    
    uint16_t ret = (_in_buffer[1]<<8) + _in_buffer[2];

    return (ret&0x3FFC)>>2;
}


uint8_t* AD5383::format_msg(bool reg_b, bool reg_read, uint8_t reg_channels, uint8_t reg_addr, uint16_t reg_data) {
    _out_buffer[0] = (!!reg_b << 7) | (!!reg_read << 6) | (reg_channels & 0x1F);
    _out_buffer[1] = (reg_addr << 6) | ((reg_data >> 6) & 0x3F);
    _out_buffer[2] = reg_data << 2;

    _in_buffer[0] = 0;
    _in_buffer[1] = 0;
    _in_buffer[2] = 0;

    return _out_buffer;
}


uint16_t AD5383::spi_xfer(bool reg_b, bool reg_read, uint8_t reg_channels, uint8_t reg_addr, uint16_t reg_data) {
    format_msg(reg_b,reg_read,reg_channels,reg_addr,reg_data);
    return spi_xfer();
}


uint16_t AD5383::spi_xfer() {
    uint16_t ret = 0;

    if(!_spi_fd)
        return 0;

    struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)_out_buffer,
		.rx_buf = (unsigned long)_in_buffer,
		
		.len = ARRAY_SIZE(_out_buffer),
		.speed_hz = AD5383_SPI_SELECT_CLOCK_HZ,
				
		.delay_usecs = 0,
		.bits_per_word = AD5383_SPI_SELECT_BITS,
		.cs_change = false
	};
    
    if(ioctl(_spi_fd, SPI_IOC_MESSAGE(1), &tr) == -1)
    {
        perror("spi_xfer/ioctl");
        return ret;
    }
    /*
    std::cout << "out : 0b " << std::bitset<8>(_out_buffer[0]) << " " << std::bitset<8>(_out_buffer[1]) << " " << std::bitset<8>(_out_buffer[2]) << std::endl;
    std::cout << "in  : 0b " << std::bitset<8>(_in_buffer[0]) << " " << std::bitset<8>(_in_buffer[1]) << " " << std::bitset<8>(_in_buffer[2]) << std::endl;
    */
    ret = (_in_buffer[2] >> 2) | ((_in_buffer[1] & 0x3f) << 6);
    return ret;
}


void AD5383::latch() {
    if(_gpio_fd)
    {
        struct timespec ts_wait, ts_rem;
        ts_wait.tv_sec = 0;
        ts_wait.tv_nsec = 20;

        write(_gpio_fd,"0",1);
        nanosleep(&ts_wait, &ts_rem);
        write(_gpio_fd,"1",1);
    }
}
