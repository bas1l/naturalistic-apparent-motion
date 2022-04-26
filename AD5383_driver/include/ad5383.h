
#ifndef AD5383_H_
#define AD5383_H_

#include <map>
#include <stdint.h>
#include <vector>
#include <linux/spi/spidev.h>

#include "ad5383_defines.h"




class AD5383
{
public:
    static constexpr int num_channels = 31;

    AD5383();
    ~AD5383();

    /**
     * @brief Initializes channels properties with default parameters
     */
    void configure();

    /**
     * @brief Set a channel's neutral values and
     * @param channel_id Channel id (from 1 to num_channels)
     * @param channel_neutral_value Neutral value (from 0 to 4095)
     */
    void set_channel_properties(uint8_t channel_id, uint16_t channel_neutral_value);

    /**
     * @brief spi_open
     * @param port_name
     * @param transfer_mode
     * @param bit_justification
     * @param bits_per_word
     * @param max_speed
     * @return
     */
    bool spi_open(const char* port_name = "/dev/spidev0.0", 
    			  uint8_t transfer_mode = SPI_MODE_2, 				uint8_t bit_justification = 0, 
				  uint8_t bits_per_word = AD5383_SPI_SELECT_BITS, 	uint32_t max_speed = AD5383_SPI_SELECT_CLOCK_HZ);

    /**
     * @brief spi_close
     * @return
     */
    bool spi_close();

    /**
     * @brief Configures the digital output used to latch data on the DAC outputs, this feature currently doesn't work
     * @return
     */
    bool gpio_open();

    /**
     * @brief gpio_close
     * @return
     */
    bool gpio_close();

    /**
     * @brief Starts a timer pushing values to the dac output at a period given by period_ns (ns)
     * @param values Vector containing values for each channel, its size should be between 0 and num_channels. Each subvector contains output values (between 0 and 4095) for a channel at a given time
     * @param period_ns Period between updates (ns)
     * @return The number of overruns (missed timer ticks)
     */
    int execute_trajectory(const std::vector<std::vector<uint16_t> >& values, long period_ns);
    
    
    /**
     * @brief Starts a timer pushing values, related to the idChannels, to the dac output at a period given by period_ns (ns)
     * @param values Vector containing values for each channel, its size should be between 0 and num_channels. Each subvector contains output values (between 0 and 4095) for a channel at a given time
     * @param channels Vector containing idChannel for each vectors of 'Values'
     * @param period_ns Period between updates (ns)
     * @return The number of overruns (missed timer ticks)
     */
    int execute_selective_trajectory(std::multimap<uint8_t,std::vector<uint16_t>> wfLetter, long period_ns);

    
    /**
     * @brief Updates the dac outputs for all channels
     * @param values (between 0 and 4095)
     */
    void execute_single_target(const std::vector<uint16_t> values);

    /**
     * @brief Updates the dac output for a particular channel
     * @param value (between 0 and 4095)
     */
    void execute_single_channel(uint16_t value, int channel);

    /**
     * @brief Read one channel
     * @param channel (between 0 and 4095)
     * @return the voltage value of the channel
     */
    uint16_t read_channel(int channel);

private:
    uint8_t* format_msg(bool reg_b, bool reg_read, uint8_t reg_channels, uint8_t reg_addr, uint16_t reg_data);
    uint16_t spi_xfer(bool reg_b, bool reg_read, uint8_t reg_channels, uint8_t reg_addr, uint16_t reg_data);
    uint16_t spi_xfer();

    void latch(); //LDAC

    uint16_t _data_neutral[num_channels];

    uint8_t _in_buffer[3];
    uint8_t _out_buffer[3];
    struct spi_ioc_transfer _ioc_xfer;

    int _spi_fd;
    int _gpio_fd;
    int _timer_fd;
};
#endif // AD5383_H_
