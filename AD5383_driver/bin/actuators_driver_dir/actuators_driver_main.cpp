
#include <iostream>     // std::cout, std::dec, std::hex, std::oct

// ad5383 header
#include "ad5383.h"

using namespace std;

int main(int argc, char *argv[])
{
    AD5383 ad;
    int overruns, refresh_rate, periodicity_message_ns;
    double periodicity_message_ms;

    overruns = 0;
    refresh_rate = 5000;
    periodicity_message_ms  = 1000/(double) refresh_rate;
    periodicity_message_ns  = periodicity_message_ms * 1000000; // * ns

    if (!ad.spi_open()) return 1;
    ad.configure();

    std::vector<std::vector<uint16_t> > values(AD5383::num_channels);
    for(int j = 0; j < AD5383::num_channels; ++j) {
        values[j].push_back(2048);
    }    
    std::cout << "Neutral::overruns : " << std::dec << ad.execute_trajectory(values, periodicity_message_ns) << std::endl;


    std::vector<uint8_t> channels(6, 0);
    ad.continuous_trajectory(channels, periodicity_message_ns);
    
    return 0;
}









/**
    // The goal of this function is to use the letters putted by the other
    // thread, one by one, and play them consecutively.
    while(!is_jobdone())
    {
		signal4keypressed();
    	copySentences(&letters);
    	
        // if last char is a space, then a word is finished
        if ((letters.front() == '+') || (letters.front() == '-'))
        {
            letters.pop_front();
            continue;
        }
        else if (prosody == PROSODY_SENTENCE&& letters.end() != std::find(letters.begin(), letters.end(), '.'))
        {
        	while (!letters.empty())
			{
				if (letters.front() == ' ')
				{
					letters.pop_front();
    				std::this_thread::sleep_for(std::chrono::milliseconds(PROSODY_WORD_DELAY));
				}
				else if (letters.front() == '.')
				{
					letters.pop_front();
    				std::this_thread::sleep_for(std::chrono::milliseconds(PROSODY_SENTENCE_DELAY));
				}
				else
				{
					values = alph->getl(letters.front());
					int ovr = ad.execute_selective_trajectory(values, durationRefresh_ns);
					if (ovr)
					{
						overruns += ovr;
					}
					values.clear();
					letters.pop_front();
    				std::this_thread::sleep_for(std::chrono::milliseconds(PROSODY_LETTER_DELAY));
				}
			}
        }
        **/