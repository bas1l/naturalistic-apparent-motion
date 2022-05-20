#include <iostream>
#include <sys/mman.h>
#include <queue>
#include <cstdlib>
#include <chrono>
#include <ctime>
#include <random>
#include <sys/ioctl.h>


#include "ad5383.h"
using namespace std;

int main(int argc, char *argv[])
{
    std::cout << "Initialising all actuators..." << std::flush;

    struct timespec t;
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    
    /**************** C'EST AVEC CETTE OPTION QUE TOUT BUG *********************
    if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
            perror("sched_setscheduler failed");
            exit(-1);
    }
    ***************************************************************************/
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
            perror("mlockall failed");
            exit(-2);
    }

    
    AD5383 ad;
    ad.spi_open();
    ad.configure();
    std::vector<std::vector<uint16_t> > values(AD5383::num_channels);
    
    for(int j = 0; j < AD5383::num_channels; ++j) {
        values[j].push_back(2048);
    }
    
    long ms = 900;
    ad.execute_trajectory(values, ms *1000000);
    //std::cout << "Neutral::overruns : " << std::dec << ad.execute_trajectory(values, ms *1000000) << std::endl;
   
    std::cout << "done." << std::endl;
    ad.spi_close();
  
    return 0;
}
