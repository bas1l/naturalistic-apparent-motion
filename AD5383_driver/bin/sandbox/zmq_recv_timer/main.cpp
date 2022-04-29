#include <iostream>
#include <string.h>

#include <unistd.h> // Clock management
#include <sys/timerfd.h> // Clock management
#include <zmq_addon.hpp> // socket communication with Python

#include "ad5383.h" // ad5383 header

std::string stop_trigger("Goodbye");

//int continuous_trajectory(std::vector<uint8_t> channels, long period_ns);
bool data_message(zmq::socket_t * sub, std::vector<uint16_t> * data);
void socket_only();
int socket_and_timer();
int socket_and_timer_and_control();


/*
 * https://brettviren.github.io/cppzmq-tour/index.html
 */
int main() {
  //int overruns = socket_and_timer();
  int overruns = socket_and_timer_and_control();
  std::cout << "[CPP_recv]:: avg overruns : " << std::dec << overruns << std::endl;

  return 0;
}



void socket_only() {
    zmq::context_t ctx(1);
    zmq::socket_t subscriber(ctx, zmq::socket_type::sub);

    subscriber.connect("tcp://localhost:5556");
    // opens ALL envelopes
    subscriber.set(zmq::sockopt::subscribe, "");

    std::cout << "[CPP]: Listening..." << std::endl;
    while (1) {
        std::vector<uint16_t> data(7);
        if (data_message(&subscriber, &data) == 1)
            break;
            
        std::cout << "[CPP_recv] Received: " << std::flush;
        for (auto& it : data) {
          std::cout << it << "," << std::flush;
        }
        std::cout << std::endl;

    }
    std::cout << "[CPP]: Done." << std::endl;
}



int socket_and_timer() {
    zmq::context_t ctx(1);
    zmq::socket_t subscriber(ctx, zmq::socket_type::sub);
    
    std::vector<uint16_t> data(7);
    unsigned long long missed = 0;
    long period_ms = 1;
    int _timer_fd;
    int ret;
    int overruns = 0;
    int value_idx = 0;
    bool keep_running;

    subscriber.connect("tcp://localhost:5556");
    subscriber.set(zmq::sockopt::subscribe, ""); // opens ALL envelopes

    struct timespec ts = {
      .tv_sec = 0,
      .tv_nsec = period_ms * 100000
    };

    struct itimerspec its = {
      .it_interval = ts,
      .it_value = ts
    };
    
    _timer_fd = timerfd_create(CLOCK_REALTIME, 0);
    if(_timer_fd == -1) {
        perror("execute_trajectory/timerfd_create");
        _timer_fd = 0;
        return overruns;
    }
    if(timerfd_settime(_timer_fd, 0, &its, NULL) == -1) {
        perror("execute_trajectory/timerfd_settime");
        close(_timer_fd);
        return overruns;
    }

    std::cout << "[CPP_recv]: Listening..." << std::endl;
    keep_running = true;
    int first = 2;
    do {
        ret = read(_timer_fd, &missed, sizeof (missed));
        if (ret == -1) {
            perror("execute_trajectory/read");
            close(_timer_fd);
            return overruns;
        }
        if (first)
          first--;
        else
          overruns += missed - 1;
        

        if (data_message(&subscriber, &data) == 1)
          keep_running = false;
        else if (!first)
          value_idx++;

            
        /*std::cout << "[CPP_recv] Received: " << std::flush;
        for (auto& it : data) {
          std::cout << it << "," << std::flush;
        }
        std::cout << std::endl;*/
    } while(keep_running);

    close(_timer_fd);
    
    std::cout << "[CPP_recv]: number of messages received: " << value_idx <<std::endl;
    std::cout << "[CPP_recv]: Done." << std::endl;
    
    return overruns/value_idx;
}



int socket_and_timer_and_control() {
    AD5383 ad;
    zmq::context_t ctx(1);
    zmq::socket_t subscriber(ctx, zmq::socket_type::sub);
    
		std::vector<uint8_t>	dacChannel{18, 15, 14, 23, 19, 22};
    std::vector<uint16_t> data(7);
    unsigned long long missed = 0;
    long period_ms = 1;
    int _timer_fd;
    int ret;
    int overruns = 0;
    int value_idx = 0;
    bool keep_running;

    if (!ad.spi_open()) return 1;
    ad.configure();

    subscriber.connect("tcp://localhost:5556");
    subscriber.set(zmq::sockopt::subscribe, ""); // opens ALL envelopes

    struct timespec ts = {
      .tv_sec = 0,
      .tv_nsec = period_ms * 100000
    };

    struct itimerspec its = {
      .it_interval = ts,
      .it_value = ts
    };
    
    _timer_fd = timerfd_create(CLOCK_REALTIME, 0);
    if(_timer_fd == -1) {
        perror("execute_trajectory/timerfd_create");
        _timer_fd = 0;
        return overruns;
    }
    if(timerfd_settime(_timer_fd, 0, &its, NULL) == -1) {
        perror("execute_trajectory/timerfd_settime");
        close(_timer_fd);
        return overruns;
    }

    std::cout << "[CPP_recv]: Listening..." << std::endl;
    keep_running = true;
    int first = 2;
    do {
        ret = read(_timer_fd, &missed, sizeof (missed));
        if (ret == -1) {
            perror("execute_trajectory/read");
            close(_timer_fd);
            return overruns;
        }
        if (first)
          first--;
        else
          overruns += missed - 1;
        

        if (data_message(&subscriber, &data) == 1) {
          std::cout << "[CPP_recv] STOP " << std::endl;
          keep_running = false;
        }
        else{
          if (1) {
            std::cout << "[CPP_recv] Received: " << std::flush;
            for (auto& it : data) { std::cout << it << "," << std::flush;}
            std::cout << std::endl;
          }
          
          for(int c=0; c<dacChannel.size(); c++) {
            ad.spi_xfer_public(AD5383_REG_A,AD5383_WRITE,dacChannel[c],AD5383_REG_DATA,data[c+1]);
          }

          if (!first) 
            value_idx++;
        }
        
    } while(keep_running);

    close(_timer_fd);
    
    std::cout << "[CPP_recv]: number of messages received: " << value_idx <<std::endl;
    std::cout << "[CPP_recv]: Done." << std::endl;
    
    return overruns/value_idx;
}



bool data_message(zmq::socket_t * sub, std::vector<uint16_t> * data) {
  static zmq::message_t msg;
  static const std::string delimiter = ",";
  std::string msg_str;
  size_t last = 0; 
  size_t next = 0; 
  int pos = 0;

  sub->recv(msg);
  msg_str = msg.to_string();
  if (stop_trigger.compare(msg_str)  == 0) {
    return 1;
  }

  while ((next = msg_str.find(delimiter, last)) != std::string::npos) {
    (*data)[pos++] = std::stoi(msg_str.substr(last, next-last));
    last = next + 1; 
  }
  (*data)[pos] = std::stoi(msg_str.substr(last));

  return 0;
}








