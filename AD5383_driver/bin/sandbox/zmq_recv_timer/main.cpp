#include <iostream>
#include <string.h>

#include <unistd.h> // Clock management
#include <sys/timerfd.h> // Clock management
#include <zmq_addon.hpp> // socket communication with Python

#include "ad5383.h" // ad5383 header

//int continuous_trajectory(std::vector<uint8_t> channels, long period_ns);
bool data_message(zmq::socket_t * sub, long long * timing, std::vector<uint16_t> * data);
bool data_configure(zmq::socket_t * sub, std::vector<uint8_t> * dacChannel, long* period_ns);
void socket_only();
int socket_and_timer();
int socket_and_timer_and_control();
int socket_and_timer_and_control_and_config();

/*
 * https://brettviren.github.io/cppzmq-tour/index.html
 */
int main() {
  //int overruns = socket_and_timer();
  int overruns = socket_and_timer_and_control_and_config();
  std::cout << "[CPP_recv]: avg overruns : " << std::dec << overruns << std::endl;
  std::cout << "[CPP_recv]: Done." << std::endl;

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
        std::vector<uint16_t> data(6);
        long long timing=0;

        if (data_message(&subscriber, &timing, &data) == 1)
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
    
    std::vector<uint16_t> data(6);
    long long timing=0;
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
        

        if (data_message(&subscriber, &timing, &data) == 1)
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
    std::vector<uint16_t> data(6);
    long long timing = 0;
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
      
      if (data_message(&subscriber, &timing, &data) == 1) {
        std::cout << "[CPP_recv] STOP " << std::endl;
        keep_running = false;
      }
      else {
        if (1) {
          std::cout << "[CPP_recv] Received: " << std::flush;
          for (auto& it : data) { std::cout << it << "," << std::flush;}
          std::cout << std::endl;
        }
        
        for(int c=0; c<dacChannel.size(); c++) {
          ad.spi_xfer_public(AD5383_REG_A,AD5383_WRITE,dacChannel[c],AD5383_REG_DATA,data[c]);
        }

        if (!first) 
          value_idx++;
      }
    } while(keep_running);

    close(_timer_fd);
    
    std::cout << "[CPP_recv]: number of messages received: " << value_idx << std::endl;
    
    return overruns/value_idx;
}




int socket_and_timer_and_control_and_config() {
  AD5383 ad;

  zmq::context_t ctx(1);
  zmq::socket_t subscriber(ctx, zmq::socket_type::sub);

  std::vector<uint16_t> data(6, 2048);
  long long timing = 0;
  std::vector<uint8_t>	dacChannel(6, 31);
  long period_ns = 0;

  unsigned long long missed = 0;
  int overruns = 0;
  int nb_read_counter = 0;
  int ignore_overruns = 2;

  bool keep_running;
  int _timer_fd;

  if (!ad.spi_open()) return 1;
  ad.configure();

  subscriber.connect("tcp://localhost:5556");
  subscriber.set(zmq::sockopt::subscribe, ""); //opens ALL envelopes

  data_configure(&subscriber, &dacChannel, &period_ns);

  struct timespec ts = {
    .tv_sec = 0,
    .tv_nsec = period_ns
  };

  struct itimerspec its = {
    .it_interval = ts,
    .it_value = ts
  };
  
  // creates a new timer object, and returns a file descriptor that refers to that timer
  _timer_fd = timerfd_create(CLOCK_REALTIME, 0);
  if(_timer_fd == -1) {
      perror("execute_trajectory/timerfd_create");
      _timer_fd = 0;
      return overruns;
  }
  // set a timer that delivers timer expiration notifications via a file descriptor
  if(timerfd_settime(_timer_fd, 0, &its, NULL) == -1) {
      perror("execute_trajectory/timerfd_settime");
      close(_timer_fd);
      return overruns;
  }

  keep_running = true;

  std::cout << "[CPP_recv]: Listening..." << std::endl;
  int cpt=95;
  do {
    if (read(_timer_fd, &missed, sizeof (missed)) == -1) {
      perror("execute_trajectory/read");
      close(_timer_fd);
      return overruns;
    }
/*
    cpt--;
    overruns += missed - 1;
    if (!cpt)
      keep_running = false;

    data_message(&subscriber, &data);
    ad.spi_xfer_public(AD5383_REG_A,AD5383_WRITE,dacChannel[0],AD5383_REG_DATA,data[1]);
    ad.spi_xfer_public(AD5383_REG_A,AD5383_WRITE,dacChannel[1],AD5383_REG_DATA,data[2]);
    ad.spi_xfer_public(AD5383_REG_A,AD5383_WRITE,dacChannel[2],AD5383_REG_DATA,data[3]);
    ad.spi_xfer_public(AD5383_REG_A,AD5383_WRITE,dacChannel[3],AD5383_REG_DATA,data[4]);
    ad.spi_xfer_public(AD5383_REG_A,AD5383_WRITE,dacChannel[4],AD5383_REG_DATA,data[5]);
    ad.spi_xfer_public(AD5383_REG_A,AD5383_WRITE,dacChannel[5],AD5383_REG_DATA,data[6]);
*/
    if (data_message(&subscriber, &timing, &data) == 1) {
      keep_running = false;
    }
    else {
      //for(int c=0; c<dacChannel.size(); c++) {ad.spi_xfer_public(AD5383_REG_A,AD5383_WRITE,dacChannel[c],AD5383_REG_DATA,data[c+1]);}
      ad.spi_xfer_public(AD5383_REG_A,AD5383_WRITE,dacChannel[0],AD5383_REG_DATA,data[0]);
      ad.spi_xfer_public(AD5383_REG_A,AD5383_WRITE,dacChannel[1],AD5383_REG_DATA,data[1]);
      ad.spi_xfer_public(AD5383_REG_A,AD5383_WRITE,dacChannel[2],AD5383_REG_DATA,data[2]);
      ad.spi_xfer_public(AD5383_REG_A,AD5383_WRITE,dacChannel[3],AD5383_REG_DATA,data[3]);
      ad.spi_xfer_public(AD5383_REG_A,AD5383_WRITE,dacChannel[4],AD5383_REG_DATA,data[4]);
      ad.spi_xfer_public(AD5383_REG_A,AD5383_WRITE,dacChannel[5],AD5383_REG_DATA,data[5]);
      //std::cout << data[0] << ", " << std::flush;
      //std::cout << data[1] << ", " << std::flush;
      //std::cout << data[2] << ", " << std::flush;
      //std::cout << data[3] << ", " << std::flush;
      //std::cout << data[4] << ", " << std::flush;
      //std::cout << data[5] << std::endl;
      
      
      if (ignore_overruns)
        ignore_overruns--;
      else {
        overruns += missed - 1;
        nb_read_counter++;
      }
    }
    
  } while(keep_running);

  close(_timer_fd);
  
  std::cout << "[CPP_recv] SIG_END_PROGRAM received: ending... " << std::endl;
  std::cout << "[CPP_recv]: number of messages received: " << nb_read_counter <<std::endl;
  std::cout << "[CPP_recv]: number of overruns: " << overruns << std::endl;

  return overruns/nb_read_counter;
}


void message_decode(std::string msg, long long * timing, std::vector<uint16_t> * data) {
  static const std::string delimiter = ",";
  size_t start = 0; 
  size_t stopped = 0; 
  int pos = 0;
  bool set_timing = true;

  try {
    while ((stopped = msg.find(delimiter, start)) != std::string::npos) {
      if (set_timing) {
        *timing = std::stoll(msg.substr(start, stopped-start));
        set_timing = false;
      }
      else
        (*data).at(pos++) = std::stoi(msg.substr(start, stopped-start));
      start = stopped + 1; 
    }
    (*data).at(pos) = std::stoi(msg.substr(start));
  }
  catch (const std::out_of_range& e) {
      std::cout << "[CPP] Out of Range error: " << std::flush;
      std::cout << "set_timing=<" << set_timing << ">\t" << std::flush;
      std::cout << "pos=<" << pos << ">\t" << std::flush;
      std::cout << "start=<" << start << ">\t" << std::flush;
      std::cout << "stopped=<" << stopped << ">\t" << std::flush;
      std::cout << std::endl;
  }

}

bool data_message(zmq::socket_t * sub, long long * timing, std::vector<uint16_t> * data) {
  static std::string stop_trigger("SIG_END_PROGRAM");
  zmq::message_t msg;
  std::string msg_str;

  sub->recv(msg); // default: zmq_recv() function shall block until the request can be satisfied. 
  msg_str = msg.to_string();

  if (stop_trigger.compare(msg_str)  == 0) {
    return 1;
  }
  
  message_decode(msg_str, timing, data);

  return 0;
}


// 
bool data_configure(zmq::socket_t * sub, std::vector<uint8_t> * dacChannel, long* period_ns) {
  static zmq::message_t msg;
  static const std::string delim_type = ":";
  static const std::string delim_val = ",";
  std::string msg_str;
  std::string type_str;
  
  int sec2ns = 1000*1000*1000; //1e+9

  bool received_channel = false;
  bool received_frequency = false;
  
  std::string type_channel("dacChannels");
  std::string type_frequency("frequency");

  int pos = 0;

  while (!received_channel || !received_frequency) {
    size_t start = 0; 
    size_t stopped = 0; 
    
    sub->recv(msg); // default: zmq_recv() function shall block until the request can be satisfied. 
    msg_str = msg.to_string();
    std::cout << "[CPP_recv]: " << msg_str << std::endl;

    // extract data type
    stopped = msg_str.find(delim_type, start);
    type_str = msg_str.substr(start, stopped-start);
    
    // prepare the pointer to the value part (jumps the delimiter)
    start = stopped + 1; 

    // different tasks by data type
    if (type_channel.compare(type_str) == 0) {
      while ((stopped = msg_str.find(delim_val, start)) != std::string::npos) {
        (*dacChannel)[pos++] = std::stoi(msg_str.substr(start, stopped-start));
        start = stopped + 1; 
      }
      (*dacChannel)[pos] = std::stoi(msg_str.substr(start));
      received_channel = true;
      std::cout << "[CPP_recv]: received channels!" << std::endl;
    }
    else if (type_frequency.compare(type_str) == 0) {
      *period_ns = sec2ns/(double)std::stoi(msg_str.substr(start));
      received_frequency = true;
      std::cout << "[CPP_recv]: received frequency!" << std::endl;
    }
    else {
      //std::cout << "Whew." << std::endl;
    }
  }

  return 0;
}








