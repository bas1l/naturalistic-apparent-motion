#include <iostream>
#include <string.h>

#include <unistd.h> // Clock management
#include <sys/timerfd.h> // Clock management
#include <zmq_addon.hpp> // socket communication with Python

#include "ad5383.h" // ad5383 header

//int continuous_trajectory(std::vector<uint8_t> channels, long period_ns);
bool message_process(zmq::socket_t * sub, long long * timing, std::vector<uint16_t> * data);
void message_getValues(std::string msg, long long * timing, std::vector<uint16_t> * data);

bool data_configure(zmq::socket_t * sub, std::vector<uint8_t> * dacChannel, long* period_ns);

/*
 * https://brettviren.github.io/cppzmq-tour/index.html
 */
int main() {

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

  std::cout << "AD: Listening..." << std::endl;
  do {
    if (read(_timer_fd, &missed, sizeof (missed)) == -1) {
      perror("execute_trajectory/read");
      close(_timer_fd);
      return overruns;
    }

    if (message_process(&subscriber, &timing, &data) == 1) {
      keep_running = false;
    }
    else {
      ad.spi_xfer_public(AD5383_REG_A,AD5383_WRITE,dacChannel[0],AD5383_REG_DATA,data[0]);
      ad.spi_xfer_public(AD5383_REG_A,AD5383_WRITE,dacChannel[1],AD5383_REG_DATA,data[1]);
      ad.spi_xfer_public(AD5383_REG_A,AD5383_WRITE,dacChannel[2],AD5383_REG_DATA,data[2]);
      ad.spi_xfer_public(AD5383_REG_A,AD5383_WRITE,dacChannel[3],AD5383_REG_DATA,data[3]);
      ad.spi_xfer_public(AD5383_REG_A,AD5383_WRITE,dacChannel[4],AD5383_REG_DATA,data[4]);
      ad.spi_xfer_public(AD5383_REG_A,AD5383_WRITE,dacChannel[5],AD5383_REG_DATA,data[5]);
      
      if (ignore_overruns)
        ignore_overruns--;
      else {
        overruns += missed - 1;
        nb_read_counter++;
      }
    }
    
  } while(keep_running);

  close(_timer_fd);
  
  std::cout << "AD: Number of <messages received>:" << nb_read_counter <<std::flush;
  std::cout << "\t<overruns>:" << overruns << std::flush;
  std::cout << "\t<AVG overruns>:" << std::dec << 100*overruns/(float)nb_read_counter << "%" << std::endl;
  std::cout << "AD: SIG_END_PROGRAM received: end. " << std::endl;
  
  return 0;
}


bool message_process(zmq::socket_t * sub, long long * timing, std::vector<uint16_t> * data) {
  static const std::string stop_trigger("SIG_END_PROGRAM");
  static zmq::message_t msg;
  std::string msg_str;

  sub->recv(msg); // default: zmq_recv() function shall block until the request can be satisfied. 
  msg_str = msg.to_string();

  if (stop_trigger.compare(msg_str)  == 0) {
    return 1;
  }
  
  message_getValues(msg_str, timing, data);

  return 0;
}


void message_getValues(std::string msg, long long * timing, std::vector<uint16_t> * data) {
  static const std::string delimiter(",");
  size_t start = 0; 
  size_t stopped = 0; 
  int pos = 0;
  bool set_timing = true;

  while ((stopped = msg.find(delimiter, start)) != std::string::npos) {
    if (set_timing) {
      *timing = std::stoll(msg.substr(start, stopped-start));
      set_timing = false;
    }
    else
      (*data).at(pos++) = std::stoll(msg.substr(start, stopped-start));
    start = stopped + 1; 
  }
  (*data).at(pos) = std::stoll(msg.substr(start));
}


bool data_configure(zmq::socket_t * sub, std::vector<uint8_t> * dacChannel, long* period_ns) {
  static const std::string delim_type = ":";
  static const std::string delim_val = ",";
  static zmq::message_t msg;
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
    //std::cout << "[CPP_recv]: " << msg_str << std::endl;

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
      //std::cout << "[CPP_recv]: received channels!" << std::endl;
    }
    else if (type_frequency.compare(type_str) == 0) {
      *period_ns = sec2ns/(double)std::stoi(msg_str.substr(start));
      received_frequency = true;
      //std::cout << "[CPP_recv]: received frequency!" << std::endl;
    }
  }

  return 0;
}








