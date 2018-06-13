#include <string>
#include "kuka_hardware_interface.h"
#include <boost/timer/timer.hpp>
#include <chrono>
#include <thread>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include "posix_serial.c"
#define timer timer_class
#include <boost/progress.hpp>
#undef timer

void move_trapezoidal(kuka_rsi_hw_interface::KukaHardwareInterface *kuka_rsi_hw_interface, int dir, int imu_fd, int test_idx){
  // Motion specification
  float acc_max = dir*(3.141592653589/180.0)*120.0; // deg s^-2
  float curr_speed = dir*(3.141592653589/180.0)*0.0; // deg s^-1
  int max_iter = 30;
  float perc = 0.05;

  // IMU variables
  unsigned char d[1] = {'\0'};
  bool first_command = true;
  bool got_response = false;
  std::string format = boost::lexical_cast<std::string>(test_idx) + ",%w\n";
  const boost::timer::nanosecond_type max_wait(1000000LL*10.0); //milliseconds wait before we have to recheck rsi
  int res;
  
  // Start timer and RSI
  kuka_rsi_hw_interface->read();
  kuka_rsi_hw_interface->write(0.0);
  boost::timer::auto_cpu_timer move_timer(9,format);
  boost::timer::cpu_timer rsi_period_timer;
  kuka_rsi_hw_interface->read();
  for (int i = 0; i < max_iter; i++){
    if (i < perc*max_iter){
      // Accelerate
      curr_speed += acc_max*(12.0/1000.0);
    } else if (i < (1.0-perc)*max_iter){
      // Constant speed
    } else {
      // Decelerate
      curr_speed -= acc_max*(12.0/1000.0);
    }
    if (first_command) {
      move_timer.start();
      first_command = false;
    }
    if (!got_response) {
      rsi_period_timer.start();
    }
    kuka_rsi_hw_interface->write(curr_speed);
    res = read(imu_fd,d,1);
    while( (rsi_period_timer.elapsed().wall < max_wait) && (res ==-1) && (!got_response)){
      res = read(imu_fd,d,1);
    }
    if (res != -1 && !got_response) {
      move_timer.report();
      got_response = true;
    }
    kuka_rsi_hw_interface->read();
  }
  kuka_rsi_hw_interface->write(0.0);
}


void move_geometric_decay(kuka_rsi_hw_interface::KukaHardwareInterface *kuka_rsi_hw_interface, int dir, int imu_fd, int test_idx){
  // Motion specification
  float curr_speed = dir*(3.141592653589/180.0)*10.0; // deg s^-1
  int max_iter = 10;

  // IMU variables
  unsigned char d[1] = {'\0'};
  bool first_command = true;
  bool got_response = false;
  std::string format = boost::lexical_cast<std::string>(test_idx) + ",%w\n";
  const boost::timer::nanosecond_type max_wait(1000000LL*10.0); //milliseconds wait before we have to recheck rsi
  int res;
  
  // Start timer and RSI
  kuka_rsi_hw_interface->read();
  kuka_rsi_hw_interface->write(0.0);
  boost::timer::auto_cpu_timer move_timer(9,format);
  boost::timer::cpu_timer rsi_period_timer;
  kuka_rsi_hw_interface->read();
  for (int i = 0; i < max_iter; i++){
    if (i < 0.5*max_iter){
      // Constant speed
    } else {
      // Decelerate
      curr_speed *= 0.7;
    }
    if (!got_response) {
      rsi_period_timer.start();
    }
    if (first_command) {
      move_timer.start();
      first_command = false;
    }
    kuka_rsi_hw_interface->write(curr_speed);
    res = read(imu_fd,d,1);
    while( (rsi_period_timer.elapsed().wall < max_wait) && (res ==-1) && (!got_response)){
      res = read(imu_fd,d,1);
    }
    if (res != -1 && !got_response) {
      move_timer.report();
      got_response = true;
    }
    kuka_rsi_hw_interface->read();
  }
  kuka_rsi_hw_interface->write(0.0);
}

void wait_standing_still(kuka_rsi_hw_interface::KukaHardwareInterface *kuka_rsi_hw_interface, int imu_fd){
  boost::timer::cpu_timer move_timer;
  move_timer.start();
  const boost::timer::nanosecond_type post_clear_delay(1000000000LL*5.0); // seconds
  // Wait until IMU stops reporting acceleration
  //std::cerr<< "Waiting for IMU to clear acceleration" << std::endl;
  unsigned char c[1] = {'A'};
  int res = 0;
  while (c[0] == 'A') {
    res = read(imu_fd,c,1);
    kuka_rsi_hw_interface->read();
    kuka_rsi_hw_interface->write(0.0);
    move_timer.start();
  }
  //std::cerr<< "Waiting for serial buffer to clear" << std::endl;
  // Wait until serial buffer is cleared
  while (res != -1){
    res = read(imu_fd, c, 1);
    kuka_rsi_hw_interface->read();
    kuka_rsi_hw_interface->write(0.0);
    move_timer.start();
  }
  //std::cerr<< "Waiting for timer" << std::endl;
  //Wait an extra little delay just in case
  while (move_timer.elapsed().wall < post_clear_delay) {
    kuka_rsi_hw_interface->read();
    kuka_rsi_hw_interface->write(0.0);
  }
}
int main(int argc, char** argv)
{
  int baud = 115200;
  int timeout = 5000;
  int num_tests = 30;
  try {
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help,h", "produce help message")
      ("baudrate", po::value<int>(), "set baudrate, default=115200")
      ("num_tests", po::value<int>(), "set number of tests, default=300");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
      std::cout << desc << std::endl;
      return 0;
    }
    if (vm.count("baud")) {
      baud = vm["baudrate"].as<int>();
    }
    if (vm.count("num_tests")) {
      num_tests = vm["num_tests"].as<int>();
    }
  }
  catch(std::exception& e) {
    std::cerr << "error:" << e.what() << "\n";
    return 1;
  }
  
  // Setup
  char usb[] = "/dev/ttyACM0";

  // Connect to serial port
  int imu_fd;
  const char *port_name = "/dev/ttyACM0";
  
  std::cerr << "Opening serial to IMU." << std::endl;
  imu_fd = open_serial(port_name);
  
  // Setup kuka_hardware_interface
  kuka_rsi_hw_interface::KukaHardwareInterface kuka_rsi_hw_interface;
  kuka_rsi_hw_interface.configure();

  // Setup easy reading
  boost::progress_display show_progress(num_tests, std::cerr);
  
  // Let's start up
  std::cout << "Test, Time" << std::endl;
  kuka_rsi_hw_interface.start();
  int dir = -1;
  for (int test_idx=0; test_idx < num_tests; test_idx++) {
    // Ensure we're standing still
    wait_standing_still(&kuka_rsi_hw_interface, imu_fd);
    move_geometric_decay(&kuka_rsi_hw_interface, dir, imu_fd, test_idx);
    dir *= -1;
  }
}
