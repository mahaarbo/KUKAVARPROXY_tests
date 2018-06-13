#include "kuka_hardware_interface.h"
#include <boost/timer/timer.hpp>
#include <chrono>
#include <thread>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include "posix_serial.c"
#include <boost/asio.hpp>
#define timer timer_class
#include <boost/progress.hpp>
#undef timer

#include <boost/thread.hpp>
#include <boost/thread/barrier.hpp>

boost::mutex mutex;

void *move30deg(kuka_rsi_hw_interface::KukaHardwareInterface *kuka_rsi_hw_interface, int dir, boost::barrier &bar){
  float acc_max = dir*(3.141592653589/180.0)*10.0; // deg s^-2
  float curr_speed = dir*(3.141592653589/180.0)*2.5; // deg s^-1
  int max_iter = 10;
  float perc = 0.7;
  kuka_rsi_hw_interface->read();
  kuka_rsi_hw_interface->write(0.0);
  bar.wait();
  boost::timer::auto_cpu_timer test_timer(9,"test_timer:%w\n");
  for (int i = 0; i < max_iter; i++){
    test_timer.start();
    kuka_rsi_hw_interface->read();
    /*if (i < perc*max_iter){
      //accelerate
      curr_speed += acc_max*(12.0/1000.0);
      kuka_rsi_hw_interface->write(curr_speed);
    }
    else*/ if (i < (1.0-perc)*max_iter){
      //constant
      kuka_rsi_hw_interface->write(curr_speed);
      
    }
    else{
      // decelerate
      curr_speed -= acc_max*(12.0/1000.0);
      kuka_rsi_hw_interface->write(curr_speed);
    }
    mutex.lock();
    test_timer.report();
    std::cout.flush();
    mutex.unlock();
  }
}

void wait_standing_still(kuka_rsi_hw_interface::KukaHardwareInterface *kuka_rsi_hw_interface, int imu_fd){
  boost::timer::cpu_timer move_timer;
  move_timer.start();
  const boost::timer::nanosecond_type post_clear_delay(1000000000LL*3.0); // seconds
  // Wait until IMU stops reporting acceleration
  //std::cerr<< "Waiting for IMU to clear acceleration" << std::endl;
  unsigned char c[1] = {'A'};
  int res = 0;
  while (c[0] == 'A') {
    res = read(imu_fd,c,1);
    kuka_rsi_hw_interface->read();
    kuka_rsi_hw_interface->write(0.0);
  }
  //std::cerr<< "Waiting for serial buffer to clear" << std::endl;
  // Wait until serial buffer is cleared
  while (res != -1){
    res = read(imu_fd, c, 1);
    kuka_rsi_hw_interface->read();
    kuka_rsi_hw_interface->write(0.0);
  }
  //std::cerr<< "Waiting for timer" << std::endl;
  //Wait an extra little delay just in case
  while (move_timer.elapsed().wall < post_clear_delay) {
    kuka_rsi_hw_interface->read();
    kuka_rsi_hw_interface->write(0.0);
    
  }
}

void time_movement(int imu_fd, boost::barrier &bar, int test_idx){
  std::string format = boost::lexical_cast<std::string>(test_idx) + ",%w\n";
  unsigned char d[1] = {'\0'};
  boost::timer::auto_cpu_timer imu_timer(9,format);
  //std::cerr<<"Waiting for barrier" << std::endl;
  bar.wait();
  //usleep(50000);
  imu_timer.start();
  while(read(imu_fd,d,1) == -1){}
  imu_timer.stop();
  mutex.lock();
  if (d[0] != 'A'){
    std::cout << test_idx << ", TIMEDOUT" << std::endl;
  } else {
    imu_timer.report();
    std::cout.flush();
  }
  mutex.unlock();
}

int main(int argc, char** argv)
{
  mutex.unlock();
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

  // Declare variables
  // IMU io
  boost::asio::io_service io;
  boost::asio::serial_port port(io);
  unsigned char c[1]; // buffer for the IMU resp

  // Connect to serial port
  int imu_fd;
  const char *port_name = "/dev/ttyACM0";
  
  std::cerr << "Opening serial to IMU." << std::endl;
  imu_fd = open_serial(port_name);
  unsigned char d[1] = {'\0'}; // Initialize receive character from IMU
  
  // Setup kuka_rsi_hw_interface
  kuka_rsi_hw_interface::KukaHardwareInterface kuka_rsi_hw_interface;
  kuka_rsi_hw_interface.configure();

  // Setup threading things
  boost::thread move_thread;
  boost::thread timing_thread;
  boost::barrier bar(2);
  
  // Setup easy reading
  boost::progress_display show_progress(num_tests, std::cerr);

  
  // Start loop
  std::cout << "Test, Time" << std::endl;
  kuka_rsi_hw_interface.start();
  int dir = -1;
  for (int test_idx = 0; test_idx < num_tests; test_idx++){
    // Timer output format
    std::string format = boost::lexical_cast<std::string>(test_idx) + ",%w\n";
    // Ensure we're standing still
    wait_standing_still(&kuka_rsi_hw_interface, imu_fd);
    // Start timer, perform test
    timing_thread = boost::thread(boost::bind(time_movement, imu_fd, boost::ref(bar), test_idx));
    move_thread = boost::thread(boost::bind(move30deg, &kuka_rsi_hw_interface, dir, boost::ref(bar)));
    timing_thread.join();
    move_thread.join();
    
    dir *= -1;
  }

}
