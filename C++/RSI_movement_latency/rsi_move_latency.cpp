#include <string>
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
  std::string kuka_ip = "10.0.0.1";
  std::string kuka_port = "7000";

  // Declare variables
  // IMU io
  boost::asio::io_service io;
  boost::asio::serial_port port(io);
  unsigned char c[1]; // buffer for the IMU resp

  // Connect to serial port
  int fd;
  const char *port_name = "/dev/ttyACM0";
  
  std::cerr << "Opening serial to IMU." << std::endl;
  fd = open_serial(port_name);
  
  // Setup kuka_hardware_interface
  kuka_rsi_hw_interface::KukaHardwareInterface kuka_rsi_hw_interface;
  kuka_rsi_hw_interface.configure();
  
  // Let's start up and ensure we're standing still
  kuka_rsi_hw_interface.start();
  // Ensure that the robot is at rest and that the interface works
  boost::timer::cpu_timer move_timer;
  move_timer.start();
  const boost::timer::nanosecond_type wait_2s(1.0*1000000000LL);
  const boost::timer::nanosecond_type wait_11ms(11*1000000LL);
  std::cerr << "Ensuring that it's stationary" << std::endl;
  while ( move_timer.elapsed().wall < wait_2s )
    {
      kuka_rsi_hw_interface.read();
      kuka_rsi_hw_interface.write(0.0);
      kuka_rsi_hw_interface.write(0.0);
    }
  std::cerr << "It's stationary." << std::endl;
  //Setup Progressbar
  boost::progress_display show_progress(num_tests, std::cerr);
  kuka_rsi_hw_interface.read(); // just in case
  kuka_rsi_hw_interface.write(0.0);
  // Start Loop
  std::cout << "Test, Time" << std::endl;
  int test_i = 0;
  int direction = 1;
  int dir_step_i = 0;
  double delta_deg = 2.0;//2.0;
  double delta = 3.141592653589*delta_deg/180.0;
  bool just_moved = false;
  move_timer.start();
  while (test_i < num_tests)
    {
      // Case 1: We are waiting for the timer
      if ( (move_timer.elapsed().wall) < wait_2s )
	{
	  if (!kuka_rsi_hw_interface.read())
	    {
	      std::cerr << "ERROR READING RSI WHILE IN WAIT" << std::endl;
	      return -1;
	    }
	  kuka_rsi_hw_interface.write(0.0);
	}
      // Make sure we're standing still for the next test
      // We'll do this mainly by timing.
      // First we 
      // Case 2: Clear serial buffer if just moved
      else if (just_moved)
	{
	  if (!kuka_rsi_hw_interface.read())
	    {
	      std::cerr << "ERROR READING RSI WHILE IN JUST MOVED" << std::endl;
	      return -1;
	    }
	  kuka_rsi_hw_interface.write(0.0);
	  int res = read(fd,c,1);
	  if (res <= 0 && c[0] != 'A')
	  {
	    just_moved = false;
	    move_timer.start();
	  }
	}
      else
	{
	  // Only enter this if the serial buffer is clear
	  // Prepare timer
	  std::string format = boost::lexical_cast<std::string>(test_i) + ",%w\n";
	  unsigned char d[1] = {'\0'};
	  
	  // Wait for RSI info
	  if (!kuka_rsi_hw_interface.read())
	    {
	      std::cerr<<"ERROR READING RSI" << std::endl;
	      return -1;
	    }
	  int res = read(fd,d,1);
	  while ( d[0] == 'A' || res != -1) {
	    res = read(fd,d,1);
	    kuka_rsi_hw_interface.write(0.0);
	    kuka_rsi_hw_interface.read();
	  }
	  boost::timer::auto_cpu_timer t(9,format);
	  kuka_rsi_hw_interface.write(delta*direction);
	  res = read(fd,d,1);
	  while ( ((d[0] != 'A') || (res == -1)))// && (t.elapsed().wall < wait_11ms) )
	  {
	    res = read(fd,d,1);
	  }
	  t.stop();
	  if (d[0] != 'A')
	    {
	      std::cout << "0, TIMED OUT "<< d[0] << std::endl;
	    }
	  else
	    {
	      t.report();
	      std::cout.flush();
	    }
	  just_moved = true;
	  move_timer.start();
	  // Move 30 degrees before you change direction
	  if (dir_step_i < 0)
	    {
	      dir_step_i++;
	    }
	  else
	    {
	      dir_step_i = 0;
	      direction *= -1;
	    }
	  test_i++;
	  ++show_progress;
	  std::cerr.flush();
	}
    }
  // Close serial port
  port.close();
}
