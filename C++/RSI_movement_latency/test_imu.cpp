#include <string>
#include <boost/timer/timer.hpp>
#include <chrono>
#include <thread>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include "posix_serial.c"
#include <boost/asio.hpp>


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
  

  // Start Loop
  std::cout << "Test, Time" << std::endl;
  int test_i = 0;
  bool just_moved = false;
  boost::timer::cpu_timer move_timer;
  const boost::timer::nanosecond_type wait_2s(5*1000000000LL);
  move_timer.start();
  while (test_i < num_tests)
    {
      // Case 1: Clear serial buffer if just moved
      if (just_moved)
	{
	  int res = read(fd,c,1);
	  if (res <= 0)
	    {
	      just_moved = false;
	    }
	}
      
      // Case 2: We are waiting for the timer
      if ( (move_timer.elapsed().wall) < wait_2s )
	{
	}
      // Make sure we're standing still for the next test
      // We'll do this mainly by timing.
      // First we 
      // Case 3:
      else
	{
	  // Prepare timer
	  std::string format = boost::lexical_cast<std::string>(test_i) + ",%w\n";
	  unsigned char d[1] = {'\0'};
	  boost::timer::auto_cpu_timer t(9,format);
	  int res = read(fd,d,1);
	  while ( d[0] != 'A' || res == -1 )
	    {
	      res = read(fd,d,1);
	    }
	  t.stop();
	  if (d[0] != 'A')
	    {
	      std::cout << "0, TIMED OUT, "<< d[0] << std::endl;
	      std::cerr << "res:" << res << std::endl;
	    }
	  else
	    {
	      t.report();
	      std::cerr << "res:" << res << "," << d[0] << std::endl;
	      std::cout.flush();
	    }
	  just_moved = true;
	  move_timer.start();
	}
    }
  // Close serial port
  port.close();
}
