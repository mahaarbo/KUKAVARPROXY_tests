#ifndef KUKA_RSI_HARDWARE_INTERFACE_
#define KUKA_RSI_HARDWARE_INTERFACE_

// STL
#include <vector>
#include <string>
#include <memory>

// Timers
#include <chrono>

// UDP server
#include "udp_server.h"

// RSI
//#include "rsi_state.h"
//#include "rsi_command.h"
#include <tinyxml.h>


namespace kuka_rsi_hw_interface
{
  static const double RAD2DEG = 57.295779513082323;
  static const double DEG2RAD = 0.017453292519943295;
  
  class RSIState
  {
  private:
    std::string xml_doc_;
    
  public:
    RSIState() :
      positions(12, 0.0),
      initial_positions(12, 0.0),
      cart_position(12, 0.0),
      initial_cart_position(12, 0.0),
      force(3, 0.0),
      torque(3, 0.0)
    {
      xml_doc_.resize(1024);
    }
    
    RSIState(std::string xml_doc);
    
    // AIPOS
    std::vector<double> positions;
    
    // ASPos
    std::vector<double> initial_positions;
    
    // RIst
    std::vector<double> cart_position;
    
    // RSol
    std::vector<double> initial_cart_position;
    
    // IPOC
    unsigned long long ipoc;
    
    // Force
    std::vector<double> force;
    
    // Tourqe
    std::vector<double> torque;
    
  };
  
  class RSICommand
  {
  public:
    RSICommand();
    RSICommand(std::vector<double> position_corrections, unsigned long long ipoc, std::vector<double> tcp_position_corrections, bool external_axes);
    std::string xml_doc;
  };

  class KukaHardwareInterface
  {
  private:
    // Configuration
    int n_dof_;
    bool external_axes_;

    // Read and command
    std::vector<std::string> joint_names_;
    std::vector<double> joint_position_;
    std::vector<double> joint_position_command_;

    //RSI
    RSIState rsi_state_;
    RSICommand rsi_command_;
    std::vector<double> rsi_initial_joint_positions_;
    std::vector<double> rsi_joint_position_corrections_;
    std::vector<double> rsi_tcp_position_corrections_;
    unsigned long long ipoc_;

    std::unique_ptr<UDPServer> server_;
    std::string local_host_;
    int local_port_;
    std::string remote_host_;
    std::string remote_port_;
    std::string in_buffer_;
    std::string out_buffer_;

  public:
    KukaHardwareInterface();
    ~KukaHardwareInterface();

    void start();
    void configure();
    bool read();
    bool write(double a5delta_rad);
  };
} // Namespace kuka_rsi_hw_interface
#endif
