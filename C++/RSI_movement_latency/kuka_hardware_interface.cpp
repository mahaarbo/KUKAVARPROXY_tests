#include "kuka_hardware_interface.h"
#include <stdexcept>

namespace kuka_rsi_hw_interface
{
  RSICommand::RSICommand()
  {
    // Intentionally empty
  }
  RSICommand::RSICommand(std::vector<double> joint_position_correction, unsigned long long ipoc, std::vector<double> tcp_position_corrections, bool external_axes = false)
  {
    TiXmlDocument doc;
    TiXmlElement* root = new TiXmlElement("Sen");
    root->SetAttribute("Type", "ImFree");
    TiXmlElement* el = new TiXmlElement("AK");
    // Add string attribute
    el->SetAttribute("A1", std::to_string(joint_position_correction[0]));
    el->SetAttribute("A2", std::to_string(joint_position_correction[1]));
    el->SetAttribute("A3", std::to_string(joint_position_correction[2]));
    el->SetAttribute("A4", std::to_string(joint_position_correction[3]));
    el->SetAttribute("A5", std::to_string(joint_position_correction[4]));
    el->SetAttribute("A6", std::to_string(joint_position_correction[5]));
    root->LinkEndChild(el);
    // External axes
    if (external_axes) {
      el = new TiXmlElement("EK");
      // Add string attribute
      el->SetAttribute("E1", std::to_string(joint_position_correction[6]));
      el->SetAttribute("E2", std::to_string(joint_position_correction[7]));
      el->SetAttribute("E3", std::to_string(joint_position_correction[8]));
      el->SetAttribute("E4", std::to_string(joint_position_correction[9]));
      el->SetAttribute("E5", std::to_string(joint_position_correction[10]));
      el->SetAttribute("E6", std::to_string(joint_position_correction[11]));
      root->LinkEndChild(el);
      
      el = new TiXmlElement("RK");
      el->SetAttribute("X", std::to_string(tcp_position_corrections[0]));
      el->SetAttribute("Y", std::to_string(tcp_position_corrections[1]));
      el->SetAttribute("Z", std::to_string(tcp_position_corrections[2]));
      el->SetAttribute("A", "0");
      el->SetAttribute("B", "0");
      el->SetAttribute("C", "0");
      root->LinkEndChild(el);
    }
    
    el = new TiXmlElement("IPOC");
    el->LinkEndChild(new TiXmlText(std::to_string(ipoc)));
    root->LinkEndChild(el);
    doc.LinkEndChild(root);
    TiXmlPrinter printer;
    printer.SetStreamPrinting();
    doc.Accept(&printer);
    
    xml_doc = printer.Str();
  }
  RSIState::RSIState(std::string xml_doc) :
    xml_doc_(xml_doc),
    positions(12, 0.0),
    initial_positions(12, 0.0),
    cart_position(12, 0.0),
    initial_cart_position(12, 0.0),
    force(3, 0.0),
    torque(3, 0.0)
  {
    // Parse message from robot
    TiXmlDocument bufferdoc;
    bufferdoc.Parse(xml_doc_.c_str());
    // Get the Rob node:
    TiXmlElement* rob = bufferdoc.FirstChildElement("Rob");
    // Extract axis specific actual position
    TiXmlElement* AIPos_el = rob->FirstChildElement("AIPos");
    AIPos_el->Attribute("A1", &positions[0]);
    AIPos_el->Attribute("A2", &positions[1]);
    AIPos_el->Attribute("A3", &positions[2]);
    AIPos_el->Attribute("A4", &positions[3]);
    AIPos_el->Attribute("A5", &positions[4]);
    AIPos_el->Attribute("A6", &positions[5]);
    // Extract axis specific setpoint position
    TiXmlElement* ASPos_el = rob->FirstChildElement("ASPos");
    ASPos_el->Attribute("A1", &initial_positions[0]);
    ASPos_el->Attribute("A2", &initial_positions[1]);
    ASPos_el->Attribute("A3", &initial_positions[2]);
    ASPos_el->Attribute("A4", &initial_positions[3]);
    ASPos_el->Attribute("A5", &initial_positions[4]);
    ASPos_el->Attribute("A6", &initial_positions[5]);
    // Extract cartesian actual position
    TiXmlElement* RIst_el = rob->FirstChildElement("RIst");
    RIst_el->Attribute("X", &cart_position[0]);
    RIst_el->Attribute("Y", &cart_position[1]);
    RIst_el->Attribute("Z", &cart_position[2]);
    RIst_el->Attribute("A", &cart_position[3]);
    RIst_el->Attribute("B", &cart_position[4]);
    RIst_el->Attribute("C", &cart_position[5]);
    // Extract cartesian actual position
    TiXmlElement* RSol_el = rob->FirstChildElement("RSol");
    RSol_el->Attribute("X", &initial_cart_position[0]);
    RSol_el->Attribute("Y", &initial_cart_position[1]);
    RSol_el->Attribute("Z", &initial_cart_position[2]);
    RSol_el->Attribute("A", &initial_cart_position[3]);
    RSol_el->Attribute("B", &initial_cart_position[4]);
    RSol_el->Attribute("C", &initial_cart_position[5]);
    // Get the IPOC timestamp
    TiXmlElement* ipoc_el = rob->FirstChildElement("IPOC");
    ipoc = std::stoull(ipoc_el->FirstChild()->Value());
    // External axes actual position
    TiXmlElement* EIPos_el = rob->FirstChildElement("EIPos");
    if (EIPos_el) {
      EIPos_el->Attribute("E1", &positions[6]);
      EIPos_el->Attribute("E2", &positions[7]);
      EIPos_el->Attribute("E3", &positions[8]);
      EIPos_el->Attribute("E4", &positions[9]);
      EIPos_el->Attribute("E5", &positions[10]);
      EIPos_el->Attribute("E6", &positions[11]);
    }
    // External axes setpoint position
    TiXmlElement* ESPos_el = rob->FirstChildElement("ESPos");
    if (ESPos_el) {
      ESPos_el->Attribute("E1", &initial_positions[6]);
      ESPos_el->Attribute("E2", &initial_positions[7]);
      ESPos_el->Attribute("E3", &initial_positions[8]);
      ESPos_el->Attribute("E4", &initial_positions[9]);
      ESPos_el->Attribute("E5", &initial_positions[10]);
      ESPos_el->Attribute("E6", &initial_positions[11]);
    }
    // Get FT data if available
    TiXmlElement* FTC_el = rob->FirstChildElement("FTC");
    if (FTC_el)
      {
	FTC_el->Attribute("Fx", &force[0]);
	FTC_el->Attribute("Fy", &force[1]);
	FTC_el->Attribute("Fz", &force[2]);	
	FTC_el->Attribute("Mx", &torque[0]);
	FTC_el->Attribute("My", &torque[1]);
	FTC_el->Attribute("Mz", &torque[2]);
      }
  }
  KukaHardwareInterface::KukaHardwareInterface()
    : joint_position_(12, 0.0)
    , joint_position_command_(12, 0.0)
    , joint_names_(12)
    , rsi_initial_joint_positions_(12, 0.0)
    , rsi_joint_position_corrections_(12, 0.0)
    , rsi_tcp_position_corrections_(3, 0.0)
    , ipoc_(0)
    , n_dof_(6)
  {
    in_buffer_.resize(1024);
    out_buffer_.resize(1024);
    remote_host_.resize(1024);
    remote_port_.resize(1024);

    external_axes_ = true;
    n_dof_ = 9;
    
      
  }
  KukaHardwareInterface::~KukaHardwareInterface()
  {
  }
  void KukaHardwareInterface::start()
  {
    std::cerr << "Starting KukaHardwareInterface" << std::endl;
    // Wait for connection from robot
    server_.reset(new UDPServer(local_host_, local_port_));

    int bytes = server_->recv(in_buffer_);

    // Drop empty <rob> frame with RSI <= 2.3
    if (bytes < 100)
      {
	bytes = server_->recv(in_buffer_);
      }
    rsi_state_ = RSIState(in_buffer_);
    for (std::size_t i = 0; i < 6; ++i)
      {
	joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
	joint_position_command_[i] = joint_position_[i];
	rsi_initial_joint_positions_[i] = rsi_state_.initial_positions[i];
      }
    for (std::size_t i  = 6; i < n_dof_; ++i)
      {
	joint_position_[i] = DEG2RAD * rsi_state_.positions[i] / 1000;
	joint_position_command_[i] = joint_position_[i];
	rsi_initial_joint_positions_[i] = joint_position_[i];
      }
    ipoc_ = rsi_state_.ipoc;
    out_buffer_ = RSICommand(rsi_joint_position_corrections_, ipoc_, rsi_tcp_position_corrections_, external_axes_).xml_doc;
    server_->send(out_buffer_);
    // Set receive timeout to 1 second
    server_->set_timeout(1000);
    std::cerr << "Got connection from robot" << std::endl;
  }
  void KukaHardwareInterface::configure()
  {
    std::cerr << "Configuring KukaHardwareInterface" << std::endl;
    local_host_ = "10.0.0.200";
    local_port_ = 49153;
  }
  bool KukaHardwareInterface::read()
  {
    //std::cerr << "KukaHardwareInterface:Read" << std::endl;
    in_buffer_.resize(1024);
    if (server_->recv(in_buffer_) == 0)
      {
	return false;
      }
    rsi_state_ = RSIState(in_buffer_);
    // Update joint positions
    for (std::size_t i = 0; i < 6; ++i)
      {
	joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
      }
    for (std::size_t i = 6; i < n_dof_; ++i)
      {
	joint_position_[i] = DEG2RAD * rsi_state_.positions[i] / 1000;
      }
    // Update IPOC number
    ipoc_ = rsi_state_.ipoc;
    return true;
  }
  bool KukaHardwareInterface::write(double a5delta_rad)
  {
    //std::cerr << "KukaHardwareInterface:Write" << std::endl;
    out_buffer_.resize(1024);
    // Ugly but simple way of only affecting the a5 joint
    joint_position_command_[4] = joint_position_command_[4] + a5delta_rad;
    for (std::size_t i = 0; i < 6; ++i)
      {
	rsi_joint_position_corrections_[i] = (RAD2DEG * joint_position_command_[i]) - rsi_initial_joint_positions_[i];
      }
    // With mathematically linked external axes, KRC will change robot joints to keep TCP static
  // Update TCP by same amount as linear axes to avoid this. Units in [mm]
  if (external_axes_)
    {
      // E1 & X
      rsi_joint_position_corrections_[6] = 1000 * (joint_position_command_[6] - rsi_initial_joint_positions_[6]);
      rsi_tcp_position_corrections_[0] = -rsi_joint_position_corrections_[6];
      // E2 & Y
      rsi_joint_position_corrections_[7] = 1000 * (joint_position_command_[7] - rsi_initial_joint_positions_[7]);
      rsi_tcp_position_corrections_[1] = -rsi_joint_position_corrections_[7];
      // E3 & Z
      rsi_joint_position_corrections_[8] = 1000 * (joint_position_command_[8] - rsi_initial_joint_positions_[8]);
      rsi_tcp_position_corrections_[2] = rsi_joint_position_corrections_[8];
    }
  out_buffer_ = RSICommand(rsi_joint_position_corrections_, ipoc_, rsi_tcp_position_corrections_, external_axes_).xml_doc;
  server_->send(out_buffer_);
  return true;
  }
}
