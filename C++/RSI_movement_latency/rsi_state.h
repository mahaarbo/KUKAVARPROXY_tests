#ifndef KUKA_RSI_HW_INTERFACE_RSI_STATE_
#define KUKA_RSI_HW_INTERFACE_RSI_STATE_

#include <string>
#include <tinyxml.h>

namespace kuka_rsi_hw_interface
{

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

} // namespace kuka_rsi_hw_interface

#endif
