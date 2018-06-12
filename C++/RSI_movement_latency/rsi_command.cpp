#include "rsi_command.h"

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
}
