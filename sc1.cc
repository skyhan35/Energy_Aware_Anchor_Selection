#include <iomanip>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/netanim-module.h"
#include "ns3/basic-energy-source.h"
#include "ns3/wifi-radio-energy-model.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/wifi-radio-energy-model-helper.h"
#include "ns3/energy-source-container.h"
#include "ns3/device-energy-model-container.h"
#include "ns3/energy-module.h"
#include <iostream>
#include <fstream>
#include <list>
#include <unordered_set>
#include <vector>
#include <string>
#include "TextTable.h"

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("sc1");

double sensor = 5;
double anchor = 3;

char gokhan[20] = "model_1";

double overall_energy = 50;
double simulationTime = 1500;
double iteration_num = 0;
AnimationInterface *pAnim = 0;
ObjectFactory m_energySource;
ObjectFactory m_deviceEnergyModel;
ObjectFactory energysourcecontainer;
NetDeviceContainer anchorDevice;
WifiRadioEnergyModelHelper radioEnergyHelper;
int sourt_count = 0;

ofstream ofs1("Scenario_1.plt", ofstream::out);
ofstream ofs2("Scenario_1.txt", ofstream::out);

class trilateration
{
public:
  int Dist;
  double Posx;
  double Posy;
  int Anchornum;
  int Sensornum;
  double rssival;
  double residual_energy;
};

class sensorpos
{

public:
  double sensorposx;
  double sensorposy;
  double sensornumber;
};

class calculation
{

public:
  double x1;
  double y1;
  double r1;
  double x2;
  double y2;
  double r2;
  double x3;
  double y3;
  double r3;
  double A;
  double B;
};

class sleepanchor
{

public:
  int anchornum;
  bool sleepmode;
  Ptr<WifiRadioEnergyModel> modeltype;
};

class sensorswitch
{
public:
  int sensornumber;
  bool comparethreshold;
};

class radiomodel
{
public:
  //WifiRadioEnergyModel modelnum;
  Ptr<WifiRadioEnergyModel> modeltype;
  int nodenumber;
};

std::list<trilateration> myList;
std::list<trilateration>::iterator it;
std::list<trilateration>::iterator itlist;
std::list<trilateration>::iterator itsort;
std::list<trilateration>::iterator itrem1;
std::list<trilateration>::iterator itrem2;
std::list<trilateration> myList1;
std::list<trilateration>::iterator itout;
std::list<sensorpos> posList;
std::list<sleepanchor> anchorList;
std::list<sleepanchor>::iterator an;
std::list<sleepanchor>::iterator an1;
std::list<sensorswitch> sensorList;
std::unordered_set<int> s;
std::list<radiomodel> radiolist;
std::list<radiomodel>::iterator modelit;
int tmp = 1;
SimpleDeviceEnergyModel anchormodel1; //Last Changes

std::vector<string> modellist(anchor);

void radiocheck(Ptr<WifiRadioEnergyModel> model1, Ptr<WifiRadioEnergyModel> model2)
{
  cout << "Model1: " << model1->GetCurrentState() << endl;
  cout << "Model2: " << model2->GetCurrentState() << endl;
}

/***************************************/

/* Convert double to string with specified number of places after the decimal
   and left padding. */
std::string prd(const double x, const int decDigits, const int width)
{
  stringstream ss;
  ss << fixed << right;
  ss.fill(' ');            // fill space around displayed #
  ss.width(width);         // set  width around displayed #
  ss.precision(decDigits); // set # places after decimal
  ss << x;
  return ss.str();
}

std::string center(const string s, const int w)
{
  stringstream ss, spaces;
  int padding = w - s.size(); // count excess room to pad
  for (int i = 0; i < padding / 2; ++i)
    spaces << " ";
  ss << spaces.str() << s << spaces.str(); // format with padding
  if (padding > 0 && padding % 2 != 0)     // if odd #, add 1 space
    ss << " ";
  return ss.str();
}

/***************************************/

bool my_compare(trilateration a, trilateration b) //sorting based on the shortest distance
{
  return a.rssival < b.rssival;
}

bool remove_duplicate(trilateration a, trilateration b){
  return a.rssival == b.rssival;
}



void localization(NodeContainer AnchorNodes, NodeContainer SensorNodes, EnergySourceContainer sources, EnergySourceContainer sources1, Ptr<WifiRadioEnergyModel> model1, DeviceEnergyModelContainer modelanchor1) //trilateration method
{

  Ptr<FriisPropagationLossModel> model = CreateObject<FriisPropagationLossModel>();

  trilateration t;
  sensorpos senpos;
  calculation c;

  for (uint32_t i = 0; i < anchor; i++)
  {
    for (uint32_t nodeId = 0; nodeId < sensor; nodeId++)
    {
      Ptr<MobilityModel> model1 = AnchorNodes.Get(i)->GetObject<MobilityModel>();
      Ptr<MobilityModel> model2 = SensorNodes.Get(nodeId)->GetObject<MobilityModel>();
      double distance = model1->GetDistanceFrom(model2) / 10; // calculate distance from sensor to anchor nodes divided by 10 to convert it to meter from centi
      double rssi = model->CalcRxPower(5.0, model2, model1);  // calculates RSSI and 5.0 is the transmit power
      Ptr<MobilityModel> mob = AnchorNodes.Get(i)->GetObject<MobilityModel>();
      Vector pos = mob->GetPosition();
      Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource>(sources.Get(nodeId));
      t.residual_energy = basicSourcePtr->GetRemainingEnergy();

      t.Dist = distance;
      t.Posx = pos.x;
      t.Posy = pos.y;
      t.Anchornum = i;
      t.Sensornum = nodeId;
      t.rssival = rssi;
      myList.push_back(t); 
      myList.sort(my_compare); 
      myList.unique(remove_duplicate);
      myList.sort(my_compare);

      
      //NS_LOG_UNCOND("Anchor number: "<<t.Anchornum<<" X coordinate: "<<t.Posx<<" Y coordinate: "<<t.Posy);
    }
  }

  /* for (itrem1 = myList.begin(); itrem1 != myList.end(); itrem1++)
  { 
    for (itrem2 = myList.begin(); itrem2 != myList.end(); ++itrem2)
    {
      if((*itrem1).rssival!=(*itrem2).rssival) myList1.push_back(*itrem1);
    }    
  } */
  

  double mean_energy = 0.0;
  double total_energy = 0.0;
  double remaining_energy;

   for (uint32_t nodeId = 0; nodeId < SensorNodes.GetN(); nodeId++)
  {

    Ptr<BasicEnergySource> basicSourcePtr1 = DynamicCast<BasicEnergySource>(sources.Get(nodeId));
    remaining_energy = basicSourcePtr1->GetRemainingEnergy();
    total_energy += remaining_energy;
    mean_energy = total_energy / sensor;
  }

  for (itlist = myList.begin(); itlist != myList.end(); itlist++)
  {
    std::cout << "Sensor number: " << (*itlist).Sensornum << " Anchor number: " << (*itlist).Anchornum << " Distance : " << (*itlist).Dist << " RSSI Value: " << (*itlist).rssival << " Residual Energy: " << (*itlist).residual_energy <<" Mean Energy: "<<mean_energy<< endl;
  }
  std::cout << "All list printed" << endl;
  std::cout<<" Original list size: "<<myList.size()<<endl;

  /*  for(itout=myList1.begin();itout!=myList1.end();itout++)
  {
    cout<<"Sensor number: "<< (*itout).Sensornum<<" Anchor number: "<<(*itout).Anchornum<<" Distance : "<<(*itout).Dist<<" RSSI Value: "<<(*itout).rssival<<endl;
  } */

  it = myList.begin();
  std::list<trilateration>::iterator itn = std::next(it, 1);
  std::list<trilateration>::iterator itn1 = std::next(it, 2);
  /* NS_LOG_UNCOND("Anchor number: "<<(*it).Anchornum<<" X coordinate: "<<(*it).Posx<<" Y coordinate: "<<(*it).Posy);
      NS_LOG_UNCOND("Anchor number: "<<(*itn).Anchornum<<" X coordinate: "<<(*itn).Posx<<" Y coordinate: "<<(*itn).Posy); 
      NS_LOG_UNCOND("Anchor number: "<<(*itn1).Anchornum<<" X coordinate: "<<(*itn1).Posx<<" Y coordinate: "<<(*itn1).Posy);  */

  c.x1 = (*it).Posx;
  c.x2 = (*itn).Posx;
  c.x3 = (*itn1).Posx;

  c.y1 = (*it).Posy;
  c.y2 = (*itn).Posy;
  c.y3 = (*itn1).Posy;

  c.r1 = (*it).Dist;
  c.r2 = (*itn).Dist;
  c.r3 = (*itn1).Dist;

  /* NS_LOG_UNCOND(" X coordinate: "<<c.x1<<" Y coordinate: "<<c.y1<<" Distance: "<<c.r1);
      NS_LOG_UNCOND(" X coordinate: "<<c.x2<<" Y coordinate: "<<c.y2<<" Distance: "<<c.r2); 
      NS_LOG_UNCOND(" X coordinate: "<<c.x3<<" Y coordinate: "<<c.y3<<" Distance: "<<c.r3); */

  for (uint32_t nodeId = 0; nodeId < SensorNodes.GetN(); nodeId++)
  {
    c.A = ((pow(c.r2, 2) - pow(c.r3, 2)) - (pow(c.x2, 2) - pow(c.x3, 2)) - (pow(c.y2, 2) - pow(c.y3, 2))) / 2;
    c.B = ((pow(c.r2, 2) - pow(c.r1, 2)) - (pow(c.x2, 2) - pow(c.x1, 2)) - (pow(c.y2, 2) - pow(c.y1, 2))) / 2;

    senpos.sensorposy = ((c.B * (c.x3 - c.x2)) - (c.A * (c.x1 - c.x2))) / (((c.y1 - c.y2) * (c.x3 - c.x2)) - ((c.y3 - c.y2) * (c.x1 - c.x2)));
    senpos.sensorposx = (c.A - (senpos.sensorposy * (c.y3 - c.y2))) / (c.x3 - c.x2);

    //Ptr<MobilityModel> mob = SensorNodes.Get(nodeId)->GetObject<MobilityModel>();
    //Vector pos = mob->GetPosition (); //getting position for anchor nodes
    //NS_LOG_UNCOND (" Sensor position: x=" << pos.x << ", y=" << pos.y ); //printing x and y position coordinates for anchor nodes

    NS_LOG_UNCOND("Calculated Sensor Position is x: " << senpos.sensorposx << " y: " << senpos.sensorposy);
    NS_LOG_UNCOND("Distance x: " << c.r1 << " " << c.r2 << " " << c.r3);
    NS_LOG_UNCOND("RSSI: " << (*it).rssival << " " << (*itn).rssival << " " << (*itn1).rssival);

    /* 
        Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource> (sources.Get(0)); //Sensor 
        Ptr<BasicEnergySource> basicSourcePtr1 = DynamicCast<BasicEnergySource> (sources1.Get(0)); //Anchor 0
        Ptr<BasicEnergySource> basicSourcePtr2 = DynamicCast<BasicEnergySource> (sources1.Get(1)); //Anchor 1
        Ptr<BasicEnergySource> basicSourcePtr3 = DynamicCast<BasicEnergySource> (sources1.Get(2)); //Anchor 2
        RemainingEnergy(basicSourcePtr,basicSourcePtr1,basicSourcePtr2,basicSourcePtr3);  */
    std::cout << "Iteration Number is : " << ++iteration_num << endl;
  }
  
  ofs2 << Simulator::Now().GetSeconds() << " ";
  for (an = anchorList.begin(); an != anchorList.end(); an++)
  {
    Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource>(sources1.Get((*an).anchornum));
    Ptr<DeviceEnergyModel> basicRadioModelPtr = basicSourcePtr->FindDeviceEnergyModels("ns3::WifiRadioEnergyModel").Get(0);
    remaining_energy = basicSourcePtr->GetRemainingEnergy();
    if (remaining_energy <= 2.0 && (*an).sleepmode == false)
    {
      //basicRadioModelPtr->SetAttribute("TxCurrentA", DoubleValue (0.0));
      //basicRadioModelPtr->SetAttribute("RxCurrentA", DoubleValue (0.0));
      //basicRadioModelPtr->SetAttribute("CcaBusyCurrentA", DoubleValue (0.0));
      basicRadioModelPtr->SetAttribute("IdleCurrentA", DoubleValue(0.033));
      //basicRadioModelPtr->SetAttribute("SwitchingCurrentA", DoubleValue (0.0));
      //basicRadioModelPtr->SetAttribute("SleepCurrentA", DoubleValue (0.0));
      cout << "Threshold Low Value Achieved" << endl;
      cout << "State of" << (*an).anchornum << " is changing" << endl;
      (*an).modeltype->ChangeState(WifiPhyState::SLEEP);
      //model1->ChangeState(WifiPhyState::SLEEP);
      (*an).sleepmode = true;
    }

    if (remaining_energy >= 4.5 && (*an).sleepmode == true)
    {
      //basicRadioModelPtr->SetAttribute("TxCurrentA", DoubleValue (0.0174));
      //basicRadioModelPtr->SetAttribute("RxCurrentA", DoubleValue (0.0197));
      //basicRadioModelPtr->SetAttribute("CcaBusyCurrentA", DoubleValue (0.200));
      basicRadioModelPtr->SetAttribute("IdleCurrentA", DoubleValue(0.273));
      //basicRadioModelPtr->SetAttribute("SwitchingCurrentA", DoubleValue (0.200));
      //basicRadioModelPtr->SetAttribute("SleepCurrentA", DoubleValue (0.033));
      cout << "Wake Up Mode Activated" << endl;
      (*an).modeltype->ChangeState(WifiPhyState::IDLE);
      //model1->ChangeState(WifiPhyState::IDLE);
      (*an).sleepmode = false;
    }
    cout << "A" << (*an).anchornum << ": " << (*an).sleepmode << endl;
    NS_LOG_UNCOND("Time " << Simulator::Now().GetSeconds() << " s "
                          << "\nA" << (*an).anchornum << " " << remaining_energy);
    cout << "A: " << (*an).anchornum << " State  " << (*an).modeltype->GetCurrentState() << endl;
  }

  Ptr<BasicEnergySource> basicSourcePtr1 = DynamicCast<BasicEnergySource>(sources1.Get(2));
  remaining_energy = basicSourcePtr1->GetRemainingEnergy();
  ofs2 << remaining_energy << " ";
  Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource>(sources.Get(0));
  remaining_energy = basicSourcePtr->GetRemainingEnergy();
  NS_LOG_UNCOND("Time " << Simulator::Now().GetSeconds() << " s "
                        << "\nS"
                        << "0"
                        << " " << remaining_energy);
  ofs2 << remaining_energy << endl;
  //double current=modelanchor1.Get(0)->GetCurrentA();
  //if(current==0) Simulator::Stop();
  //cout<<"Current 1 "<<modelanchor1.Get(0)->GetCurrentA()<<" Current 2 "<<modelanchor1.Get(1)->GetCurrentA()<<" Current 3 "<<modelanchor1.Get(2)->GetCurrentA()<<endl;
  //cout << model1->GetCurrentState() << endl;
  //cout<<model1->GetMaximumTimeInState(WifiPhyState::TX)<<" "<<model1->GetMaximumTimeInState(WifiPhyState::SLEEP)<<endl;
  //radiocheck(model1,model2);
  Simulator::Schedule(Seconds(0.1), &localization, AnchorNodes, SensorNodes, sources, sources1, model1, modelanchor1);
  cout << "Current 1 " << modelanchor1.Get(0)->GetCurrentA() << " Current 2 " << modelanchor1.Get(1)->GetCurrentA() << " Current 3 " << modelanchor1.Get(2)->GetCurrentA() << endl;
 
  
  cout << " Mean Energy :" << mean_energy << endl;
  cout << " Total Energy: " << total_energy << endl;

  std::cout << center(" ", 15) << " | "
            << center("RSSI", 36) << " | "
            << center("Distance", 36) << " | "
            << center("Mean Energy", 10) << " | "
            << center("Residual Energy", 10) << " | "
            << center(" ", 1) << "\n";
  std::cout << std::string(40 * 3 + 2 * 3, '-') << "\n";
  std::cout << center(" Sensor/Anchor ", 15) << " | ";
  for (uint32_t i = 0; i < 2; i++)
  {
    for (uint32_t i = 0; i < anchor; i++)
    {
      std::cout << center(std::to_string(i), 10) << " | ";
    }
  }
  std::cout << center(" ", 1) << "\n";
  std::cout << std::string(40 * 3 + 2 * 3, '-') << "\n";

  itlist = myList.begin();    
  
    std::list<trilateration>::iterator itlist1 = myList.begin();
    std::list<trilateration>::iterator itlist2 = myList.begin();
    std::advance(itlist1,1);
    std::advance(itlist2,2);  

    Ptr<BasicEnergySource> basicSourcePtr5 = DynamicCast<BasicEnergySource>(sources.Get(0));
    remaining_energy = basicSourcePtr5->GetRemainingEnergy();
    std::cout << center(std::to_string((*itlist).Sensornum), 15) << " | "
              << center(std::to_string((*itlist).rssival), 10) << " | "
              << center(std::to_string((*itlist1).rssival), 10) << " | "
              << center(std::to_string((*itlist2).rssival), 10) << " | "
              << center(std::to_string((*itlist).Dist), 10) << " | "
              << center(std::to_string((*itlist1).Dist), 10) << " | "
              << center(std::to_string((*itlist2).Dist), 10) << " | "
              << center(std::to_string(mean_energy), 11) << " | "
              << center(std::to_string(remaining_energy), 15) << " | "
              << center(" ", 1) << "\n";

    std::advance(itlist,3);
    std::advance(itlist1,4);
    std::advance(itlist2,5);
    Ptr<BasicEnergySource> basicSourcePtr6 = DynamicCast<BasicEnergySource>(sources.Get(1));
    remaining_energy = basicSourcePtr6->GetRemainingEnergy();
    std::cout << center(std::to_string((*itlist).Sensornum), 15) << " | "
              << center(std::to_string((*itlist).rssival), 10) << " | "
              << center(std::to_string((*itlist1).rssival), 10) << " | "
              << center(std::to_string((*itlist2).rssival), 10) << " | "
              << center(std::to_string((*itlist).Dist), 10) << " | "
              << center(std::to_string((*itlist1).Dist), 10) << " | "
              << center(std::to_string((*itlist2).Dist), 10) << " | "
              << center(std::to_string(mean_energy), 11) << " | "
              << center(std::to_string(remaining_energy), 15) << " | "
              << center(" ", 1) << "\n";

    std::advance(itlist,5);
    std::advance(itlist1,6);
    std::advance(itlist2,7);
    Ptr<BasicEnergySource> basicSourcePtr7 = DynamicCast<BasicEnergySource>(sources.Get(2));
    remaining_energy = basicSourcePtr7->GetRemainingEnergy();
    std::cout << center(std::to_string((*itlist).Sensornum), 15) << " | "
              << center(std::to_string((*itlist).rssival), 10) << " | "
              << center(std::to_string((*itlist1).rssival), 10) << " | "
              << center(std::to_string((*itlist2).rssival), 10) << " | "
              << center(std::to_string((*itlist).Dist), 10) << " | "
              << center(std::to_string((*itlist1).Dist), 10) << " | "
              << center(std::to_string((*itlist2).Dist), 10) << " | "
              << center(std::to_string(mean_energy), 11) << " | "
              << center(std::to_string(remaining_energy), 15) << " | "
              << center(" ", 1) << "\n";

    
  
  

  
  
  
}

int main(int argc, char *argv[])
{

  bool udp = true;
  double frequency = 868; //Mhz
  double energy = overall_energy / (anchor + sensor);
  m_energySource.SetTypeId("ns3::BasicEnergySource");
  m_deviceEnergyModel.SetTypeId("ns3::WifiRadioEnergyModel");

  radiomodel radio;

  double harvestingUpdateInterval = 0.1;
  sleepanchor sa;

  CommandLine cmd;
  cmd.AddValue("frequency", "Whether working in the 868Mhz band (other values gets rejected)", frequency);
  cmd.AddValue("simulationTime", "Simulation time in seconds", simulationTime);
  cmd.AddValue("udp", "UDP if set to 1, TCP otherwise", udp);
  cmd.Parse(argc, argv);

  ofs1 << "set terminal png" << endl;
  ofs1 << "set output 'Scenario_1.png'" << endl;
  ofs1 << "set title 'Network Lifetime'" << endl;
  ofs1 << "set xlabel 'Localization Round Number' " << endl;
  ofs1 << "set ylabel 'Energy (J)' " << endl;
  //ofs1 << "set autoscale"<<endl;
  ofs1 << "set xrange [0:70]" << endl;
  ofs1 << " set yrange [0:13]" << endl;
  ofs1 << "plot "
       << " 'Scenario_1.txt' "
       << "using 1:2 with lines title'Anchor-i[0-3]',"
       << "'Scenario_1.txt' using 1:3 with lines title 'Sensor'," << endl;

  uint32_t payloadSize; //1500 byte IP packet
  if (udp)
  {
    payloadSize = 1472; //bytes
  }
  else
  {
    payloadSize = 1448; //bytes
    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(payloadSize));
  }

  NodeContainer SensorNodes;
  SensorNodes.Create(sensor);
  NodeContainer AnchorNodes;
  AnchorNodes.Create(anchor);

  //Delay Model and Propogation loss model definition
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
  YansWifiPhyHelper phy = YansWifiPhyHelper::Default();
  phy.SetChannel(channel.Create());
  channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  channel.AddPropagationLoss("ns3::FriisPropagationLossModel",
                             "Frequency", DoubleValue(2.412e9));

  /******************************************************/

  /******************************************************/

  //Frequency Selection
  WifiMacHelper mac;
  WifiHelper wifi;
  if (frequency == 868)
  {
    wifi.SetStandard(WIFI_PHY_STANDARD_80211b);
  }
  else
  {
    std::cout << "Wrong frequency value!" << std::endl;
    return 0;
  }

  ostringstream oss;
  oss << "HtMcs" << 1;
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(oss.str()),
                               "ControlMode", StringValue(oss.str()));

  Ssid ssid = Ssid("ns3-80211n");
  mac.SetType("ns3::StaWifiMac",
              "Ssid", SsidValue(ssid));
  NetDeviceContainer sensorDevice;
  sensorDevice = wifi.Install(phy, mac, SensorNodes);
  mac.SetType("ns3::ApWifiMac",
              "Ssid", SsidValue(ssid));
  //NetDeviceContainer anchorDevice;
  anchorDevice = wifi.Install(phy, mac, AnchorNodes);

  /* Exact Location Mobility Method */

  /*  MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
    positionAlloc->Add (Vector (7.5, 7.5, 15.0));
    positionAlloc->Add (Vector (10.0, 5.0, 0.0));
    positionAlloc->Add (Vector (10.0, 10.0, 0.0));
    positionAlloc->Add (Vector (5.0, 5.0, 0.0));

    mobility.SetPositionAllocator (positionAlloc);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    
    mobility.Install (SensorNodes); //setting constant velocity model to sensor nodes
	  mobility.Install (AnchorNodes); //setting constant position model to anchor nodes */

  /* ************************************* */

  /* Random Mobility Model */

  Ptr<UniformRandomVariable> randomizer1 = CreateObject<UniformRandomVariable>();
  randomizer1->SetAttribute("Min", DoubleValue(0));
  randomizer1->SetAttribute("Max", DoubleValue(75));

  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  mobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
                                "X", PointerValue(randomizer1),
                                "Y", PointerValue(randomizer1),
                                "Z", PointerValue(randomizer1));
  mobility.Install(SensorNodes); //setting constant velocity model to sensor nodes
  mobility.Install(AnchorNodes); //setting constant position model to anchor nodes

  //Internet Settings
  InternetStackHelper stack;
  stack.Install(AnchorNodes); //installing internet to anchor nodes
  stack.Install(SensorNodes); //installing internet to sensor nod
  Ipv4AddressHelper address;
  address.SetBase("192.168.1.0", "255.255.255.0");
  Ipv4InterfaceContainer staNodeInterface;
  Ipv4InterfaceContainer apNodeInterface;
  staNodeInterface = address.Assign(sensorDevice); //assigning ip to sensor
  apNodeInterface = address.Assign(anchorDevice);  //assigning ip to anchor

  //Energy Resource Settings
  BasicEnergySourceHelper basicSourceHelper;
  basicSourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(energy)); //setting initial energy;
  basicSourceHelper.Set("BasicEnergySupplyVoltageV", DoubleValue(1.0));
  basicSourceHelper.Set("PeriodicEnergyUpdateInterval", TimeValue(Seconds(0.1)));
  EnergySourceContainer sources = basicSourceHelper.Install(SensorNodes);

  BasicEnergySourceHelper basicSourceHelper1;
  basicSourceHelper1.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(energy)); //setting initial energy;
  //basicSourceHelper1.Set("BasicEnergySupplyVoltageV",DoubleValue (2.0));
  basicSourceHelper1.Set("PeriodicEnergyUpdateInterval", TimeValue(Seconds(0.1)));
  EnergySourceContainer sources1 = basicSourceHelper1.Install(AnchorNodes);

  /* device energy model */
  WifiRadioEnergyModelHelper radioEnergyHelper;
  // configure sensor energy model
  radioEnergyHelper.Set("TxCurrentA", DoubleValue(0.0174)); //setting power 0.0174
  radioEnergyHelper.Set("RxCurrentA", DoubleValue(0.0197));
  radioEnergyHelper.Set("CcaBusyCurrentA", DoubleValue(0.273));
  radioEnergyHelper.Set("IdleCurrentA", DoubleValue(0.273));
  radioEnergyHelper.Set("SwitchingCurrentA", DoubleValue(0.273));
  radioEnergyHelper.Set("SleepCurrentA", DoubleValue(0.033));

  // install device model
  DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install(sensorDevice, sources);

  //DeviceEnergyModelContainer deviceModels1 = radioEnergyHelper.Install (anchorDevice, sources1);

  BasicEnergyHarvesterHelper basicHarvesterHelper;
  // configure energy harvester
  basicHarvesterHelper.Set("PeriodicHarvestedPowerUpdateInterval", TimeValue(Seconds(harvestingUpdateInterval)));

  basicHarvesterHelper.Set("HarvestablePower", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=0.4]"));
  // install harvester on all energy sources
  EnergyHarvesterContainer harvesters = basicHarvesterHelper.Install(sources1);

  Ptr<WifiRadioEnergyModel> model1 = m_deviceEnergyModel.Create<WifiRadioEnergyModel>();
  Ptr<WifiRadioEnergyModel> gokhan = m_deviceEnergyModel.Create<WifiRadioEnergyModel>();

  //cout<<radioEnergyHelper.GetCurrentState();

  //model1->ChangeState (1);
  /*  double txCurrent=model1->GetTxCurrentA(); 
    model1->SetIdleCurrentA(0.273);
    cout<<txCurrent<<endl; */
  //FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Set("IdleCurrentA",DoubleValue(0.273));

  //Sensor
  /*  model1->SetEnergySource (sources1.Get(0));          			
			    sources1.Get(0)->AppendDeviceEnergyModel (model1);
          //DeviceEnergyModelContainer modelsensor =sources.Get(0)->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel"); 
          DeviceEnergyModelContainer modelanchor=sources1.Get(0)->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel"); */
  DeviceEnergyModelContainer modelanchor1;

  /*  anchormodel1.SetEnergySource (sources1.Get(0));
          sources1.Get(0)->AppendDeviceEnergyModel (model1);
          anchormodel1.SetNode(AnchorNodes.Get(0));
          anchormodel1.SetCurrentA(0.5);
          cout<<"Current of new model "<<anchormodel1.GetCurrentA()<<endl; */

  /* WifiRadioEnergyModel wifimodel;          
          wifimodel.SetEnergySource(sources1.Get(0));
          sources1.Get(0)->AppendDeviceEnergyModel (model1);
          wifimodel.SetEnergySource(sources1.Get(1));
          sources1.Get(1)->AppendDeviceEnergyModel (model1);
          wifimodel.SetEnergySource(sources1.Get(2));
          sources1.Get(2)->AppendDeviceEnergyModel (model1);
          //wifimodel.SetEnergySource(sources.Get(0));          
          cout<<wifimodel.GetCurrentState()<<endl; */
  //cout<<deviceModels1.Get(0)->GetCurrentA()<<endl;
  /* 
              model2->SetEnergySource (sources1.Get(0));
              sources1.Get(0)->AppendDeviceEnergyModel (model2);
              modelanchor1.Add(sources1.Get(0)->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel")); */

  /*  for (int nodeId = 0; nodeId < anchor; nodeId++)
  {
    model1->SetEnergySource(sources1.Get(nodeId));
    sources1.Get(nodeId)->AppendDeviceEnergyModel(model1);
    modelanchor1.Add(sources1.Get(nodeId)->FindDeviceEnergyModels("ns3::WifiRadioEnergyModel"));
    //cout<<sources1.Get(nodeId)->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get(nodeId)<<"Device Model "<<endl;
  } */

  Ptr<WifiRadioEnergyModel> radiox[AnchorNodes.GetN()];

  for (uint32_t i = 0; i < AnchorNodes.GetN(); i++)
  {
    sa.anchornum = i;
    sa.sleepmode = false;
    radiox[i] = m_deviceEnergyModel.Create<WifiRadioEnergyModel>();
    radiox[i]->SetEnergySource(sources1.Get(i));
    sa.modeltype = radiox[i];
    anchorList.push_back(sa);
    radiolist.push_back(radio);
    sources1.Get(i)->AppendDeviceEnergyModel(radiox[i]);
    modelanchor1.Add(sources1.Get(i)->FindDeviceEnergyModels("ns3::WifiRadioEnergyModel"));
  }

  /* radiox[2]->ChangeState(WifiPhyState::SLEEP);
  cout << radiox[2]->GetCurrentState() << endl;
  cout << radiox[0]->GetCurrentState() << endl; */
  /*
  for (uint32_t i = 0; i < AnchorNodes.GetN(); i++)
  {
    sa.anchornum = i; 
    sa.sleepmode = false;
    anchorList.push_back(sa);
    //Ptr<WifiRadioEnergyModel> radiox = m_deviceEnergyModel.Create<WifiRadioEnergyModel>();
    //radiox->SetEnergySource(sources1.Get(i));
    radio.modeltype = radiox;
    radiolist.push_back(radio);
    cout << radiox->GetCurrentState() << endl;
    radiox->ChangeState(WifiPhyState::SLEEP);
    cout << radiox->GetCurrentState() << endl;
  }*/

  WifiRadioEnergyModel wifimodel;
  wifimodel.SetEnergySource(sources1.Get(0));
  cout << "State of model " << wifimodel.GetCurrentState() << endl;

  /*  Ptr<DeviceEnergyModel> ModelNum1=modelanchor1.Get(0);
          ModelNum1.ChangeState(0); */

  //cout<<model1->GetTxCurrentA();

  //model1->ChangeState (WifiPhyState::TX);

  Simulator::Schedule(Seconds(1), &localization, AnchorNodes, SensorNodes, sources, sources1, model1, modelanchor1);
  radiocheck(model1, gokhan);

  /* Ptr<BasicEnergySource> basicSourcePtr3 = DynamicCast<BasicEnergySource> (sources.Get(0));
    Simulator::Schedule (Minutes (0), &RemainingEnergy,basicSourcePtr3);  */

  /* Setting applications */
  ApplicationContainer serverApp, sinkApp;
  if (udp)
  {
    //UDP flow
    UdpServerHelper myServer(9);
    serverApp = myServer.Install(SensorNodes.Get(0));
    serverApp.Start(Seconds(0.0));
    serverApp.Stop(Seconds(simulationTime + 1));
    UdpClientHelper myClient(staNodeInterface.GetAddress(0), 9);
    myClient.SetAttribute("MaxPackets", UintegerValue(4294967295u));
    myClient.SetAttribute("Interval", TimeValue(Time("0.00001"))); //packets/s
    myClient.SetAttribute("PacketSize", UintegerValue(payloadSize));
    ApplicationContainer clientApp = myClient.Install(AnchorNodes.Get(0));
    clientApp.Start(Seconds(1.0));
    clientApp.Stop(Seconds(simulationTime + 1));
  }
  else
  {
    //TCP flow
    uint16_t port = 50000;
    Address apLocalAddress(InetSocketAddress(Ipv4Address::GetAny(), port));
    PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory", apLocalAddress);
    sinkApp = packetSinkHelper.Install(SensorNodes.Get(0));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simulationTime + 1));
    OnOffHelper onoff("ns3::TcpSocketFactory", Ipv4Address::GetAny());
    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    onoff.SetAttribute("PacketSize", UintegerValue(payloadSize));
    onoff.SetAttribute("DataRate", DataRateValue(DataRate("150Mbps"))); //bit/s1000000000
    ApplicationContainer apps;
    AddressValue remoteAddress(InetSocketAddress(staNodeInterface.GetAddress(0), port));
    onoff.SetAttribute("Remote", remoteAddress);
    apps.Add(onoff.Install(AnchorNodes.Get(0)));
    apps.Start(Seconds(1.0));
    apps.Stop(Seconds(simulationTime + 1));
  }

  for (uint32_t nodeId = 0; nodeId < 3; nodeId++)
  {
    Ptr<MobilityModel> mob = AnchorNodes.Get(nodeId)->GetObject<MobilityModel>();
    Vector pos = mob->GetPosition();                                                    //getting position for anchor nodes
    NS_LOG_UNCOND(" Anchor " << nodeId << " position: x=" << pos.x << ", y=" << pos.y); //printing x and y position coordinates for anchor nodes
  }

  Ptr<MobilityModel> mob = SensorNodes.Get(0)->GetObject<MobilityModel>();
  Vector pos = mob->GetPosition();                                   //getting position for anchor nodes
  NS_LOG_UNCOND(" Sensor position: x=" << pos.x << ", y=" << pos.y); //printing x and y position coordinates for anchor nodes

  Ipv4GlobalRoutingHelper::PopulateRoutingTables();

  Ptr<FriisPropagationLossModel> model = CreateObject<FriisPropagationLossModel>();

  pAnim = new AnimationInterface("sc1.xml"); //NetAnim file creation

  for (uint32_t nodeId = 0; nodeId < sensor; nodeId++)
  {
    pAnim->UpdateNodeDescription(SensorNodes.Get(nodeId), "S" + std::to_string(nodeId)); //setting description S to sensor node
    pAnim->UpdateNodeColor(SensorNodes.Get(nodeId), 255, 0, 0);
  }
  for (uint32_t nodeId = 0; nodeId < anchor; nodeId++)
  {
    pAnim->UpdateNodeDescription(AnchorNodes.Get(nodeId), "A" + std::to_string(nodeId)); //setting description A to anchor node
    pAnim->UpdateNodeColor(AnchorNodes.Get(nodeId), 0, 255, 0);                          //updating color to anchor node
  }
  Simulator::Stop(Seconds(simulationTime + 1)); //stopping the simulation
  Simulator::Run();
  Simulator::Destroy();

  return 0;
}
