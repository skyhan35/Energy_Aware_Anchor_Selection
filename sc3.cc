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
#include "ns3/timer.h"
#include "ns3/simulator.h"
#include "ns3/simulation-singleton.h"
#include "ns3/log.h"
#include <iostream>
#include <fstream>
#include <list>
#include <unordered_set>



using namespace ns3;
using namespace std;


NS_LOG_COMPONENT_DEFINE ("sc3");

double sensor=1;
double anchor=10;
double overall_energy = 500;
double simulationTime = 1000;
double iteration_num=0;
AnimationInterface * pAnim = 0;
ObjectFactory m_energySource; 
ObjectFactory m_deviceEnergyModel;
ObjectFactory m_radioEnergy;
ObjectFactory energysourcecontainer;

ofstream ofs1 ("Scenario_3.plt", ofstream::out);
ofstream ofs2 ("Scenario_3.txt",ofstream::out);



class trilateration 
{
public: 
	int Dist;
  double Posx;
  double Posy;
  int Anchornum;
  int Sensornum;
  double rssival;
};

class sensorpos{
  
  public:
  double sensorposx;
  double sensorposy;
  double sensornumber;
};

class calculation{

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

class sleepanchor{

  public:
  int anchornum;
  bool sleepmode;
};

class sensorswitch{
  public:
  int sensornumber;
  bool comparethreshold;
};

std::list<trilateration> myList;
std::list<trilateration>::iterator it;
std::list<trilateration>::iterator itenergy;
std::list<sensorpos> posList;
std::list<sleepanchor> anchorList;
std::list<sleepanchor>::iterator an;
std::list<sleepanchor>::iterator an1;
std::list<sensorswitch> sensorList;
std::unordered_set<int> s;
int tmp=1;


/*  void RemainingEnergy (Ptr<BasicEnergySource> basicSourcePtr)
{
  NS_LOG_UNCOND("Time "<<Simulator::Now ().GetSeconds ()<<" s "<<" Remaining Energy of Sensor: "<<basicSourcePtr -> GetRemainingEnergy() ); // Get remaining energy
  ofs2 << Simulator::Now ().GetSeconds ()<< " " <<basicSourcePtr -> GetRemainingEnergy()<<endl;
  //Simulator::Schedule (Seconds (0.1), &RemainingEnergy, basicSourcePtr);
} */
/* void energy(Ptr<BasicEnergySource> source, int n, double seconds)
{
	// obtain remaining energy from source
	double remainingEnergy = source->GetRemainingEnergy ();
	std::cout<<"Node "<<n<<" Remaining energy is " << remainingEnergy << " at Time " << Simulator::Now().GetSeconds() << "s" <<std::endl;
	Simulator::Schedule(Seconds(seconds),&energy,source,n,seconds);
} */
/* 
void RemainingEnergy (Ptr<BasicEnergySource> basicSourcePtr,Ptr<BasicEnergySource> basicSourcePtr1,Ptr<BasicEnergySource> basicSourcePtr2,Ptr<BasicEnergySource> basicSourcePtr3,Ptr<BasicEnergySource> basicSourcePtr4,Ptr<BasicEnergySource> basicSourcePtr5,Ptr<BasicEnergySource> basicSourcePtr6,Ptr<BasicEnergySource> basicSourcePtr7,Ptr<BasicEnergySource> basicSourcePtr8,Ptr<BasicEnergySource> basicSourcePtr9,Ptr<BasicEnergySource> basicSourcePtr10)
{
  NS_LOG_UNCOND("Time "<<Simulator::Now ().GetSeconds ()<<" s "<< "\nA0: " <<basicSourcePtr1 -> GetRemainingEnergy()<<"\nA1: "<<basicSourcePtr2 -> GetRemainingEnergy()<<"\nA2: "<<basicSourcePtr3 -> GetRemainingEnergy()<<"\nA3: "<<basicSourcePtr4 -> GetRemainingEnergy()<<"\nA4: "<<basicSourcePtr5 -> GetRemainingEnergy()<<"\nA5: "<<basicSourcePtr6 -> GetRemainingEnergy()<<"\nA6: "<<basicSourcePtr7 -> GetRemainingEnergy()<<"\nA7: "<<basicSourcePtr8 -> GetRemainingEnergy()<<"\nA8: "<<basicSourcePtr9 -> GetRemainingEnergy()<<"\nA9: "<<basicSourcePtr10 -> GetRemainingEnergy()<<"\nSensor: "<<basicSourcePtr -> GetRemainingEnergy() );
  ofs2 << Simulator::Now ().GetSeconds ()<< " " << basicSourcePtr1 -> GetRemainingEnergy()<< " " <<basicSourcePtr2 -> GetRemainingEnergy()<< " " <<basicSourcePtr3 -> GetRemainingEnergy()<< " " <<basicSourcePtr -> GetRemainingEnergy()<< " " <<basicSourcePtr4 -> GetRemainingEnergy()<< " " <<basicSourcePtr5 -> GetRemainingEnergy()<< " " <<basicSourcePtr6 -> GetRemainingEnergy()<< " " <<basicSourcePtr7 -> GetRemainingEnergy()<< " " <<basicSourcePtr8 -> GetRemainingEnergy()<< " " <<basicSourcePtr9 -> GetRemainingEnergy()<< " " <<basicSourcePtr10 -> GetRemainingEnergy()<<endl;
  //Simulator::Schedule (Seconds (0.01), &RemainingEnergy, basicSourcePtr,basicSourcePtr1,basicSourcePtr2,basicSourcePtr3);
  for (itenergy = myList.begin(); itenergy!=myList.end(); ++itenergy)
      {
          Ptr<Node> node=AnchorNodes.Get ((*itenergy).Anchornum);
			    Ptr<BasicEnergySource> source = m_energySource.Create<BasicEnergySource> ();			    			    
			    double remainingEnergy = source->GetRemainingEnergy ();           
      }  
  
  
} */


/* void
RemainingEnergy (double oldValue, double remainingEnergy)
{
  NS_LOG_UNCOND("Time"<<Simulator::Now ().GetSeconds ()<< " Remaining energy " << remainingEnergy);  
}  */

/* void energy(Ptr<BasicEnergySource> source)
{
	//obtain remaining energy from source
	double remainingEnergy = source->GetRemainingEnergy ();
	std::cout<<" Remaining energy is " << remainingEnergy << " at Time " << Simulator::Now().GetSeconds() << "s" <<std::endl;
  //ofs2<<Simulator::Now().GetSeconds()<<" "<<remainingEnergy<<std::endl;	
	Simulator::Schedule(Seconds(0.1),&energy,source);
	if(remainingEnergy<0.5) Simulator::Stop();
} */

 





void localization(NodeContainer AnchorNodes, NodeContainer SensorNodes,EnergySourceContainer sources,EnergySourceContainer sources1) //trilateration method
{ 
  
  m_energySource.SetTypeId ("ns3::BasicEnergySource");
	m_deviceEnergyModel.SetTypeId ("ns3::WifiRadioEnergyModel");
	Ptr<FriisPropagationLossModel> model = CreateObject<FriisPropagationLossModel> ();       
  	
   
	

  trilateration t;
  sensorpos senpos;
  calculation c;  



  /* for(uint32_t i = 0; i < AnchorNodes.GetN(); i++)
  {
    sa.anchornum=i;
    sa.sleepmode=false;
    anchorList.push_back(sa);
    
  } */
  //cout<<"Something happened";
 /*  for(an=anchorList.begin();an!=anchorList.end();an++)
  {
    for(an1=std::next(an,1);an1!=anchorList.end();an1++)
    {
        if(an!=an1)
        {
          if((*an).anchornum==(*an1).anchornum) anchorList.erase(an1);
        }
    }
  } */

 /* an=anchorList.begin();
 while(an!=anchorList.end()){
   if((*an).anchornum==n)
   {
     anchorList.erase(an++);
     an++;
   }
   n++;
 } */

  
  
  
  /* for(uint32_t i = 0; i < SensorNodes.GetN(); i++)
  {
    sensorList.push_back(i);
  } */




   for (uint32_t i = 0; i < anchor; i++)
      {
          for (uint32_t nodeId = 0; nodeId < 1; nodeId++)
          {
            Ptr<MobilityModel> model1 = AnchorNodes.Get(i)->GetObject<MobilityModel>();
			      Ptr<MobilityModel> model2 = SensorNodes.Get(nodeId)->GetObject<MobilityModel>();
			      double distance = model1->GetDistanceFrom (model2); // calculate distance from sensor to anchor nodes divided by 10 to convert it to meter from centi
			      double rssi = model->CalcRxPower (5.0, model2, model1); // calculates RSSI and 5.0 is the transmit power
            Ptr<MobilityModel> mob = AnchorNodes.Get(i)->GetObject<MobilityModel>();
			      Vector pos = mob->GetPosition ();

            t.Dist=distance;
            t.Posx=pos.x;
            t.Posy=pos.y;
            t.Anchornum=i;
            t.Sensornum=nodeId;
            t.rssival=rssi;
            myList.push_back(t);             
            NS_LOG_UNCOND("Anchor number: "<<t.Anchornum<<" X coordinate: "<<t.Posx<<" Y coordinate: "<<t.Posy<<" Distance: "<<distance);        


          }
      }  
     
		
     
      
      
      //std::list<trilateration>::iterator itn = std::next(it,1);
      //std::list<trilateration>::iterator itn1 = std::next(it,2);
      /* NS_LOG_UNCOND("Anchor number: "<<(*it).Anchornum<<" X coordinate: "<<(*it).Posx<<" Y coordinate: "<<(*it).Posy);
      NS_LOG_UNCOND("Anchor number: "<<(*itn).Anchornum<<" X coordinate: "<<(*itn).Posx<<" Y coordinate: "<<(*itn).Posy); 
      NS_LOG_UNCOND("Anchor number: "<<(*itn1).Anchornum<<" X coordinate: "<<(*itn1).Posx<<" Y coordinate: "<<(*itn1).Posy);  */
      for(int n=2;n<10;n++){
        it=myList.begin();
        std::list<trilateration>::iterator itn = std::next(it,1);
        std::list<trilateration>::iterator itn1 = std::next(it,n);
            
            c.x1=(*it).Posx;
            c.x2=(*itn).Posx;
            c.x3=(*itn1).Posx;

            c.y1=(*it).Posy;
            c.y2=(*itn).Posy;
            c.y3=(*itn1).Posy;

            c.r1=(*it).Dist;
            c.r2=(*itn).Dist;
            c.r3=(*itn1).Dist;
            

      //NS_LOG_UNCOND("Anchor number: "<<(*it).Anchornum<<" X coordinate: "<<(*it).Posx<<" Y coordinate: "<<(*it).Posy);
      //NS_LOG_UNCOND("Anchor number: "<<(*itn).Anchornum<<" X coordinate: "<<(*itn).Posx<<" Y coordinate: "<<(*itn).Posy); 
      //NS_LOG_UNCOND("Anchor number: "<<(*itn1).Anchornum<<" X coordinate: "<<(*itn1).Posx<<" Y coordinate: "<<(*itn1).Posy);  

      c.A=((pow(c.r2,2)-pow(c.r3,2))-(pow(c.x2,2)-pow(c.x3,2))-(pow(c.y2,2)-pow(c.y3,2)))/2;
        c.B=((pow(c.r2,2)-pow(c.r1,2))-(pow(c.x2,2)-pow(c.x1,2))-(pow(c.y2,2)-pow(c.y1,2)))/2;


        senpos.sensorposy=((c.B*(c.x3-c.x2))-(c.A*(c.x1-c.x2)))/(((c.y1-c.y2)*(c.x3-c.x2))-((c.y3-c.y2)*(c.x1-c.x2)));
        senpos.sensorposx=(c.A-(senpos.sensorposy*(c.y3-c.y2)))/(c.x3-c.x2);                     
        
        
        

        
        NS_LOG_UNCOND("Sensor Position Calculated with these anchors : "<<"A"<<(*it).Anchornum<<" A"<<(*itn).Anchornum<<" A"<<(*itn1).Anchornum);
        NS_LOG_UNCOND("Calculated Sensor Position is x: "<<senpos.sensorposx<<" y: "<<senpos.sensorposy);
        NS_LOG_UNCOND("Distance x: "<<c.r1<<" "<<c.r2<<" "<<c.r3);
        NS_LOG_UNCOND("RSSI: "<<(*it).rssival<<" "<<(*itn).rssival<<" "<<(*itn1).rssival);
        


       

      }
      

      //NS_LOG_UNCOND(" X coordinate: "<<c.x1<<" Y coordinate: "<<c.y1<<" Distance: "<<c.r1);
      //NS_LOG_UNCOND(" X coordinate: "<<c.x2<<" Y coordinate: "<<c.y2<<" Distance: "<<c.r2); 
      //NS_LOG_UNCOND(" X coordinate: "<<c.x3<<" Y coordinate: "<<c.y3<<" Distance: "<<c.r3); 

      /* for (uint32_t nodeId= 0; nodeId < SensorNodes.GetN(); nodeId++)
      {
        c.A=((pow(c.r2,2)-pow(c.r3,2))-(pow(c.x2,2)-pow(c.x3,2))-(pow(c.y2,2)-pow(c.y3,2)))/2;
        c.B=((pow(c.r2,2)-pow(c.r1,2))-(pow(c.x2,2)-pow(c.x1,2))-(pow(c.y2,2)-pow(c.y1,2)))/2;


        senpos.sensorposy=((c.B*(c.x3-c.x2))-(c.A*(c.x1-c.x2)))/(((c.y1-c.y2)*(c.x3-c.x2))-((c.y3-c.y2)*(c.x1-c.x2)));
        senpos.sensorposx=(c.A-(senpos.sensorposy*(c.y3-c.y2)))/(c.x3-c.x2);              
        
        
        
        

        //Ptr<MobilityModel> mob = SensorNodes.Get(nodeId)->GetObject<MobilityModel>();
        //Vector pos = mob->GetPosition (); //getting position for anchor nodes
        //NS_LOG_UNCOND (" Sensor position: x=" << pos.x << ", y=" << pos.y ); //printing x and y position coordinates for anchor nodes

        NS_LOG_UNCOND("Calculated Sensor Position is x: "<<senpos.sensorposx<<" y: "<<senpos.sensorposy);
        NS_LOG_UNCOND("Distance x: "<<c.r1<<" "<<c.r2<<" "<<c.r3);
        NS_LOG_UNCOND("RSSI: "<<(*it).rssival<<" "<<(*itn).rssival<<" "<<(*itn1).rssival);

        


        Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource> (sources.Get(0)); //Sensor 
        Ptr<BasicEnergySource> basicSourcePtr1 = DynamicCast<BasicEnergySource> (sources1.Get(0)); //Anchor 0
        Ptr<BasicEnergySource> basicSourcePtr2 = DynamicCast<BasicEnergySource> (sources1.Get(1)); //Anchor 1
        Ptr<BasicEnergySource> basicSourcePtr3 = DynamicCast<BasicEnergySource> (sources1.Get(2)); //Anchor 2
        RemainingEnergy(basicSourcePtr,basicSourcePtr1,basicSourcePtr2,basicSourcePtr3);   
        std::cout<<"Iteration Number is : "<<++iteration_num<<endl;
        
        

      }	 */

   /*    for (itenergy = myList.begin(); itenergy!=myList.end(); ++itenergy)
      {
          Ptr<Node> node=AnchorNodes.Get ((*itenergy).Anchornum);
			    Ptr<BasicEnergySource> source = m_energySource.Create<BasicEnergySource> ();
			    source->SetInitialEnergy(overall_energy/(anchor+sensor)); 			    
			    node->AggregateObject (source);			    
			    Ptr<WifiRadioEnergyModel> model =
			    m_deviceEnergyModel.Create<WifiRadioEnergyModel> ();			    
			    model->SetEnergySource (source);			    
			    source->AppendDeviceEnergyModel (model);			    
			    DeviceEnergyModelContainer models =source->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel");
			    double sec=(sensor/anchor)*0.01;
          int n=(int)((*itenergy).Anchornum);  
			    energy(source,n,sec);            
      }  
 */
      ofs2 << Simulator::Now ().GetSeconds ()<<" ";
      for (an= anchorList.begin(); an!=anchorList.end(); an++)
      { 
          Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource> (sources1.Get((*an).anchornum));
          double remaining_energy=basicSourcePtr	-> GetRemainingEnergy();          
          if(remaining_energy<=1.0 && (*an).sleepmode==false){
            Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource>  (sources1.Get((*an).anchornum));
            Ptr<DeviceEnergyModel> basicRadioModelPtr = basicSourcePtr->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get (0);
            basicRadioModelPtr->SetAttribute("RxCurrentA", DoubleValue (0.0));
            basicRadioModelPtr->SetAttribute("TxCurrentA", DoubleValue (0.0));
            basicRadioModelPtr->SetAttribute("IdleCurrentA", DoubleValue (0.0));
            basicRadioModelPtr->SetAttribute("CcaBusyCurrentA", DoubleValue (0.0));
            (*an).sleepmode=true;
          }

           if(remaining_energy>=3.0 && (*an).sleepmode==true){
            Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource>  (sources1.Get((*an).anchornum));
            Ptr<DeviceEnergyModel> basicRadioModelPtr = basicSourcePtr->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get (0);
            basicRadioModelPtr->SetAttribute("RxCurrentA", DoubleValue (0.0197));
            basicRadioModelPtr->SetAttribute("TxCurrentA", DoubleValue (0.0174));
            basicRadioModelPtr->SetAttribute("IdleCurrentA", DoubleValue (0.273));
            basicRadioModelPtr->SetAttribute("CcaBusyCurrentA", DoubleValue (0.273));
            (*an).sleepmode=false;
          }
          
          NS_LOG_UNCOND("Time "<<Simulator::Now ().GetSeconds ()<<" s "<< "\nA"<<(*an).anchornum <<" "<<remaining_energy); 
          ofs2 <<basicSourcePtr -> GetRemainingEnergy()<<" ";                       
      }  
      ofs2<<endl;      

       /* Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource> (sources1.Get(0));
       Ptr<DeviceEnergyModel> basicRadioModelPtr = basicSourcePtr->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get (0);
       basicRadioModelPtr->SetAttribute("RxCurrentA", DoubleValue (0.0));
       basicRadioModelPtr->SetAttribute("TxCurrentA", DoubleValue (0.0));
       basicRadioModelPtr->SetAttribute("IdleCurrentA", DoubleValue (0.0));
       basicRadioModelPtr->SetAttribute("CcaBusyCurrentA", DoubleValue (0.0));
       Ptr<Node> node=AnchorNodes.Get (0); */
       //node->SetDown(1);
     

          
          


         /*     
        Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource> (sources.Get(0)); //Anchor 0 
        Ptr<BasicEnergySource> basicSourcePtr1 = DynamicCast<BasicEnergySource> (sources1.Get(0)); //Anchor 0
        Ptr<BasicEnergySource> basicSourcePtr2 = DynamicCast<BasicEnergySource> (sources1.Get(1)); //Anchor 1
        Ptr<BasicEnergySource> basicSourcePtr3 = DynamicCast<BasicEnergySource> (sources1.Get(2)); //Anchor 2
        Ptr<BasicEnergySource> basicSourcePtr4 = DynamicCast<BasicEnergySource> (sources1.Get(3)); //Anchor 3
        Ptr<BasicEnergySource> basicSourcePtr5 = DynamicCast<BasicEnergySource> (sources1.Get(4)); //Anchor 4
        Ptr<BasicEnergySource> basicSourcePtr6 = DynamicCast<BasicEnergySource> (sources1.Get(5)); //Anchor 5
        Ptr<BasicEnergySource> basicSourcePtr7 = DynamicCast<BasicEnergySource> (sources1.Get(6)); //Anchor 6
        Ptr<BasicEnergySource> basicSourcePtr8 = DynamicCast<BasicEnergySource> (sources1.Get(7)); //Anchor 7
        Ptr<BasicEnergySource> basicSourcePtr9 = DynamicCast<BasicEnergySource> (sources1.Get(8)); //Anchor 8
        Ptr<BasicEnergySource> basicSourcePtr10 = DynamicCast<BasicEnergySource> (sources1.Get(9)); //Anchor 9 */

        //RemainingEnergy(basicSourcePtr,basicSourcePtr1,basicSourcePtr2,basicSourcePtr3,basicSourcePtr4,basicSourcePtr5,basicSourcePtr6,basicSourcePtr7,basicSourcePtr8,basicSourcePtr9,basicSourcePtr10); 
      
        //NS_LOG_UNCOND("Time "<<Simulator::Now ().GetSeconds ()<<" s "<< "\nA0: " <<basicSourcePtr1 -> GetRemainingEnergy()<<"\nA1: "<<basicSourcePtr2 -> GetRemainingEnergy()<<"\nA2: "<<basicSourcePtr3 -> GetRemainingEnergy()<<"\nA3: "<<basicSourcePtr4 -> GetRemainingEnergy()<<"\nA4: "<<basicSourcePtr5 -> GetRemainingEnergy()<<"\nA5: "<<basicSourcePtr6 -> GetRemainingEnergy()<<"\nA6: "<<basicSourcePtr7 -> GetRemainingEnergy()<<"\nA7: "<<basicSourcePtr8 -> GetRemainingEnergy()<<"\nA8: "<<basicSourcePtr9 -> GetRemainingEnergy()<<"\nA9: "<<basicSourcePtr10 -> GetRemainingEnergy()<<"\nSensor: "<<basicSourcePtr -> GetRemainingEnergy() );

        
        std::cout<<"Iteration Number is : "<<++iteration_num<<endl; 
        Simulator::Schedule (Seconds (0.1), &localization, AnchorNodes,SensorNodes,sources,sources1);
        //localization(AnchorNodes,SensorNodes,sources,sources1);
       
    //Simulator::Stop (Minutes (simulationTime + 1));
      
}







int main (int argc, char *argv[])
{

    bool udp = false;     
    double frequency = 868; //Mhz
    double energy = overall_energy/(anchor+sensor);
    m_energySource.SetTypeId ("ns3::BasicEnergySource");
    m_deviceEnergyModel.SetTypeId ("ns3::WifiRadioEnergyModel");    
    double harvestingUpdateInterval = 0.1;    
    sleepanchor sa;

    CommandLine cmd;
    cmd.AddValue ("frequency", "Whether working in the 868Mhz band (other values gets rejected)", frequency);
    cmd.AddValue ("simulationTime", "Simulation time in seconds", simulationTime);
    cmd.AddValue ("udp", "UDP if set to 1, TCP otherwise", udp);
    cmd.Parse (argc,argv);


    ofs1 << "set terminal png"<<endl;
	  ofs1 << "set output 'Scenario_3.png'"<<endl;
	  ofs1 << "set title 'Scenario_3'"<<endl;
	  ofs1 << "set xlabel 'Time (Second)' "<<endl;
	  ofs1 << "set ylabel 'Energy (J)' "<<endl;	 
    ofs1 << "set autoscale"<<endl;    
	  ofs1 << "plot "<<" 'Scenario_3.txt' " <<"using 1:2 with linespoints title'Anchor0',"<<"'Scenario_3.txt' using 1:3 with linespoints title 'Anchor1',"<<"'Scenario_3.txt' using 1:4 with linespoints title 'Anchor2',"<<"'Scenario_3.txt' using 1:5 with linespoints title 'Anchor3',"<<"'Scenario_3.txt' using 1:6 with linespoints title 'Anchor4',"<<"'Scenario_3.txt' using 1:7 with linespoints title 'Anchor5',"<<"'Scenario_3.txt' using 1:8 with linespoints title 'Anchor6',"<<"'Scenario_3.txt' using 1:9 with linespoints title 'Anchor7',"<<"'Scenario_3.txt' using 1:10 with linespoints title 'Anchor8',"<<"'Scenario_3.txt' using 1:11 with linespoints title 'Anchor9',"<<endl;


  
  

    uint32_t payloadSize; //1500 byte IP packet
    if (udp)
      {
        payloadSize = 1472; //bytes
      }
    else
      {
        payloadSize = 1448; //bytes
        Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (payloadSize));
      }

   NodeContainer SensorNodes;
	 SensorNodes.Create (sensor); 
	 NodeContainer AnchorNodes;
	 AnchorNodes.Create (anchor); 

    //Delay Model and Propogation loss model definition
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
    YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
    phy.SetChannel (channel.Create ());
    channel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel"); 
    channel.AddPropagationLoss ("ns3::FriisPropagationLossModel",  
                                 "Frequency",DoubleValue(2.412e9));

    //Frequency Selection
    WifiMacHelper mac;
    WifiHelper wifi;
     if (frequency == 868)  
       {
         wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
       }
     else
       {
         std::cout<<"Wrong frequency value!"<<std::endl;
         return 0;
       }

    ostringstream oss;
    oss << "HtMcs" << 1;
    wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue (oss.str ()),
                                            "ControlMode", StringValue (oss.str ()));
      
    Ssid ssid = Ssid ("ns3-80211n");
    mac.SetType ("ns3::StaWifiMac",
                 "Ssid", SsidValue (ssid)); 
    NetDeviceContainer sensorDevice;
    sensorDevice = wifi.Install (phy, mac, SensorNodes); 
    mac.SetType ("ns3::ApWifiMac",
                 "Ssid", SsidValue (ssid)); 
    NetDeviceContainer anchorDevice;
    anchorDevice = wifi.Install (phy, mac, AnchorNodes); 

Ptr<UniformRandomVariable> randomizer1 = CreateObject<UniformRandomVariable> ();
	  randomizer1->SetAttribute ("Min", DoubleValue (0));
	  randomizer1->SetAttribute ("Max", DoubleValue (300));

    MobilityHelper mobility;
    mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
	  mobility.SetPositionAllocator ("ns3::RandomBoxPositionAllocator",
			"X", PointerValue (randomizer1),
			"Y", PointerValue (randomizer1),
			"Z", PointerValue (randomizer1));
	  mobility.Install (SensorNodes); //setting constant velocity model to sensor nodes
	  mobility.Install (AnchorNodes); //setting constant position model to anchor nodes
  

  

    //Internet Settings
    InternetStackHelper stack;
    stack.Install (AnchorNodes); //installing internet to anchor nodes
    stack.Install (SensorNodes); //installing internet to sensor nod
    Ipv4AddressHelper address;
    address.SetBase ("192.168.1.0", "255.255.255.0");
    Ipv4InterfaceContainer staNodeInterface;
    Ipv4InterfaceContainer apNodeInterface;
    staNodeInterface = address.Assign (sensorDevice); //assigning ip to sensor
    apNodeInterface = address.Assign (anchorDevice); //assigning ip to anchor


    
   
    ///Energy Resource Settings
    BasicEnergySourceHelper basicSourceHelper;
	// configure energy source
	basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (energy)); //setting initial energy;
  basicSourceHelper.Set("BasicEnergySupplyVoltageV",DoubleValue (2.5));//setting initial voltage
  basicSourceHelper.Set("BasicEnergyLowBatteryThreshold",DoubleValue (0.1));//setting initial voltage
  BasicEnergySourceHelper basicSourceHelper1;
  basicSourceHelper1.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (energy)); //setting initial energy;
  basicSourceHelper1.Set("BasicEnergySupplyVoltageV",DoubleValue (2.5));//setting initial voltage
  
	// install source
	EnergySourceContainer sources = basicSourceHelper1.Install (SensorNodes);
	EnergySourceContainer sources1 = basicSourceHelper.Install (AnchorNodes);   

	/* device energy model */
	WifiRadioEnergyModelHelper radioEnergyHelper;
	// configure sensor energy model
	radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.0174)); //setting power 0.23A 
  radioEnergyHelper.Set ("RxCurrentA", DoubleValue (0.0197)); 
	// install device model

	//DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (sensorDevice, sources);
  DeviceEnergyModelContainer deviceModels1 = radioEnergyHelper.Install (anchorDevice, sources1);  
  
  BasicEnergyHarvesterHelper basicHarvesterHelper;
   // configure energy harvester
   basicHarvesterHelper.Set ("PeriodicHarvestedPowerUpdateInterval", TimeValue (Seconds (harvestingUpdateInterval)));
   basicHarvesterHelper.Set ("HarvestablePower", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=0.015]"));
   // install harvester on all energy sources
   EnergyHarvesterContainer harvesters = basicHarvesterHelper.Install (sources1);
  
  

  Ptr<WifiRadioEnergyModel> model1 =m_deviceEnergyModel.Create<WifiRadioEnergyModel> (); 

          //Sensor
          model1->SetEnergySource (sources.Get(0));          			
			    sources.Get(0)->AppendDeviceEnergyModel (model1);
          DeviceEnergyModelContainer modelsensor =sources.Get(0)->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel");  

         for (int nodeId = 0; nodeId < 4; nodeId++)
          {
              model1->SetEnergySource (sources1.Get(nodeId));
              sources1.Get(nodeId)->AppendDeviceEnergyModel (model1);
              DeviceEnergyModelContainer modelanchor =sources1.Get(nodeId)->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel");
              
          } 
          for (int nodeId = 4; nodeId < anchor; nodeId++)
          {
              model1->SetEnergySource (sources1.Get(nodeId));
              sources1.Get(nodeId)->AppendDeviceEnergyModel (model1);
              DeviceEnergyModelContainer modelanchor2 =sources1.Get(nodeId)->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel");
              
          }
        for(uint32_t i = 0; i < AnchorNodes.GetN(); i++)
         {
            sa.anchornum=i;
            sa.sleepmode=false;
            anchorList.push_back(sa);
    
          }





          
         /*  //Anchor0
          model1->SetEnergySource (sources1.Get(0));
          sources1.Get(0)->AppendDeviceEnergyModel (model1);
          DeviceEnergyModelContainer models1 =sources1.Get(0)->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel"); 
          //models.Add(sources1.Get(0)->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel"));

            
          //Anchor1
          model1->SetEnergySource (sources1.Get(1));
          sources1.Get(1)->AppendDeviceEnergyModel (model1);
          DeviceEnergyModelContainer models2 =sources1.Get(1)->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel");
          //models1.Add(sources1.Get(1)->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel"));

          //Anchor2
          model1->SetEnergySource (sources1.Get(2));
          sources1.Get(2)->AppendDeviceEnergyModel (model1);
          DeviceEnergyModelContainer models3 =sources1.Get(2)->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel");  */ 
          //models.Add(sources1.Get(2)->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel"));
         
          //DeviceEnergyModelContainer models1 =sources1.Get(2)->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel");

  

  

     /* Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource> (sources.Get(0));
			basicSourcePtr->TraceConnectWithoutContext ("RemainingEnergy", MakeCallback (&RemainingEnergy)); //checks remaining energy for sensor nodes */
  
			

    Simulator::Schedule (Seconds (1), &localization, AnchorNodes,SensorNodes,sources,sources1);

       
      //localization(AnchorNodes,SensorNodes,sources,sources1);
    
  
    

	

    
    
    /* Setting applications */
    ApplicationContainer serverApp, sinkApp;
    if (udp)
      {
        //UDP flow
        UdpServerHelper myServer (9);
        serverApp = myServer.Install (SensorNodes.Get (0));
        serverApp.Start (Seconds (0.0));
        serverApp.Stop (Seconds (simulationTime + 1));
        UdpClientHelper myClient (staNodeInterface.GetAddress (0), 9);
        myClient.SetAttribute ("MaxPackets", UintegerValue (4294967295u));
        myClient.SetAttribute ("Interval", TimeValue (Time ("0.00001"))); //packets/s
        myClient.SetAttribute ("PacketSize", UintegerValue (payloadSize));
        ApplicationContainer clientApp = myClient.Install (AnchorNodes.Get (0));
        clientApp.Start (Seconds (1.0));
        clientApp.Stop (Seconds (simulationTime + 1));
      }
    else
      {
        //TCP flow
        uint16_t port = 50000;
        Address apLocalAddress (InetSocketAddress (Ipv4Address::GetAny (), port));
        PacketSinkHelper packetSinkHelper ("ns3::TcpSocketFactory", apLocalAddress);
        sinkApp = packetSinkHelper.Install (SensorNodes.Get (0));
        sinkApp.Start (Seconds (0.0));
        sinkApp.Stop (Seconds (simulationTime + 1));
        OnOffHelper onoff ("ns3::TcpSocketFactory",Ipv4Address::GetAny ());
        onoff.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
        onoff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
        onoff.SetAttribute ("PacketSize", UintegerValue (payloadSize));
        onoff.SetAttribute ("DataRate", DataRateValue (DataRate ("150Mbps"))); //bit/s1000000000
        ApplicationContainer apps;
        AddressValue remoteAddress (InetSocketAddress (staNodeInterface.GetAddress (0), port));
        onoff.SetAttribute ("Remote", remoteAddress);
        apps.Add (onoff.Install (AnchorNodes.Get (0)));
        apps.Start (Seconds (1.0));
        apps.Stop (Seconds (simulationTime + 1));
      }


      for (uint32_t nodeId = 0; nodeId < 10; nodeId++)
        {
          Ptr<MobilityModel> mob = AnchorNodes.Get(nodeId)->GetObject<MobilityModel>();
          Vector pos = mob->GetPosition (); //getting position for anchor nodes
          NS_LOG_UNCOND (" Anchor "<< nodeId<< " position: x=" << pos.x << ", y=" << pos.y ); //printing x and y position coordinates for anchor nodes
        }

        Ptr<MobilityModel> mob = SensorNodes.Get(0)->GetObject<MobilityModel>();
          Vector pos = mob->GetPosition (); //getting position for anchor nodes
          NS_LOG_UNCOND (" Sensor position: x=" << pos.x << ", y=" << pos.y ); //printing x and y position coordinates for anchor nodes
 


      Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

      Ptr<FriisPropagationLossModel> model = CreateObject<FriisPropagationLossModel> ();  
       
      
      pAnim = new AnimationInterface ("sc3.xml");   //NetAnim file creation

       for (uint32_t nodeId = 0; nodeId < sensor; nodeId++)
      {
      pAnim->UpdateNodeDescription (SensorNodes.Get (nodeId), "S"+std::to_string (nodeId)); //setting description S to sensor node
      pAnim->UpdateNodeColor (SensorNodes.Get(nodeId), 255, 0, 0);
      }
      for (uint32_t nodeId = 0; nodeId < anchor; nodeId++){
      pAnim->UpdateNodeDescription (AnchorNodes.Get (nodeId), "A"+std::to_string (nodeId)); //setting description A to anchor node
      pAnim->UpdateNodeColor (AnchorNodes.Get(nodeId), 0, 255, 0); //updating color to anchor node
      } 
      Simulator::Stop (Seconds (simulationTime + 1)); //stopping the simulation
      Simulator::Run ();
      Simulator::Destroy ();

  return 0;
    

}

