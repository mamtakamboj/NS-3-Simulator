/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */

#include <fstream>
#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <cstring>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv4-global-routing-helper.h"
//#include<map>;
using namespace std;

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("Assignment");

static bool g_verbose = true;
static std::string mac_sa;
static std::string power;
static Ptr<Node> n;
//static std::map<char,string> mymap;

//Function to calculate the distance travel by the visitor
static double Distance(double dX0, double dY0, double dX1, double dY1)
{
    return sqrt((dX1 - dX0)*(dX1 - dX0) + (dY1 - dY0)*(dY1 - dY0));
}


static void SetPosition(Ptr<Node> node, Vector position) {
	Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
	mobility->SetPosition(position);
}

static Vector GetPosition(Ptr<Node> node) {
	Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
	return mobility->GetPosition();
}

static void AdvancePosition(Ptr<Node> node) {
	Vector pos = GetPosition(node);
	pos.x += 0.5;

	SetPosition(node, pos);
	Simulator::Schedule(Seconds(1.0), &AdvancePosition, node);
}

static void PrintCurrentPosition(Ptr<Node> node)
{
	Vector pos = GetPosition(node);
	NS_LOG_UNCOND ("Current Position: x=" << pos.x << " y=" << pos.y << " MAC SA= "<<mac_sa << " Power: " << power);
	Simulator::Schedule(Seconds(1.0), &PrintCurrentPosition, node);
}

static void AssocTrace(std::string context, Mac48Address address) {

	NS_LOG_UNCOND("Associated with AP");
}


static void WifiPhyRx(std::string context, Ptr<const Packet> packet, uint16_t channelFreqMhz, uint16_t channelNumber, uint32_t rate, bool isShortPreamble, double signalDbm, double noiseDbm) {


	Ptr<Packet> p_copy = packet->Copy();

	std::ostringstream  os;
	p_copy->Print(os);
//	std::cout << os.str() ;

	std::string packetinfo = os.str();


//	NS_LOG_UNCOND(p_copy->GetSize() << " - " << packetinfo << signalDbm << " ceva");
	std::string str_mngmt_beacon="MGT_BEACON";
	std::size_t beacon_found = os.str().find(str_mngmt_beacon);

	std::ostringstream strs;
	strs << signalDbm;
	std::string signalDbm_str = strs.str();

	if (beacon_found!=std::string::npos)
	{
//		NS_LOG_UNCOND("Beacon Received");
		std::string str_sa="SA=";
		std::size_t sa_found = os.str().find(str_sa);

		std::string str_sa_msc="ff:ff:ff:ff:ff:ff";
		str_sa_msc = os.str().substr(sa_found+3,17);

		mac_sa=str_sa_msc;
		power=signalDbm_str;
		
		//mymap['x'] = mac_sa;
		//mymap['y'] = power;
		//g_mapBeacons[mac_sa,power];
		//for (std::map<char,string>::iterator it=mymap.begin(); it!=mymap.end(); ++it)
    		 //NS_LOG_UNCOND(it->first << " => " << it->second << '\n');
		//return 0;

		//New code for task1 Starts
		std::ofstream myfile;
		myfile.open ("distance.dat", ios::out | ios::app);
		myfile << Distance(GetPosition(n).x, GetPosition(n).y, 0, 0) << "    " << signalDbm_str << '\n';
		//New code for task1 End

		NS_LOG_UNCOND("x:" << GetPosition(n).x << ":y:" <<GetPosition(n).y << ":MAC:" << str_sa_msc << ":Power[dBm]:" << signalDbm_str);
		NS_LOG_UNCOND("Distance is" << Distance(GetPosition(n).x, GetPosition(n).y, 0, 0));
		

	}
}

int main(int argc, char *argv[]) {
	CommandLine cmd;
	cmd.AddValue("verbose", "Print trace information if true", g_verbose);
	cmd.Parse(argc, argv);

	remove("ass4_tpVsTime.txt");
	remove("ass4_tpVsDistanceToAp.txt");
	remove("ass4_modulation.txt");
	int standard = 2; // 0 = a, 1 = b, 2 = g

	double simulation_time = 370.0;	//g

	// enable rts cts all the time.
	Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold",
			StringValue("0"));
	// disable fragmentation
	Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold",
			StringValue("2200"));

	NS_LOG_INFO ("Create nodes.");
	NodeContainer ap; //all AP nodes

	NodeContainer stas; //all station nodes
	stas.Create(1);
	n=stas.Get(0);

	//ap.Create(3);
	ap.Create(1);


	NS_LOG_INFO ("Set Standard.");
	WifiHelper wifi = WifiHelper::Default();

	if (standard == 0) {
		wifi.SetStandard(WIFI_PHY_STANDARD_80211a);
	} else if (standard == 1) {
		wifi.SetStandard(WIFI_PHY_STANDARD_80211b);
	} else {
		wifi.SetStandard(WIFI_PHY_STANDARD_80211g);
	}

	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
	YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
	wifiPhy.SetChannel(wifiChannel.Create());

	wifiPhy.Set("TxPowerEnd",DoubleValue (-5.0206));
	wifiPhy.Set("TxPowerStart",DoubleValue (-5.0206));

	NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default();
	wifi.SetRemoteStationManager("ns3::ArfWifiManager");

	Ssid ssid = Ssid("museum");
	// setup stas.
	NetDeviceContainer staDevs, apDevs;
	wifiMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing",
			BooleanValue(false));
	staDevs = wifi.Install(wifiPhy, wifiMac, stas);
	// setup ap.
	wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid),
			"EnableBeaconJitter", BooleanValue(true),
			"BeaconInterval", TimeValue (MicroSeconds (102400)));
	apDevs = wifi.Install(wifiPhy, wifiMac, ap);

	NS_LOG_INFO ("Set Positions:");
	MobilityHelper mobility;
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

	mobility.Install(stas);

//	mobility.Install(ap);

//	SetPosition(ap.Get(0), Vector(0.0, 0.0, 0.0));
//	SetPosition(ap.Get(1), Vector(200.0, 00.0, 0.0));
//	SetPosition(ap.Get(2), Vector(400.0, 0.0, 0.0));
        
	

	mobility.SetPositionAllocator ("ns3::GridPositionAllocator","MinX", DoubleValue(0.0),"MinY", DoubleValue (0.0),
	"DeltaX", DoubleValue (100), "DeltaY", DoubleValue (100), "GridWidth", UintegerValue (4), "LayoutType", StringValue ("RowFirst"));

	mobility.Install (ap);


	Vector pos = GetPosition(stas.Get(0));
	pos.x = pos.x + 0 ;
	SetPosition(stas.Get(0), pos);

	//Vector ap1 = Vector(1.0, 2.0, 0.0);
  	//SetPosition(ap.Get(0), ap1);
	
	//Vector ap2 = Vector(5.0, 20.0, 0.0);
  	//SetPosition(ap.Get(1), ap2);

	//Vector ap3 = Vector(-10.0, 10.0, 0.0);
  	//SetPosition(ap.Get(2), ap3);

	InternetStackHelper stack;
	stack.Install(ap);
	stack.Install(stas);

	NS_LOG_INFO ("Assign IP Addresses:");
	Ipv4AddressHelper ipv4;
	ipv4.SetBase("10.1.1.0", "255.255.255.0");
	Ipv4InterfaceContainer i = ipv4.Assign(staDevs);
	ipv4.Assign(apDevs);

	Simulator::Schedule(Seconds(1.0), &AdvancePosition, stas.Get(0));


	Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::StaWifiMac/Assoc",
				MakeCallback(&AssocTrace));

	Config::Connect("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
					MakeCallback(&WifiPhyRx));


	NS_LOG_INFO ("Run Simulation.");
	Simulator::Stop(Seconds(simulation_time));
	Simulator::Run();
	Simulator::Destroy();
	NS_LOG_INFO ("Done.");

	return 0;
}
