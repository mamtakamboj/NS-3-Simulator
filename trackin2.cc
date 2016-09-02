/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
#include <fstream>
#include <iostream>
#include <string>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include <sstream>
#include <cstring>

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("Assignment");

static bool g_verbose = true;
static std::string mac_sa;
static std::string power;
static Ptr<Node> n;


struct coordinates {
	double x;
	double y;
};

static std::vector<double> g_distances;
static std::vector<double> g_powers;
static std::map<std::string, std::string> g_mapBeacons;
static std::map<std::string, coordinates> g_mapApLocations;
static std::map<std::string, coordinates> g_mapExhibits;

//Function to calculate the distance travel by the visitor
static double Distance(double dX0, double dY0, double dX1, double dY1)
{
    return sqrt((dX1 - dX0)*(dX1 - dX0) + (dY1 - dY0)*(dY1 - dY0));
}

static void SetPosition(Ptr<Node> node, Vector position)
{
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

std::vector<double> trilateration(double pointA[], double pointB[], double pointC[], double ad, double bd, double cd)
{
    std::vector<double> resultPose;
	double s = (pow(pointC[0], 2) - pow(pointB[0], 2) + pow(pointC[1], 2) - pow(pointB[1], 2) + pow(bd, 2) - pow(cd, 2))/2;
	double t = (pow(pointA[0], 2) - pow(pointB[0], 2) + pow(pointA[1], 2) - pow(pointB[1], 2) + pow(bd, 2) - pow(ad, 2))/2;
	double y = (t*(pointB[0] - pointC[0]) - s*(pointB[0] - pointA[0]))/((pointA[1] - pointB[1])*(pointB[0] - pointC[0]) - (pointC[1] - pointB[1])*(pointB[0] - pointA[0]));
	double x = (y*(pointA[1] - pointB[1]) - t)/(pointB[0] - pointA[0]);

    resultPose.push_back(x);
   resultPose.push_back(y);
    return resultPose;
}

static void WifiPhyRx(std::string context, Ptr<const Packet> packet, uint16_t channelFreqMhz, uint16_t channelNumber, uint32_t rate, bool isShortPreamble, double signalDbm, double noiseDbm) {
	Ptr<Packet> p_copy = packet->Copy();
	std::ostringstream  os;
	p_copy->Print(os);
	std::string packetinfo = os.str();
	std::string str_mngmt_beacon="MGT_BEACON";
	std::size_t beacon_found = os.str().find(str_mngmt_beacon);
	std::ostringstream strs;
	strs << signalDbm;
	std::string signalDbm_str = strs.str();

	if (beacon_found!=std::string::npos)
	{
		std::string str_sa="SA=";
		std::size_t sa_found = os.str().find(str_sa);
		std::string str_sa_msc="ff:ff:ff:ff:ff:ff";
		str_sa_msc = os.str().substr(sa_found+3,17);
		mac_sa=str_sa_msc;
		power=signalDbm_str;
		g_mapBeacons [mac_sa] = power;
		//NS_LOG_UNCOND(power);
	}
}

void printExhibits()
{
	double radius = 25;
	double distance;
	std::map<std::string, coordinates>::iterator it;
	for(it = g_mapExhibits.begin(); it != g_mapExhibits.end(); it++)
	{

			std::string paintingName = it->first;
			coordinates pcoordinates = it->second;
			distance = Distance(GetPosition(n).x, GetPosition(n).y, pcoordinates.x, pcoordinates.y);
			if(distance < radius)
			{
				//NS_LOG_UNCOND("Name : " << paintingName << " Coordinates : [" << pcoordinates.x << " , " << pcoordinates.y << " ] ");
				//NS_LOG_UNCOND("Distance from painting : " << distance);
				NS_LOG_UNCOND("User Can see  : " << paintingName);
			}
	}
}

void calculateTrilateration ()
{
	if (g_mapBeacons.size()>=3)
	{
		NS_LOG_UNCOND(g_mapBeacons.size() << " AP Nodes are in reach");
		int g_powers_size = g_powers.size();
		coordinates location1, location2, location3;
		double distance1, distance2, distance3;
		std::map<std::string, std::string>::iterator it=g_mapBeacons.begin();
		{
			// ********* values of first AP ***********
			std::string amac1 = it->first;
			double beaconPower1 = std::atof(it->second.c_str());
			location1 = g_mapApLocations[amac1];
			uint16_t locpower1;
			for (int j=0;j<g_powers_size;j++)
			{
				if (g_powers[j] < beaconPower1)
				{
				locpower1=j;
				break;
				}
			}

			distance1 = g_distances[locpower1];
			it++;
			//******** values of Second AP ********
			std::string amac2 = it->first;
			double beaconPower2 = std::atof(it->second.c_str());
			location2 = g_mapApLocations[amac2];
			uint16_t locpower2;
			for (int m=0;m<g_powers_size;m++)
			{
				if (g_powers[m] < beaconPower2)
				{
				locpower2=m;
				break;
				}
			}
			distance2 = g_distances[locpower2];
			it++;
			// *******values of third AP********
			std::string amac3 = it->first;
			double beaconPower3 = std::atof(it->second.c_str());
			location3 = g_mapApLocations[amac3];
			uint16_t locpower3;
			for (int h=0;h<g_powers_size;h++)
			{
				if (g_powers[h] < beaconPower3)
				{
				locpower3=h;
				break;
				}
			}
			distance3 = g_distances[locpower3];
			std::vector<double> finalPose;
			double pA[] = {location1.x,location1.y};
			double pB[] = {location2.x,location2.y};
			double pC[] = {location3.x,location3.y};
			//******* call trilateration()
		finalPose = trilateration(pA,pB,pC,distance1,distance2,distance3);

		NS_LOG_UNCOND("Ap1: [" << location1.x << " , " << location1.y << "]  distance: " << distance1 << " Grid Units away ");
		NS_LOG_UNCOND("Ap2: [" << location2.x << " , " << location2.y << "]  distance: " << distance2 << " Grid Units away");
		NS_LOG_UNCOND("Ap3: [" << location3.x << " , " << location3.y << "]  distance: " << distance3 << " Grid Units away ");
		NS_LOG_UNCOND("-------------------------------------------------------------");
		NS_LOG_UNCOND("Calculated Position : [ " << finalPose[0] << " , " << finalPose[1] << "] ");
		NS_LOG_UNCOND("Actual Position : [ " << GetPosition(n).x << ", " << GetPosition(n).y << "] ");
		NS_LOG_UNCOND("Distance from Actual position to Localized position : " << Distance(GetPosition(n).x, GetPosition(n).y, finalPose[0], finalPose[1]) << " Grid Units");
		NS_LOG_UNCOND("Total Distance Travelled : " << Distance(GetPosition(n).x, GetPosition(n).y, 0, 0) << " Grid Units");
		printExhibits();
		NS_LOG_UNCOND("=====================================================");
	}

	}
	else
	{
	//print a text saying that you do not have enough information for location detection
	NS_LOG_UNCOND("Not enough info to compute location");
	}
	Simulator::Schedule(Seconds(0.5), &calculateTrilateration);

	//clear the map of beacons, otherwise the moving node may use old information
	g_mapBeacons.clear();
}

void LoadPowerDistanceMapping (std::string filename)
{
  double distance, power;
  std::ifstream ifTraceFile;
  ifTraceFile.open (filename.c_str (), std::ifstream::in);
  g_distances.clear ();
  g_powers.clear();
  if (!ifTraceFile.good ())
    {
      NS_LOG_UNCOND("Something wrong with the file.");
    }
  while (ifTraceFile.good ())
    {
      ifTraceFile >> distance >> power;

      g_distances.push_back (distance);
      g_powers.push_back(power);

    }
  ifTraceFile.close ();
}

//Main function - Start here to configure simulation
int main(int argc, char *argv[]) {
	CommandLine cmd;
	cmd.AddValue("verbose", "Print trace information if true", g_verbose);
	cmd.Parse(argc, argv);

	remove("ass4_tpVsTime.txt");
	remove("ass4_tpVsDistanceToAp.txt");
	remove("ass4_modulation.txt");
	int standard = 2;

	double simulation_time = 370.0;

	LoadPowerDistanceMapping("./scratch/distance.dat");
	Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("0"));

	Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue("2200"));

	NS_LOG_INFO ("Create nodes.");
	NodeContainer ap;

	NodeContainer stas;
	stas.Create(1);
	n=stas.Get(0);

	ap.Create(3);

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
	NetDeviceContainer staDevs, apDevs;
	wifiMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(false));
	staDevs = wifi.Install(wifiPhy, wifiMac, stas);
	wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid), "EnableBeaconJitter", BooleanValue(true), "BeaconInterval", TimeValue (MicroSeconds (102400)));
	apDevs = wifi.Install(wifiPhy, wifiMac, ap);


// YOUR CODE IS HERE YOUR CODE IS HERE YOUR CODE IS HERE YOUR CODE IS HERE YOUR CODE IS HERE

	NS_LOG_INFO ("Set Positions:");
	MobilityHelper mobility;
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

	mobility.Install(stas);

	mobility.Install(ap);

//	SetPosition(ap.Get(0), Vector(0.0, 0.0, 0.0));
//	SetPosition(ap.Get(1), Vector(200.0, 00.0, 0.0));
//	SetPosition(ap.Get(2), Vector(400.0, 0.0, 0.0));

	mobility.SetPositionAllocator ("ns3::GridPositionAllocator","MinX", DoubleValue(0.0),"MinY", DoubleValue (0.0),
	"DeltaX", DoubleValue (100), "DeltaY", DoubleValue (100), "GridWidth", UintegerValue (4), "LayoutType", StringValue ("RowFirst"));

	//mobility.Install (ap);

	Vector pos = GetPosition(stas.Get(0));
	pos.x = pos.x + 0;
	SetPosition(stas.Get(0), pos);

	SetPosition(ap.Get(0), Vector(24.0, -10.0, 0.0));
	coordinates ap1coordinates;
	ap1coordinates.x = GetPosition(ap.Get(0)).x; //x coordinateof AP0
	ap1coordinates.y = GetPosition(ap.Get(0)).y; //  y	coordinate of AP1
	g_mapApLocations["00:00:00:00:00:02"] = ap1coordinates;

	SetPosition(ap.Get(1), Vector(5.0, 15.0, 0.0));
	coordinates ap2coordinates;
	ap2coordinates.x = GetPosition(ap.Get(1)).x; //x coordinateof AP0
	ap2coordinates.y = GetPosition(ap.Get(1)).y; //  y	coordinate of AP1
	g_mapApLocations["00:00:00:00:00:03"] = ap2coordinates;

	SetPosition(ap.Get(2), Vector(-2.0, -5.0, 0.0));
	coordinates ap3coordinates;
	ap3coordinates.x = GetPosition(ap.Get(2)).x; //x coordinateof AP0
	ap3coordinates.y = GetPosition(ap.Get(2)).y; //  y	coordinate of AP1
	g_mapApLocations["00:00:00:00:00:04"] = ap3coordinates;

	/*SetPosition(ap.Get(3), Vector(20.0, -5.0, 0.0));
	coordinates ap4coordinates;
	ap4coordinates.x = GetPosition(ap.Get(3)).x; //x coordinateof AP0
	ap4coordinates.y = GetPosition(ap.Get(3)).y; //  y	coordinate of AP1
	g_mapApLocations["00:00:00:00:00:05"] = ap4coordinates;*/

	coordinates p1;
	p1.x = 22;
	p1.y = -9;
	g_mapExhibits["Painting1"] = p1;

	coordinates p2;
	p2.x = -4;
	p2.y = -14;
	g_mapExhibits["Painting2"] = p2;

	coordinates p3;
	p3.x = -6;
	p3.y = 8;
	g_mapExhibits["Painting3"] = p3;

	coordinates p4;
	p4.x = 18;
	p4.y = 10;
	g_mapExhibits["Painting4"] = p4;

	coordinates p5;
	p5.x = 26;
	p5.y = -20;
	g_mapExhibits["Painting5"] = p5;

	coordinates p6;
	p6.x = -26;
	p6.y = -20;
	g_mapExhibits["Painting6"] = p6;

	coordinates p7;
	p7.x = 26;
	p7.y = 20;
	g_mapExhibits["Painting7"] = p7;

	InternetStackHelper stack;
	stack.Install(ap);
	stack.Install(stas);

	NS_LOG_INFO ("Assign IP Addresses:");
	Ipv4AddressHelper ipv4;
	ipv4.SetBase("10.1.1.0", "255.255.255.0");
	Ipv4InterfaceContainer i = ipv4.Assign(staDevs);
	ipv4.Assign(apDevs);

	Simulator::Schedule(Seconds(1.0), &AdvancePosition, stas.Get(0));

	calculateTrilateration();


	Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::StaWifiMac/Assoc", MakeCallback(&AssocTrace));

	Config::Connect("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx", MakeCallback(&WifiPhyRx));

	NS_LOG_INFO ("Run Simulation.");
	Simulator::Stop(Seconds(simulation_time));
	Simulator::Run();
	Simulator::Destroy();
	NS_LOG_INFO ("Done.");

	return 0;
}
