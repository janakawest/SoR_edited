/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 Hiroaki Nishi Laboratory, Keio University, Japan
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Author: Janaka Wijekoon <janaka@west.sd.ekio.ac.jp>, Hiroaki Nishi <west@sd.keio.ac.jp>
 */

// Network topology
//
//    SRC(0) (IoT device)
//     |<=== source network
//     A(3)-----B(4)----SRC2(1) (request node)
//      \       /       /
//       \     /       /
//       	\   /       /
//        C(5)       /
//         |        /
//         |       /
//        D(6)-----
//         |<=== target network
//        DST(2) (15.16.16.2) 
//
//
// A, B, C and D are ESLR Enabled routers.
// A and D are configured with static addresses.
// SRC and DST will exchange packets.

#include <fstream>

#include "ns3/core-module.h"
#include "ns3/eslr-module.h"

#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-routing-table-entry.h"

#include "ns3/v4ping-helper.h"
//#include "ns3/pyviz.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("IotExample");

void MakeLinkDown (Ptr<Node> nodeA, Ptr<Node> nodeB, uint32_t intA, uint32_t intB)
{
	nodeA->GetObject<Ipv4>()->SetDown (intA);
	nodeB->GetObject<Ipv4>()->SetDown (intB);
}

void MakeLinkUp (Ptr<Node> nodeA, Ptr<Node> nodeB, uint32_t intA, uint32_t intB)
{
  nodeA->GetObject<Ipv4>()->SetUp (intA);
  nodeB->GetObject<Ipv4>()->SetUp (intB);
}

void MakeInterfaceDown (Ptr<Node> nodeA, uint32_t intA)
{
	nodeA->GetObject<Ipv4>()->SetDown (intA);
}

void MakeInterfaceUp (Ptr<Node> nodeA, uint32_t intA)
{
  nodeA->GetObject<Ipv4>()->SetUp (intA);
}


int 
main (int argc, char *argv[])
{
  bool verbose = true;
  bool MTable = false;
  bool BTable = false;
  bool NTable = false;

  CommandLine cmd;

	//{ PyViz v; }

  cmd.AddValue ("verbose", "Tell application to log if true", verbose);
  cmd.AddValue ("NTable", "Print the Neighbor Table", NTable);
  cmd.AddValue ("MTable", "Print the Main Routing Table", MTable);
  cmd.AddValue ("BTable", "Print the Backup Routing Table", BTable);

  cmd.Parse (argc,argv);

 	NS_LOG_INFO ("Create nodes.");
  
  Ptr<Node> src = CreateObject<Node> ();
  Names::Add ("SrcNode", src);
	src->SetNodeType (Node::CLIENT_NODE);
	
  Ptr<Node> src2 = CreateObject<Node> ();
  Names::Add ("SrcNode2", src2);
	src2->SetNodeType (Node::CLIENT_NODE);	
    
  Ptr<Node> dst = CreateObject<Node> ();
  Names::Add ("DstNode", dst);
	dst->SetNodeType (Node::SERVER_NODE);
  
  Ptr<Node> a = CreateObject<Node> ();
  Names::Add ("RouterA", a);
  a->SetNodeType (Node::SoR);
  a->SetAttribute ("CPUType", EnumValue (Node::XEON));
  a->SetAttribute ("MemoryType", EnumValue (Node::RAM));  
  a->SetAttribute ("MemorySize", UintegerValue (20));
  
  Ptr<Node> b = CreateObject<Node> ();
  Names::Add ("RouterB", b);
  b->SetNodeType (Node::SoR);  
  b->SetAttribute ("CPUType", EnumValue (Node::XEON));
  b->SetAttribute ("MemoryType", EnumValue (Node::RAM));  
  b->SetAttribute ("MemorySize", UintegerValue (50));
  
  Ptr<Node> c = CreateObject<Node> ();
  Names::Add ("RouterC", c);
  //c->SetNodeType (Node::SoR);  
  c->SetAttribute ("CPUType", EnumValue (Node::XEON));
  c->SetAttribute ("MemoryType", EnumValue (Node::RAM));  
  c->SetAttribute ("MemorySize", UintegerValue (5));
  
  Ptr<Node> d = CreateObject<Node> ();
  Names::Add ("RouterD", d);
  d->SetNodeType (Node::SoR); 
  d->SetAttribute ("CPUType", EnumValue (Node::XEON));
  d->SetAttribute ("MemoryType", EnumValue (Node::RAM));  
  d->SetAttribute ("MemorySize", UintegerValue (30));   
  
  
  NodeContainer net1 (src, a);
  NodeContainer net2 (a, b);
  NodeContainer net3 (a, c);
  NodeContainer net4 (b, c);
  NodeContainer net5 (c, d);
  NodeContainer net6 (b, d);
  NodeContainer net7 (d, dst);
  NodeContainer net8 (src2, b);
  NodeContainer routers (a, b, c, d);
  NodeContainer nodes (src, src2, dst);

	NS_LOG_INFO ("Create channels.");
	PointToPointHelper p2p;
	p2p.SetDeviceAttribute ("DataRate", StringValue ("100Mbps"));
  //p2p.SetChannelAttribute ("DataRate", DataRateValue (5000000));
  p2p.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (2)));
  NetDeviceContainer ndc1 = p2p.Install (net1);
  NetDeviceContainer ndc2 = p2p.Install (net2);
  NetDeviceContainer ndc3 = p2p.Install (net3);
  NetDeviceContainer ndc4 = p2p.Install (net4);
  NetDeviceContainer ndc5 = p2p.Install (net5);
  NetDeviceContainer ndc6 = p2p.Install (net6);
  NetDeviceContainer ndc7 = p2p.Install (net7);
  NetDeviceContainer ndc8 = p2p.Install (net8);

	NS_LOG_INFO ("Create IPv4 and routing");
	EslrHelper eslrRouting;

  // Rule of thumb:
  // Interfaces are added sequentially, starting from 0
  // However, interface 0 is always the loopback...
  eslrRouting.ExcludeInterface (a, 1);
  eslrRouting.ExcludeInterface (d, 3);
  eslrRouting.ExcludeInterface (b, 4);

  if (MTable)
    eslrRouting.Set ("PrintingMethod", EnumValue(eslr::MAIN_R_TABLE));
  else if (BTable)
  	eslrRouting.Set ("PrintingMethod", EnumValue(eslr::BACKUP_R_TABLE));
  else if (NTable) 
  	eslrRouting.Set ("PrintingMethod", EnumValue(eslr::N_TABLE));

  Ipv4ListRoutingHelper list;
  list.Add (eslrRouting, 0);

	InternetStackHelper internet;
 	internet.SetRoutingHelper (list);
	internet.Install (routers);

	InternetStackHelper internetNodes;
	internetNodes.Install (nodes);

	NS_LOG_INFO ("Assign IPv4 Addresses.");
	Ipv4AddressHelper ipv4;
	ipv4.SetBase ("192.168.16.0","255.255.255.0");
	Ipv4InterfaceContainer iic1 = ipv4.Assign (ndc1);

  ipv4.SetBase ("172.16.10.0","255.255.255.0");
  Ipv4InterfaceContainer iic2 = ipv4.Assign (ndc2);

  ipv4.SetBase ("203.15.19.0","255.255.255.0");
  Ipv4InterfaceContainer iic3 = ipv4.Assign (ndc3);

  ipv4.SetBase ("201.13.15.0","255.255.255.0");
  Ipv4InterfaceContainer iic4 = ipv4.Assign (ndc4);

  ipv4.SetBase ("10.10.10.0","255.255.255.0");
  Ipv4InterfaceContainer iic5 = ipv4.Assign (ndc5);

  ipv4.SetBase ("11.118.126.0","255.255.255.0");
  Ipv4InterfaceContainer iic6 = ipv4.Assign (ndc6);

  ipv4.SetBase ("15.16.16.0","255.255.255.0");
  Ipv4InterfaceContainer iic7 = ipv4.Assign (ndc7);
  
  ipv4.SetBase ("192.168.55.0","255.255.255.0");
  Ipv4InterfaceContainer iic8 = ipv4.Assign (ndc8);  
  
  NS_LOG_INFO ("Setting the default gateways of the Source and Destination.");
  Ipv4StaticRoutingHelper statRouting;
  
	// setting up the 'A' as the default gateway of the 'Src' 
	Ptr<Ipv4StaticRouting> statSrc = statRouting.GetStaticRouting (src->GetObject<Ipv4> ());
	statSrc->SetDefaultRoute (a->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), 1, 1);
	
	// setting up the 'B' as the default gateway of the 'Src' 
	Ptr<Ipv4StaticRouting> statSrc2 = statRouting.GetStaticRouting (src2->GetObject<Ipv4> ());
	statSrc2->SetDefaultRoute (b->GetObject<Ipv4> ()->GetAddress (4, 0).GetLocal (), 1, 1);	
	
  // setting up the 'D' as the default gateway of the 'Dst'
	Ptr<Ipv4StaticRouting> statDst = statRouting.GetStaticRouting (dst->GetObject<Ipv4> ());
	statDst->SetDefaultRoute (d->GetObject<Ipv4> ()->GetAddress (3, 0).GetLocal (), 1, 1);
	
	

      EslrHelper routingHelper;
      Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> (&std::cout);

      if (MTable || NTable || BTable)
        routingHelper.PrintRoutingTableEvery (Seconds (30), b, routingStream);
        
  
  NS_LOG_INFO ("Setting up UDP echo server and client.");
	//create server  	
	uint16_t port = 9; // well-known echo port number
	UdpEchoServerHelper server (port);
	ApplicationContainer apps = server.Install (dst); 

	apps.Start (Seconds (100.0));
	apps.Stop (Seconds (205.0));
	
	//create client	
	Ptr<Ipv4> ipv4dst = dst->GetObject<Ipv4> ();
	
	UdpEchoClientHelper client (ipv4dst->GetAddress (1, 0).GetLocal (), port); 
  client.SetAttribute ("MaxPackets", UintegerValue (100));
  client.SetAttribute ("Id", UintegerValue (1));
  client.SetAttribute ("ClientType", EnumValue(UdpEchoClient::IOT_DEV));  
  //client.SetAttribute ("Interval", TimeValue (Seconds (1.)));
  client.SetAttribute ("PacketSize", UintegerValue (1024));
  
  ApplicationContainer appc = client.Install (src);
	client.SetFill (appc.Get (0), "abcdefghijklmnopqrstuvwxyz1");
  
  appc.Start (Seconds (101.0));
	appc.Stop (Seconds (160.0));

	UdpEchoClientHelper client2 (ipv4dst->GetAddress (1, 0).GetLocal (), port); 
  client2.SetAttribute ("MaxPackets", UintegerValue (100));
  client2.SetAttribute ("Id", UintegerValue (2));
  client2.SetAttribute ("ClientType", EnumValue(UdpEchoClient::IOT_CLIENT));    
  //client.SetAttribute ("Interval", TimeValue (Seconds (1.)));
  client2.SetAttribute ("PacketSize", UintegerValue (1024));
  
  ApplicationContainer appc2 = client2.Install (src2);
	client2.SetFill (appc2.Get (0), "abcdefghijklmnopqrstuvwxyz1");
  
  appc2.Start (Seconds (170.0));
	appc2.Stop (Seconds (200.0));
/*			Ptr<V4Ping> app = CreateObject<V4Ping> (); 

			Ptr<Ipv4> ipv4Server = dst->GetObject<Ipv4> (); 
			app->SetAttribute ("Remote", Ipv4AddressValue (ipv4Server->GetAddress (1, 0).GetLocal ()));
			app->SetAttribute ("Verbose", BooleanValue (true));
			src->AddApplication (app);
			app->SetStartTime (Seconds (102.0));
			app->SetStopTime (Seconds (420.0));
*/


//      Simulator::Schedule (Seconds (400), &MakeLinkDown, b, d, 3, 2); 
//      Simulator::Schedule (Seconds (550), &MakeLinkUp, b, d, 3, 2);
      
//      Simulator::Schedule (Seconds (410), &MakeLinkDown, b, c, 2, 2); 
//      Simulator::Schedule (Seconds (550), &MakeLinkUp, b, c, 2, 2); 
//      
//      Simulator::Schedule (Seconds (420), &MakeLinkDown, b, a, 1, 2); 
//      Simulator::Schedule (Seconds (550), &MakeLinkUp, b, a, 1, 2);  

//      Simulator::Schedule (Seconds (40), &MakeInterfaceDown, b, 3); 
//      Simulator::Schedule (Seconds (185), &MakeInterfaceUp, b, 3); 
      

	Simulator::Stop (Seconds (210));
  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}


