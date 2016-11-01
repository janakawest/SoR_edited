/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013 West Laboratory, Keio University
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Janaka Wijekoon <janaka@west.sd.keio.ac.jp>
 */
/*
 * This module is added to the network because the dependability issues.
 * if this module is added as a separate module in the network layer
 * dependency cycles might occur between the node and the internet modules.
 * Therefore, SoR module has been implemented under the network module to 
 * keep and maintain the simplicity.
 *
 * However the SoR module does not parented by the Node module, because 
 * the SoR module is just an stand alone module.
 */
 
#include "sor.h"

#include "ns3/ipv4.h"

#include "ns3/llc-snap-header.h"
#include "ns3/ipv4-header.h"
#include "ns3/udp-header.h"
#include "ns3/tcp-header.h"
#include "ns3/ptr.h"
#include "ns3/net-device.h"
#include "ns3/core-module.h"
#include "ns3/iot-header.h" //This is the IoT genaral Header

#include "node.h"
#include "channel.h"
#include "net-device.h"

#include "ns3/uinteger.h"
#include "ns3/log.h"
#include "ns3/assert.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4.h"

NS_LOG_COMPONENT_DEFINE ("SoR");
#define COMM_PORT 55555
namespace ns3
{
	TypeId 
	SoR::GetTypeId (void)
	{
    static TypeId tid = TypeId ("ns3::SoR")
      .SetParent<Object> ()
      .SetGroupName("Network")
      .AddConstructor<SoR> ()
//      .AddAttribute ("DeviceList", "The list of devices associated to this Node.",
//                     ObjectVectorValue (),
//                     MakeObjectVectorAccessor (&Node::m_devices),
//                     MakeObjectVectorChecker<NetDevice> ())
//      .AddAttribute ("ApplicationList", "The list of applications associated to this Node.",
//                     ObjectVectorValue (),
//                     MakeObjectVectorAccessor (&Node::m_applications),
//                     MakeObjectVectorChecker<Application> ())
//      .AddAttribute ("Id", "The id (unique integer) of this Node.",
//                     TypeId::ATTR_GET, // allow only getting it.
//                     UintegerValue (0),
//                     MakeUintegerAccessor (&Node::m_id),
//                     MakeUintegerChecker<uint32_t> ())
//      .AddAttribute ("SystemId", "The systemId of this node: a unique integer used for parallel simulations.",
//                     TypeId::ATTR_GET || TypeId::ATTR_SET,
//                     UintegerValue (0),
//                     MakeUintegerAccessor (&Node::m_sid),
//                     MakeUintegerChecker<uint32_t> ())
//		  .AddAttribute ("NodeType", "The node type; a router, an SoR, or an application nodes.",
//									   EnumValue (ROUTER),
//									   MakeEnumAccessor (&Node::m_nodeType),
//									   MakeEnumChecker (ROUTER, "A ROUTER",
//										   								SERVER_NODE, "A SERVER NODE",
//																		  CLIENT_NODE, "A CLIENT NODE",
//																		  SoR, "An SoR Node"))                   
    ;
    return tid;	
	}

	SoR::SoR (): m_sorEnabled (0),
	             m_node (0),
	             m_sorId (0),
	             m_ipv4 (0)
	{
    NS_LOG_FUNCTION (this);	
    /* cstrctr*/
	}
	
	SoR::~SoR ()
	{
    NS_LOG_FUNCTION (this);	
	  /*dstrctr*/
	}	
	
	void 
	SoR::InitializeSoR (Ptr<Node> node)
	{
    NS_LOG_FUNCTION (this);	
  
	  m_sorEnabled = true;
	  m_node = node;
	  m_sorId = node->GetId ();
	  m_ipv4 = node->GetObject<Ipv4> ();
    m_maxMemory = node->GetMemorySize ();	
    m_localAddress = m_ipv4->GetAddress (1, 0).GetLocal (); // Get the IP address of the first interface as the SoRs IP address
    
	  InitializeCommunicationSocket (COMM_PORT); // An arbitrary Socket number
	  
    m_rng = CreateObject<UniformRandomVariable> ();	  
	  m_rng->SetStream (1);
	  
		CollectNeighborInformation ();
	}
	
	void 
	SoR::DoDispose (void)
	{
    m_commSocket = 0;
		// Flush the on-memory-database
		m_iotCacheTable.clear ();
	}
	
	void 
	SoR::PassPacketToSoR (Ptr<Packet> packet)
	{
    NS_LOG_FUNCTION (this);	
    
    // Note: We assumed an un limited packet buffer
    packetBuffer.push (packet);   	
	}
	
  void 
  SoR::InitializeCommunicationSocket (uint16_t port)
  {
    if (m_commSocket == 0)
    {
      TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
      m_commSocket = Socket::CreateSocket (m_node, tid);
      
      NS_ASSERT (m_commSocket != 0);

      //Create the communication socket using the IP address of the interface #1 
			m_commSocket->Bind (InetSocketAddress(m_localAddress, port));  
			
			// Only ingress routers (Edge routers) are configured to maintain a comunication
			// channel with Master routers
      if (m_node->GetNodePlacement () == Node::INGRESS_ROUTER)      
			{
        m_commSocket->SetRecvCallback (MakeCallback (&SoR::HandleRecommendationPackets, this));
      }        
    }
  }	
  
  void
  SoR::HandleRecommendationPackets (Ptr<Socket> socket)
  {
    NS_LOG_FUNCTION (this << socket);
    
    Ptr<Packet> receivedPacket;
    Address from;

    IoTHeader iotHeader;		
		
    while ((receivedPacket = socket->RecvFrom (from)))
	  {
	    // add it the the cache
	    // send a recommend packet to the client and ask him to re-initiate the connection
	    // d 
	    receivedPacket->RemoveHeader (iotHeader); 
	    if (iotHeader.GetType ()  == IoTHeader::IOT_RECOMMENDATION_PACKET)
      {
	      NS_LOG_DEBUG("@ " <<  Simulator::Now ().GetSeconds () 
											<< " " << m_sorId << " received a Recommendation"); 
	    }
	    else
	    {
	      return;
	    }
	  }
  }  
  
  bool 
  SoR::AnalyzePacetInformationForIoT (Ptr<Packet> packet, Ptr<NetDevice> interface)
  {
    bool retVal = false;
    
    // remove headers
    Ipv4Header ipv4hdr; //!< ip header information
    Ipv4Address sourceAddress; //!< the source address 
    Ipv4Address destAddress; //!< the destination address
    uint8_t protocol; //!< the protocol number 	// this is to identify the packet type
                                                // 17 UDP
                                                // 6 TCP
                                                // 1 ICMP
    UdpHeader udpHeader; //!< UDP header
    TcpHeader tcpHeader; //!< TCP header
    IoTHeader iotHeader; //!< IoT header
    
    uint16_t sourcePort; //!< Source port 
    uint16_t destPort;  //!< Destination port
    
    packet->RemoveHeader (ipv4hdr);
    sourceAddress = ipv4hdr.GetSource (); // source address
    destAddress = ipv4hdr.GetDestination (); // destination address
    protocol = ipv4hdr.GetProtocol (); // protocol number 

// After this point, the coding is highly depending on the application
// Basically, this is the point that Recommender module comes in to the big picture.
// TODO write a separate function to do the recommendation part.		
		std::string tempData = "";
    NS_ASSERT_MSG (m_ipv4 != 0, "SoR_ERROR: Ip information of " 
				<< interface << " is wrong. Hint: please check the Internet stack");

    // Get the IP layer information of the interface 
    Ipv4InterfaceAddress iface = m_ipv4->GetAddress (m_ipv4->GetInterfaceForDevice(interface), 0);
    
		// Ignore Route update messages to improve the efficiency
		// In case of the simulation uses ListRoutingProtocol
		// It is the responsibility of the user to edit following line according to the
		// routing protocol used in the simulation
		// As this tested against the folowing protocols
    // Broadcast route pacekts (ESLR, SLR, DVRP uses broadcast updates)
    if ((iface.GetBroadcast () == destAddress) && (protocol == 17)) 
    {
      // if a route packet add the header and return true
      packet->AddHeader (ipv4hdr);
      return (retVal = true);
    }
    else
    {
      packet->RemoveHeader (udpHeader);
      destPort = udpHeader.GetDestinationPort();
	    sourcePort = udpHeader.GetSourcePort();	

    	packet->RemoveHeader (iotHeader);
      if (iotHeader.GetType ()  == IoTHeader::IOT_REQUEST_PACKET)
      {
        bool dataIsFound = false;
        dataIsFound = FindIoTDataInCache (iotHeader.GetData ());
        if (dataIsFound) // The packet is not yet processed 
        {
          if (iotHeader.GetProcessedSoRId () == 0) // there might be several places that the data is stored.
          {
            iotHeader.SetProcessedSoRId (m_sorId); // saying that the data is stored in this SoR
            iotHeader.SetHopCount ();          
            iotHeader.SetLastSoRId (m_sorId);
            iotHeader.SetType (IoTHeader::IOT_PROCESSED_PACKET);
            
            ReplyIoTRequest (sourceAddress, destAddress, sourcePort, destPort);
          }
          else // if the data is already sent by another router, just pass it to the server for statistics
          {
            iotHeader.SetHopCount ();                    
          }
          packet->AddHeader (iotHeader);
          packet->AddHeader (udpHeader);
          packet->AddHeader (ipv4hdr);
          return (retVal = true);        
        } 
        else
        {
          if (iotHeader.GetIngressSoRAddress () == "0.0.0.0" && 
							m_node->GetNodePlacement () == Node::INGRESS_ROUTER)
          {
            iotHeader.SetHopCount ();          
            iotHeader.SetLastSoRId (m_sorId);
            iotHeader.SetIngressSoRAddress (m_localAddress); 
            iotHeader.SetIngressSoRId (m_sorId);
          }
          else if (iotHeader.GetIngressSoRAddress () != "0.0.0.0" && 
									 m_node->GetNodePlacement () == Node::MASTER_ROUTER)
          {
						
            // Recommender module recommends information 
            iotHeader.SetProcessedSoRId (m_sorId); // saying that the data is stored in this SoR
            iotHeader.SetHopCount ();          
            iotHeader.SetLastSoRId (m_sorId);
    
            // Create a recommendation packet to the ingress Router
            Ptr<Packet> p = Create<Packet> ();
            IoTHeader iotReplyHeader; //!< IoT header
            
            iotReplyHeader.SetType (IoTHeader::IOT_RECOMMENDATION_PACKET);
            iotReplyHeader.SetLastSoRId (m_sorId);
            iotReplyHeader.SetProcessedSoRId (m_sorId); // saying that the data is stored in this SoR
		        iotReplyHeader.SetTimestamp (Simulator::Now ().GetSeconds ());
            p->AddHeader (iotReplyHeader);
            
          	NS_LOG_DEBUG(Simulator::Now ().GetSeconds () 
											<< " "<< m_sorId << " Send a Recommendation to " 
											<< iotHeader.GetIngressSoRAddress ());            
            
						m_commSocket->SendTo (p, 0, InetSocketAddress (iotHeader.GetIngressSoRAddress (), COMM_PORT/*a temporary socket number*/));               
          }
          packet->AddHeader (iotHeader);
          packet->AddHeader (udpHeader);
          packet->AddHeader (ipv4hdr);
          return (retVal = true);          
        }
      }
      else if (iotHeader.GetType ()  == IoTHeader::IOT_DATA_PACKET)
      { 
        // Get the routing information       
        Socket::SocketErrno sockerr;
        Ptr<Ipv4Route> rt = m_ipv4->GetRoutingProtocol ()->RouteOutput (packet, ipv4hdr, 0, sockerr);
        if (!rt)
        {
          NS_LOG_INFO (m_sorId << " no route found for " << destAddress) ;
          
          iotHeader.SetHopCount ();
          iotHeader.SetLastSoRId (m_sorId);  
          packet->AddHeader (iotHeader);
          packet->AddHeader (udpHeader);
          packet->AddHeader (ipv4hdr);
          return (retVal = true);
        }
        
        Ptr<NetDevice> forwardingDevice = rt->GetOutputDevice ();
        NS_ASSERT_MSG (forwardingDevice, "SoR_ERROR: No NetDevice Found. Please check the connectivity.");
                
        if (( (GetNextHopCapability (forwardingDevice) == -1) && /* if my neighbor is out of memory*/
              (GetAvailableMemory () > 0) &&
              (iotHeader.GetStoreStatus () == IoTHeader::NOT_STORED)) ||
            ( (GetNextHopType (forwardingDevice) != (uint8_t) Node::SoR) && /*if my neighbor is not an SoR and I have stoage*/
              (GetAvailableMemory () > 0) &&
              (iotHeader.GetStoreStatus () == IoTHeader::NOT_STORED)) ||
            ( (GetMyCapability () != -1) && /* if my cost is better than my neighbors cost NOTE: packet's next hop towards destination'*/
              (GetMyCapability () >= GetNextHopCapability (forwardingDevice)) && 
              (iotHeader.GetStoreStatus () == IoTHeader::NOT_STORED)))
        {
           	NS_LOG_DEBUG(m_sorId <<" "
						<< iotHeader.GetAreaId () 
						<<" " << iotHeader.GetDeviceId ());

          IoTRecord *newRecord  = new IoTRecord (iotHeader.GetAreaId (), 
																									iotHeader.GetDeviceId (), 
																									iotHeader.GetHopCount (), 
																									iotHeader.GetData ()); //FIXME data has to be the remaining part
					m_iotCacheTable.push_front (std::make_pair (newRecord, EventId ())); //TODO use the event
          
          iotHeader.SetHopCount ();          
          iotHeader.SetStoreStatus(IoTHeader::STORED);
          iotHeader.SetLastSoRId (m_sorId);
          iotHeader.SetProcessedSoRId (m_sorId); // saying that the data is stored in this SoR
        }
        else if (iotHeader.GetStoreStatus () == IoTHeader::STORED && 
								 m_node->GetNodePlacement () == Node::MASTER_ROUTER)
        {
          	NS_LOG_DEBUG("Master Router " <<  
            m_sorId << " " << m_localAddress << " received a Data, ID: " <<  
            iotHeader.GetData () << 
            " genarated @" << 
            iotHeader.GetTimestamp () << 
            "s and the data are stored @ " <<  
            iotHeader.GetProcessedSoRId  ());
        }
        else  
        {
          iotHeader.SetHopCount ();        
        }
      }
      else if (iotHeader.GetType ()  == IoTHeader::IOT_REPLY_PACKET)
      {
        iotHeader.SetHopCount ();
      }
      else if (iotHeader.GetType ()  == IoTHeader::IOT_PROCESSED_PACKET)
      {    
        iotHeader.SetHopCount ();
      }     
      else
      {
        // FIXME
      }         

      packet->AddHeader (iotHeader);
      packet->AddHeader (udpHeader);
      packet->AddHeader (ipv4hdr);
      
      return (retVal = true);
    }
    return retVal;
  }
  
  void 
  SoR::ReplyIoTRequest (Ipv4Address packetSource, Ipv4Address packetDestination, uint16_t packetSourcePort, uint16_t packetDestinationPort)
  {
    Ptr<Packet> p = Create<Packet> ();
    IoTHeader iotHeader; //!< IoT header
    
    iotHeader.SetType (IoTHeader::IOT_REPLY_PACKET);
    iotHeader.SetLastSoRId (m_sorId);
    iotHeader.SetProcessedSoRId (m_sorId); // saying that the data is stored in this SoR
		iotHeader.SetTimestamp (Simulator::Now ().GetSeconds ());
    p->AddHeader (iotHeader);
    
    m_commSocket->SendTo (p, 0, InetSocketAddress (packetSource, packetSourcePort));    
  }

	void
	SoR::CollectNeighborInformation (void)
	{
		Ptr<NetDevice> neighborNetdevice;
		Ptr<NetDevice> myNetdevice;		
		Ptr<Channel> attachedChannel;
		Ptr<Node> neighborNode;
		
	  StringValue getBW;
    std::string temp, bandwidth;
    double linkBandwidth;

		for (uint16_t devNo = 0; devNo <= (m_node->GetNDevices () - 1); devNo++)
		{
			attachedChannel = m_node->GetDevice (devNo)->GetChannel ();
			if (!attachedChannel)
				continue;

			uint16_t nDevices = attachedChannel->GetNDevices ();
			NS_ASSERT_MSG (nDevices, "SoR_ERROR: No NetDevice Found. Please check the connectivity.");

			neighborNetdevice = attachedChannel->GetDevice (nDevices - 2);
			if(neighborNetdevice == m_node->GetDevice (devNo))
				neighborNetdevice = attachedChannel->GetDevice (nDevices - 1);
			
			neighborNode = neighborNetdevice->GetNode ();

      NeighborDetails neighborDetails;
      neighborDetails.neighborID = neighborNode->GetId ();
      neighborDetails.neighborDevice = neighborNetdevice;
		  neighborDetails.processorType = neighborNode->GetProcCostFactor ();
		  neighborDetails.memoryType = neighborNode->GetMemCostFactor ();
		  neighborDetails.localInterfaceNo = m_node->GetDevice (devNo)->GetIfIndex ();
		  neighborDetails.localInterface = m_node->GetDevice (devNo);
		  neighborDetails.availableMemory = neighborNode->GetAvailableSpace ();
		  neighborDetails.nodeType = (uint8_t)neighborNode->GetNodeType ();
				 	// Calculate and assign link cost
					// Get the capacity of the link
      m_node->GetDevice (devNo)->GetAttribute("DataRate", getBW);
      bandwidth = getBW.Get ().c_str ();

      temp = bandwidth.substr (0, (bandwidth.size () - 3));
      linkBandwidth = (double) (atof (temp.c_str ())) / 1000000; // in Mbps		  
		  
		  neighborDetails.linkCost = (double) 100.0/ linkBandwidth; // method used in OSPF
		  m_neighborInfomrationTable.push_front (neighborDetails);
		} 
		// Schedule next update
    Time delay = Seconds (m_rng->GetValue (10, 15));
    m_neighborUpdateEvent = Simulator::Schedule (delay, &SoR::UpdateNeighborInformation, this);		
	}
	
	void 
	SoR::UpdateNeighborInformation (void)
	{
	  NeighborTableI iter;	
	  for (iter = m_neighborInfomrationTable.begin (); iter != m_neighborInfomrationTable.end (); iter++)
	  {
	    iter->availableMemory = iter->neighborDevice->GetNode ()-> GetAvailableSpace ();
	  }	  
    Time delay = Seconds (m_rng->GetValue (10, 15));
    m_neighborUpdateEvent = Simulator::Schedule (delay, &SoR::UpdateNeighborInformation, this);	  
	}
	
	double 
	SoR::GetNextHopCapability (Ptr<NetDevice> device)
	{
	  //uint8_t procRank = 0, memRank = 0;
	  double cost = 0.0;
	  NeighborTableI iter;
	  
	  for (iter = m_neighborInfomrationTable.begin (); iter != m_neighborInfomrationTable.end (); iter++)
	  {
	    if (iter->localInterface == device)
	    {
	      if (iter->availableMemory > 0)
    	  {
    	    cost = (double)(iter->availableMemory / AVG_DATA_SIZE)/((uint8_t)iter->processorType + (uint8_t)iter->memoryType) - iter->linkCost;
    	  }
    	  else if (iter->availableMemory <= 0)
    	  {
    	    cost = -1;
    	  }
	      break;
	    }
	  }
	  return cost;
	}
	
	uint8_t 
	SoR::GetNextHopType (Ptr<NetDevice> device)
	{
	  uint8_t type = 0;
	  NeighborTableI iter;
	  
	  for (iter = m_neighborInfomrationTable.begin (); iter != m_neighborInfomrationTable.end (); iter++)
	  {
	    if (iter->localInterface == device)
	    {
	      type = iter->nodeType;
	      break;
	    }
	  }
	  return type;	
	}
	
	double 
	SoR::GetMyCapability (void)
	{
	  double cost = 0.0;
	  if (GetAvailableMemory () <= 0)
	  {
	    cost = -1;
	  }
	  else
	  {
      cost = (double)(GetAvailableMemory ()/AVG_DATA_SIZE)/((uint8_t)m_node->GetProcCostFactor () + (uint8_t)m_node->GetMemCostFactor ()); // Consider both CPU and MEM both alpha and beta ==1
    }
	  return cost;
	}
	
	bool 
	SoR::FindIoTDataInCache (std::string/*uint32_t*/ data) //TODO std::string data to emulate SOPE messages
	{
	  bool retVal = false;
	  IoTCacherecordI iter;
	  for (iter = m_iotCacheTable.begin (); iter != m_iotCacheTable.end (); iter++)
	  {
	    if (data == iter->first->GetData ())
	    {
	      retVal = true;
	    }	    
	  }
	  return retVal;
	}
}// ns3
