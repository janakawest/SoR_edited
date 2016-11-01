/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006 Georgia Tech Research Corporation, INRIA
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
 * Authors: George F. Riley<riley@ece.gatech.edu>
 *          Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */
 
#include <math.h>
#include <string>

#include "node.h"
#include "node-list.h"
#include "net-device.h"
#include "application.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/object-vector.h"
#include "ns3/uinteger.h"
#include "ns3/log.h"
#include "ns3/assert.h"
#include "ns3/global-value.h"
#include "ns3/boolean.h"
#include "ns3/simulator.h"
#include "ns3/core-module.h"


#include "ns3/double.h"
#include "node-packet-queue.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("Node");

NS_OBJECT_ENSURE_REGISTERED (Node);

/**
 * \brief A global switch to enable all checksums for all protocols.
 */
static GlobalValue g_checksumEnabled  = GlobalValue ("ChecksumEnabled",
                                                     "A global switch to enable all checksums for all protocols",
                                                     BooleanValue (false),
                                                     MakeBooleanChecker ());

TypeId 
Node::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::Node")
    .SetParent<Object> ()
    .SetGroupName("Network")
    .AddConstructor<Node> ()
    .AddAttribute ("DeviceList", "The list of devices associated to this Node.",
                   ObjectVectorValue (),
                   MakeObjectVectorAccessor (&Node::m_devices),
                   MakeObjectVectorChecker<NetDevice> ())
    .AddAttribute ("ApplicationList", "The list of applications associated to this Node.",
                   ObjectVectorValue (),
                   MakeObjectVectorAccessor (&Node::m_applications),
                   MakeObjectVectorChecker<Application> ())
    .AddAttribute ("Id", "The id (unique integer) of this Node.",
                   TypeId::ATTR_GET, // allow only getting it.
                   UintegerValue (0),
                   MakeUintegerAccessor (&Node::m_id),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("SystemId", "The systemId of this node: a unique integer used for parallel simulations.",
                   TypeId::ATTR_GET || TypeId::ATTR_SET,
                   UintegerValue (0),
                   MakeUintegerAccessor (&Node::m_sid),
                   MakeUintegerChecker<uint32_t> ())
		.AddAttribute ("NodeType", "The node type; a router, an SoR, or an application nodes.",
									 EnumValue (ROUTER),
									 MakeEnumAccessor (&Node::m_nodeType),
									 MakeEnumChecker (ROUTER, "A ROUTER",
										 								SERVER_NODE, "A SERVER NODE",
																		CLIENT_NODE, "A CLIENT NODE",
																		SoR, "An SoR Node")) 
		.AddAttribute ("NodePlacement", "Where the node is placed on.",
									 EnumValue (CORE_ROUTER),
									 MakeEnumAccessor (&Node::m_nodePlacement),
									 MakeEnumChecker (CORE_ROUTER, "A Core router",
										 								MASTER_ROUTER, "An Edge router for DC",
																		INGRESS_ROUTER, "An Edge foruter for clients")) 																		
		.AddAttribute ("CPUType", "Which Type of a CPU that a particular Node has",
									 EnumValue (XEON),
									 MakeEnumAccessor (&Node::m_cpuType),
									 MakeEnumChecker (XEON, "Intel Xeon E3-1290 V2 @ 3.70GHz",
										 								I7, "Intel Core i7-3770K @ 3.50GHz",
																		I5, "Intel Core i5-3570 @ 3.40GHz")) 	
		.AddAttribute ("MemoryType", "Which Type Primary memory type of a particular Node",
									 EnumValue (RAM),
									 MakeEnumAccessor (&Node::m_memType),
									 MakeEnumChecker (RAM, "A RAM",
										 								SSD, "A SSD",
																		HDD, "A HDD"))
    .AddAttribute ("MemorySize", "Available Memory Size (RAM/SSD/HDD) in MB",
                   UintegerValue (2000), /*Default we assume that each node has 2GB of memory*/
                   MakeUintegerAccessor (&Node::m_totalMemorySize),
                   MakeUintegerChecker<uint32_t> ())																		 																																				                  
  ;
  return tid;
}

Node::Node()
  : m_id (0),
    m_sid (0),
		m_initiator(0),
		m_totPacketCount (0),
 		m_Mue(0.0),
		m_Lambda(0.0),
		m_serviceRate(0.0)
{
  NS_LOG_FUNCTION (this);
  
  m_nodeType = ROUTER; // By default each node is a router
  
  Construct ();
}

Node::Node(uint32_t sid)
  : m_id (0),
    m_sid (sid),
		m_initiator(0),
		m_totPacketCount (0),
		m_Mue(0.0),
		m_Lambda(0.0),
		m_serviceRate(0.0)
{ 
  NS_LOG_FUNCTION (this << sid);
  
  m_nodeType = ROUTER; // By default each node is a router
    
  Construct ();
}

void
Node::Construct (void)
{
  NS_LOG_FUNCTION (this);
  
	m_rng = CreateObject<UniformRandomVariable> ();
	  
  m_id = NodeList::Add (this);
}

Node::~Node ()
{
  NS_LOG_FUNCTION (this);
}

uint32_t
Node::GetId (void) const
{
  NS_LOG_FUNCTION (this);
  return m_id;
}

uint32_t
Node::GetSystemId (void) const
{
  NS_LOG_FUNCTION (this);
  return m_sid;
}

uint32_t
Node::AddDevice (Ptr<NetDevice> device)
{
  NS_LOG_FUNCTION (this << device);
  uint32_t index = m_devices.size ();
  m_devices.push_back (device);
  device->SetNode (this);
  device->SetIfIndex (index);
  device->SetReceiveCallback (MakeCallback (&Node::NonPromiscReceiveFromDevice, this));
  Simulator::ScheduleWithContext (GetId (), Seconds (0.0), 
                                  &NetDevice::Initialize, device);
  NotifyDeviceAdded (device);
  
  // creating per-node packet counters
  DeviceStats stat;
  stat.RxCount = 0;
  stat.avgPacketSize = 0;
  stat.TxCount = 0;
  
  m_deviceStats.push_back (std::make_pair (device, stat));
    
  return index;
}
Ptr<NetDevice>
Node::GetDevice (uint32_t index) const
{
  NS_LOG_FUNCTION (this << index);
  NS_ASSERT_MSG (index < m_devices.size (), "Device index " << index <<
                 " is out of range (only have " << m_devices.size () << " devices).");
  return m_devices[index];
}
uint32_t 
Node::GetNDevices (void) const
{
  NS_LOG_FUNCTION (this);
  return m_devices.size ();
}

uint32_t 
Node::AddApplication (Ptr<Application> application)
{
  NS_LOG_FUNCTION (this << application);
  uint32_t index = m_applications.size ();
  m_applications.push_back (application);
  application->SetNode (this);
  Simulator::ScheduleWithContext (GetId (), Seconds (0.0), 
                                  &Application::Initialize, application);
  return index;
}
Ptr<Application> 
Node::GetApplication (uint32_t index) const
{
  NS_LOG_FUNCTION (this << index);
  NS_ASSERT_MSG (index < m_applications.size (), "Application index " << index <<
                 " is out of range (only have " << m_applications.size () << " applications).");
  return m_applications[index];
}
uint32_t 
Node::GetNApplications (void) const
{
  NS_LOG_FUNCTION (this);
  return m_applications.size ();
}

void 
Node::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  m_deviceAdditionListeners.clear ();
  m_handlers.clear ();
  for (std::vector<Ptr<NetDevice> >::iterator i = m_devices.begin ();
       i != m_devices.end (); i++)
    {
      Ptr<NetDevice> device = *i;
      device->Dispose ();
      *i = 0;
    }
  m_devices.clear ();
  for (std::vector<Ptr<Application> >::iterator i = m_applications.begin ();
       i != m_applications.end (); i++)
    {
      Ptr<Application> application = *i;
      application->Dispose ();
      *i = 0;
    }
  m_applications.clear ();
  m_deviceStats.clear (); // clear device stat map.
  if (m_nodeType == SoR)
	{  
    sorInstance.DoDispose (); // Clear the SoR related data.
  }
  
  Object::DoDispose ();
}

// \name for queue packets at the nodes
// \{  

  uint16_t 
  Node::GetBufferSize(void)
  {
  	NS_LOG_FUNCTION (this);
	
	  return m_nodePacketBuffer.GetSize();  
  }
  
  uint64_t 
  Node::GetNofPacketsOfDevice (Ptr<NetDevice> device)
  {
	  NS_LOG_FUNCTION (this << device);
	  
	  for (deviceStatI iter = m_deviceStats.begin (); iter != m_deviceStats.end (); iter++)
	  {
	    if (iter->first == device)
	    {
        if ((iter->second.RxCount - iter->second.TxCount) == 0)
	        return 1;
	      else
	        return (iter->second.RxCount - iter->second.TxCount);
	    }
	  }
	  return 0;
  }
  
	uint64_t 
	Node::GetAveragePacketSizeOfDevice (Ptr<NetDevice> device)
	{
		NS_LOG_FUNCTION (this << device);
		
	  for (deviceStatI iter = m_deviceStats.begin (); iter != m_deviceStats.end (); iter++)
	  {
	    if (iter->first == device)
	      return (iter->second.avgPacketSize);
	  }
	  return 0;
	}
	
	double 
	Node::GetAveragePacketSizeOfRouter (void)
	{
		NS_LOG_FUNCTION (this);
		double averagePacketSize = 0.0;
		uint8_t deviceCount = 0;
		
	  for (deviceStatI iter = m_deviceStats.begin (); iter != m_deviceStats.end (); iter++)
    {
      averagePacketSize = averagePacketSize + iter->second.avgPacketSize;
      deviceCount++;
    }		
		
  	return (averagePacketSize/deviceCount);
	}
	
	double 
	Node::GetRouterMue (void)
	{
		NS_LOG_FUNCTION (this);  
  	return m_Mue;
	}
	
	double
	Node::GetRouterServiceRate (void)
	{
		NS_LOG_FUNCTION (this);	
	  return m_serviceRate;
	}
	
	double 
	Node::GetRouterLambda (void)
	{
	  NS_LOG_FUNCTION (this);
	  
	  return (m_totPacketCount / Simulator::Now ().GetSeconds ());	
	}
// \}

void 
Node::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
  for (std::vector<Ptr<NetDevice> >::iterator i = m_devices.begin ();
       i != m_devices.end (); i++)
    {
      Ptr<NetDevice> device = *i;
      device->Initialize ();
    }
  for (std::vector<Ptr<Application> >::iterator i = m_applications.begin ();
       i != m_applications.end (); i++)
    {
      Ptr<Application> application = *i;
      application->Initialize ();
    }

  if (m_nodeType == SoR)
  {
    sorInstance.InitializeSoR (this);
  }
  Object::DoInitialize ();
}

void
Node::RegisterProtocolHandler (ProtocolHandler handler, 
                               uint16_t protocolType,
                               Ptr<NetDevice> device,
                               bool promiscuous)
{
  NS_LOG_FUNCTION (this << &handler << protocolType << device << promiscuous);
  struct Node::ProtocolHandlerEntry entry;
  entry.handler = handler;
  entry.protocol = protocolType;
  entry.device = device;
  entry.promiscuous = promiscuous;

  // On demand enable promiscuous mode in netdevices
  if (promiscuous)
    {
      if (device == 0)
        {
          for (std::vector<Ptr<NetDevice> >::iterator i = m_devices.begin ();
               i != m_devices.end (); i++)
            {
              Ptr<NetDevice> dev = *i;
              dev->SetPromiscReceiveCallback (MakeCallback (&Node::PromiscReceiveFromDevice, this));
            }
        }
      else
        {
          device->SetPromiscReceiveCallback (MakeCallback (&Node::PromiscReceiveFromDevice, this));
        }
    }

  m_handlers.push_back (entry);
}

void
Node::UnregisterProtocolHandler (ProtocolHandler handler)
{
  NS_LOG_FUNCTION (this << &handler);
  for (ProtocolHandlerList::iterator i = m_handlers.begin ();
       i != m_handlers.end (); i++)
    {
      if (i->handler.IsEqual (handler))
        {
          m_handlers.erase (i);
          break;
        }
    }
}

bool
Node::ChecksumEnabled (void)
{
  NS_LOG_FUNCTION_NOARGS ();
  BooleanValue val;
  g_checksumEnabled.GetValue (val);
  return val.Get ();
}

bool
Node::PromiscReceiveFromDevice (Ptr<NetDevice> device, Ptr<const Packet> packet, uint16_t protocol,
                                const Address &from, const Address &to, NetDevice::PacketType packetType)
{
  NS_LOG_FUNCTION (this << device << packet << protocol << &from << &to << packetType);

 	// NOTE:  
 	//    This has been modified to buffer packet on Callback Fire
 	//    Author: Janaka Wijekoon <janaka@west.sd.ekio.ac.jp>
 	//    Packets are not buffered in Client and Server nodes. Those are directly 
 	//    forward to the protocol handler. 
 	//    However, for Routers and SoRs, packets are passed to the packet buffer first and then send to the 
 	//    protocol handler based on the service rate of the Router. 
 	//    In the case of SoRs, the packets are initially passed to the SoR packet buffer and
 	//    return to the protocol handler method. 
 	//    literally such packets are subject to go through two packet buffers.
 	//      1.  Node packet buffer
 	//      2.  SoR packet buffer 
	if (m_nodeType == SERVER_NODE || m_nodeType == CLIENT_NODE)
	{
		return ReceiveFromDevice (device, packet, protocol, from, to, packetType, true);
	}
	else
	{
		NodeQueueEntry nodeEnqueueEntry(device,
																	packet,
																	protocol,
																	from,
																	to,
																	packetType,
																	true);
		m_totPacketCount++;

  	for (deviceStatI iter = m_deviceStats.begin (); iter != m_deviceStats.end (); iter++)
  	{
    	if (iter->first == device)
    	{
      	if (iter->second.RxCount == 0)
      	{
       		iter->second.RxCount = iter->second.RxCount + 1;
        	iter->second.avgPacketSize = packet->GetSize();
      	}
      	else
      	{
        	iter->second.RxCount = iter->second.RxCount + 1;
        	iter->second.avgPacketSize = ((iter->second.avgPacketSize * (iter->second.RxCount - 1)) + packet->GetSize()) / (iter->second.RxCount) ;        
      	}
    	}
  	}	

		NS_LOG_DEBUG ("Move the packet " << packet << " to the buffer.");											
		m_nodePacketBuffer.EnQueue(nodeEnqueueEntry);

		if (!m_initiator)
  	{
			// Initiate the packet transmitor for the first time.
    	NS_LOG_DEBUG ("Initiate the packet transmitter.");
    	ScheduleTransmit(device);
    	m_initiator = 1; // Packet transmitor is initialized
  	}
	
		//Note: As every packet is buffered, the return is always true
  	return true;
	}
}

bool
Node::NonPromiscReceiveFromDevice (Ptr<NetDevice> device, Ptr<const Packet> packet, uint16_t protocol,
                                   const Address &from)
{
  NS_LOG_FUNCTION (this << device << packet << protocol << &from);
  
  if (m_nodeType == SERVER_NODE || m_nodeType == CLIENT_NODE)
	{
		return ReceiveFromDevice (device, packet, protocol, from, device->GetAddress (), NetDevice::PacketType (0), false);
	}
	else
	{
		NodeQueueEntry nodeEnqueueEntry(device,
																	packet,
																	protocol,
																	from,
																	device->GetAddress (),
																	NetDevice::PacketType (0),
																	false);
		m_totPacketCount++;
  	for (deviceStatI iter = m_deviceStats.begin (); iter != m_deviceStats.end (); iter++)
  	{
    	if (iter->first == device)
    	{
      	if (iter->second.RxCount == 0)
      	{
        	iter->second.RxCount = iter->second.RxCount + 1;
        	iter->second.avgPacketSize = packet->GetSize();
      	}
      	else
      	{
        	iter->second.RxCount = iter->second.RxCount + 1;
        	iter->second.avgPacketSize = ((iter->second.avgPacketSize * (iter->second.RxCount - 1)) + packet->GetSize()) / (iter->second.RxCount); 
      	}
    	}
  	}

		NS_LOG_DEBUG ("Move the packet " << packet << " to the buffer.");
  	m_nodePacketBuffer.EnQueue(nodeEnqueueEntry);

		if(!m_initiator)
		{
			//Initiate the packet transmitor for the for the first time 
    	NS_LOG_DEBUG ("Initiate the packet transmitter.");
    	
			ScheduleTransmit(device);
			m_initiator = 1; // Transmitor has been initialized
		}
	
		//Note: As every packet is buffered, the return is always true
		return true;
	}
}

void
Node::ScheduleTransmit(Ptr<NetDevice> device)
{
	NS_LOG_FUNCTION (this);
	
	m_serviceRate = 9000000000.0;//90Mbps 900Mbps
	double randValue = 0.0, tempTime = 0.0;
	Time t_reSchedule = Time ();
	
  if (!m_nodePacketBuffer.IsEmpty ())
	{
	  randValue = m_rng->GetValue (0.0, 1.0);
	  m_Mue = m_serviceRate / (GetAveragePacketSizeOfRouter () * 8); // convert it to process a bit

	  tempTime = ((-1/m_Mue) *(log (randValue))) * 1000000 ; // microseconds
	  t_reSchedule = MicroSeconds (tempTime);	
	  
  	ReceiveFromBuffer ();
	  m_nextTransmission = Simulator::Schedule (t_reSchedule, &Node::ScheduleTransmit, this, device);
	}
	else
	{
	  m_nextTransmission = Simulator::Schedule (MilliSeconds (m_rng->GetValue (2.0, 4.0)), 
	                                            &Node::ScheduleTransmit, this, device);
	}
}

void
Node::ReceiveFromBuffer(void)
{
  if (m_nodePacketBuffer.IsEmpty())
  {
	  NS_LOG_DEBUG ("No packet to transmit, reschedule.");
	  return;//break;
  }
  else
  {
    NodeQueueEntry deQueueEntry;
    Ptr<Packet> packettoSend;
    m_nodePacketBuffer.DeQueue(deQueueEntry);

    Ptr<Packet> tstPacket = deQueueEntry.GetPacket()->Copy();
		
    NS_LOG_DEBUG (  deQueueEntry.GetNetDevice() 
							      << deQueueEntry.GetPacket() 
							      << deQueueEntry.GetProtocol() 
							      << deQueueEntry.GetFrom() 
							      << deQueueEntry.GetTo() 
							      << deQueueEntry.GetPacketType() 
							      << deQueueEntry.GetPromiscuous());
	
    Ptr<Packet> packetToSoR = deQueueEntry.GetPacket()->Copy ();	
	
		// This part is seriously assume the packet infomration
		// When the packet is handed over to the SoR, all necessary meta data are
		// assumed based on the dequeue entry.
		if (m_nodeType == SoR)
	  {
	    // does the necessary modifications 
	    // and receive the modified packet if necessary.
			// Directly copy the packet to the analyzer module. 
			// There is another alternative method to pass the packet to SoR packet buffer as follow
			// sorInstance.PassPacketToSoR (packetToSoR);
    	sorInstance.AnalyzePacetInformationForIoT (packetToSoR, deQueueEntry.GetNetDevice());
	  }
							
    NS_ASSERT_MSG (Simulator::GetContext () == GetId (), 
								      "Received packet with erroneous context ; " 
								      << "make sure the channels in use are correctly updating events context " 
								      << "when transferring events from one node to another.");
    NS_LOG_DEBUG ("Node " << GetId () 
								      << " ReceiveFromDevice:  dev "
								      << deQueueEntry.GetNetDevice()->GetIfIndex () 
								      << " (type=" 
								      << deQueueEntry.GetNetDevice()->GetInstanceTypeId ().GetName () 
								      << ") Packet UID " 
								      << deQueueEntry.GetPacket()->GetUid ());
								
    for (ProtocolHandlerList::iterator i = m_handlers.begin (); i != m_handlers.end (); i++)
    {
      if (i->device == 0 || (i->device != 0 && i->device == deQueueEntry.GetNetDevice()))
      {
	      if (i->protocol == 0 || i->protocol == deQueueEntry.GetProtocol())
	      {
		      if (deQueueEntry.GetPromiscuous() == i->promiscuous)
		      {
			      i->handler( deQueueEntry.GetNetDevice(),
									      packetToSoR/*deQueueEntry.GetPacket()*/, /*the modified packet passes to the upper layer (routing)*/
									      deQueueEntry.GetProtocol(),
									      deQueueEntry.GetFrom(),
									      deQueueEntry.GetTo(),
									      deQueueEntry.GetPacketType());
		      }
	      }
      }
    }
    for (deviceStatI iter = m_deviceStats.begin (); iter != m_deviceStats.end (); iter++)
    {
      if (iter->first == deQueueEntry.GetNetDevice())
      {
        iter->second.TxCount = iter->second.TxCount + 1;
        break;          
      }
    }
  }
}

void
Node::PrintStats ()
{
  NS_LOG_FUNCTION (this);

  //used for testing purposes  
  uint64_t totBitCount = 0;
  NS_UNUSED (totBitCount); // suppress "set but not used" compiler warning in optimized builds  
  uint8_t count = 1;
  for (deviceStatI iter = m_deviceStats.begin (); iter != m_deviceStats.end (); iter++, count++)
  {
    double h = 0.0, c = 0.0;

    if (count != 1) // omit the LOCALHOST interface
    {
      h = ((iter->second.avgPacketSize * iter->second.RxCount * 8) / Simulator::Now ().GetSeconds ()); // in bps
      
      // Get Channel Attributes
      StringValue getBW;
      std::string temp, bandwidth;
      
      // Get the capacity of the link
      iter->first->GetAttribute("DataRate",getBW);
      bandwidth = getBW.Get ().c_str ();

      temp = bandwidth.substr (0, (bandwidth.size () - 3));
      c = (double) (atof (temp.c_str ())); // in bps 

    }
  }	
  m_outEvent = Simulator::Schedule (Seconds (50), &Node::PrintStats, this);  
}

bool
Node::ReceiveFromDevice (Ptr<NetDevice> device, Ptr<const Packet> packet, uint16_t protocol,
                         const Address &from, const Address &to, NetDevice::PacketType packetType, bool promiscuous)
{
  NS_LOG_FUNCTION (this << device << packet << protocol << &from << &to << packetType << promiscuous);
  NS_ASSERT_MSG (Simulator::GetContext () == GetId (), "Received packet with erroneous context ; " <<
                 "make sure the channels in use are correctly updating events context " <<
                 "when transfering events from one node to another.");
  NS_LOG_DEBUG ("Node " << GetId () << " ReceiveFromDevice:  dev "
                        << device->GetIfIndex () << " (type=" << device->GetInstanceTypeId ().GetName ()
                        << ") Packet UID " << packet->GetUid ());
  bool found = false;

  for (ProtocolHandlerList::iterator i = m_handlers.begin ();
       i != m_handlers.end (); i++)
    {
      if (i->device == 0 ||
          (i->device != 0 && i->device == device))
        {
          if (i->protocol == 0 || 
              i->protocol == protocol)
            {
              if (promiscuous == i->promiscuous)
                {
                  i->handler (device, packet, protocol, from, to, packetType);
                  found = true;
                }
            }
        }
    }
  return found;
}
void 
Node::RegisterDeviceAdditionListener (DeviceAdditionListener listener)
{
  NS_LOG_FUNCTION (this << &listener);
  m_deviceAdditionListeners.push_back (listener);
  // and, then, notify the new listener about all existing devices.
  for (std::vector<Ptr<NetDevice> >::const_iterator i = m_devices.begin ();
       i != m_devices.end (); ++i)
    {
      listener (*i);
    }
}
void 
Node::UnregisterDeviceAdditionListener (DeviceAdditionListener listener)
{
  NS_LOG_FUNCTION (this << &listener);
  for (DeviceAdditionListenerList::iterator i = m_deviceAdditionListeners.begin ();
       i != m_deviceAdditionListeners.end (); i++)
    {
      if ((*i).IsEqual (listener))
        {
          m_deviceAdditionListeners.erase (i);
          break;
         }
    }
}
 
void 
Node::NotifyDeviceAdded (Ptr<NetDevice> device)
{
  NS_LOG_FUNCTION (this << device);
  for (DeviceAdditionListenerList::iterator i = m_deviceAdditionListeners.begin ();
       i != m_deviceAdditionListeners.end (); i++)
    {
      (*i) (device);
    }  
}
 

} // namespace ns3
