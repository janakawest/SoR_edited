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

#ifndef SOR_H
#define SOR_H

#include <string>
#include <vector>
#include <queue>
#include <sys/time.h>

#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/timer.h"
#include "ns3/random-variable-stream.h"
#include "ns3/ipv4-address.h"

#define AVG_DATA_SIZE 2 /*in MB*/

//#include "ns3/packet.h"
//#include "ns3/core-module.h"
namespace ns3 {

class Node;
class Application;
class Packet;
class Address;
class Internet;
class Socket;
class NetDevice;
class Ipv4;
class Ipv4Address;


	/**
	 * color definitions*/
	#define C_RESET   "\033[0m"
	#define C_BLACK   "\033[30m"      /* Black */
	#define C_RED     "\033[31m"      /* Red */
	#define C_GREEN   "\033[32m"      /* Green */
	#define C_YELLOW  "\033[33m"      /* Yellow */
	#define C_BLUE    "\033[34m"      /* Blue */
	#define C_MAGENTA "\033[35m"      /* Magenta */
	#define C_CYAN    "\033[36m"      /* Cyan */
	#define C_WHITE   "\033[37m"      /* White */
	#define C_BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
	#define C_BOLDRED     "\033[1m\033[31m"      /* Bold Red */
	#define C_BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
	#define C_BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
	#define C_BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
	#define C_BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
	#define C_BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
	#define C_BOLDWHITE   "\033[1m\033[37m"      /* Bold White */
/**
 * \ingroup network
 *
 * \brief The SoR module attached to each node.
 *
 *    This method will analyze the packets up to application layer information
 *    Furthermore, this class is used to store the content in in-memory database
 *    The in memory data base is used to provide user services such as CDN redirection and IoT services. 
 */
/**
 * This class has been particularly designed to store the IoT data records
 * in an SoR. Most of the times, the IoT data records are stored in the 
 * Ingress, i.e., gateway routers of the areas, routers. 
 * */
class IoTRecord
{
public:
  IoTRecord (uint16_t areaId, uint32_t sourceId, uint16_t m_distance, /*uint32_t*/std::string data): m_areaId (areaId),
                                                                        m_devId (sourceId),
																																				m_distanceFromSource (m_distance),
																																				m_data (data)
	{
		SetTime ();
	}	 

	~IoTRecord ()
	{
		/* dstrctr*/
	}
	
	void SetAreaId (uint32_t areaId)
  {
    m_areaId = areaId;
  }  
  uint16_t GetAreaId (void)
  {
    return m_areaId;
  }  
  
	void SetDeviceId (uint32_t id)
  {
    m_devId = id;
  }  
  uint32_t GetDeviceId (void)
  {
    return m_devId;
  }  

	void SetDistance (uint16_t distance)
  {
    m_distanceFromSource = distance;
  }  
  uint16_t GetDistance (void)
  {
    return m_distanceFromSource;
  }
  
	void SetData (/*uint32_t*/std::string data)
  {
    m_data = data;
  }  
  /*uint32_t*/std::string GetData (void)
  {
    return m_data;
  } 

	void SetTime (void)
  {
    m_timeStamp = Simulator::Now ().GetSeconds ();
  }  
  double GetLifeTime (void)
  {
    return (Simulator::Now ().GetSeconds () -  m_timeStamp);
  }
   
private:
  uint16_t m_areaId;
  uint32_t m_devId;
  uint16_t m_distanceFromSource;
  /*uint32_t*/std::string m_data; //!< TODO convert it to std::string
  double m_timeStamp;
}; //IoTRecord

/**
 * This class has been designed to store the recommendation records which are
 * recommended by the a Master, i.e., the gateway routers of the Cloud/Storage, 
 * routers. When a master router receives a IoT data packet, which is not yet 
 * served form either its gateway router or the core routers, the Master router
 * checks the gateway router that has the requested IoT data. When it found the 
 * gateway router, the master router send a recommendation message to the ingress
 * router of that particular data packet. When the ingress router received the 
 * recommendation message, it uses this structure to store the respective ingress
 * router.*/
class RecommendationRecord
{
public:
  RecommendationRecord (uint16_t areaId, uint32_t deviceId, double delay, uint32_t sorId, Ipv4Address contactPoint) : m_areaId (areaId),
                                                                                                                      m_deviceId (deviceId), 
                                                                                                                      m_routingDelay (delay),
                                                                                                                      m_SoRId (sorId),
                                                                                                                      m_contactPoint  (contactPoint)
  {
    /*constrctr*/
  }
                                                                                                                        
  ~RecommendationRecord ()
  {
    /*dstrctr*/
  }
  
	void SetAreaID (uint16_t id)
  {
    m_areaId = id;
  }  
  uint16_t GetAreaId (void)
  {
    return m_areaId;
  }
	void SetDeviceId (uint32_t id)
  {
    m_deviceId = id;
  }  
  uint32_t GetDeviceId (void)
  {
    return m_deviceId;
  }
	void SetRoutingDelay (double delay)
  {
    m_routingDelay = delay;
  }  
  double GetRoutingDelay (void)
  {
    return m_routingDelay;
  }  
	void SetSoRId (uint32_t id)
  {
    m_SoRId = id;
  }  
  uint32_t GetSoRId (void)
  {
    return m_SoRId;
  }
  void SetContactPoint (Ipv4Address address)
  {
    m_contactPoint = address;
  }
  Ipv4Address GetContactPoint (void)
  {
    return m_contactPoint;
  }
      
private:
  uint16_t m_areaId; //!< The Area ID
  uint32_t m_deviceId; //!< The Device ID
  double m_routingDelay; //!< Time to reach the device (the gateway SoR)
  uint32_t m_SoRId; //!< The gateway SoR that has the data stored in 
  Ipv4Address m_contactPoint; //!< The IP address that should contact in order to download the data
};

/**
 * The topology view class is mainly maintained by the master routers.
 * Master routers sniff the IoT data packets and update this table continuously.
 * Note that the master routers are the gateway routers of the Data centers,
 * which are used to store the IoT data. Therefore, technically, the Master routers 
 * know all information about the areas of the IoT networks and the devices.
 * So that the master routers maintain following table. When an IoT data request
 * message reaches to a master router, it first check the ingress router of the 
 * area, which that data is actually generated. Finally, the master router creates
 * a recommendation packet and send to the Ingress router about the point can use
 * to connect the data. This method has its own pros and cons and we have to
 * evaluate carefully.  
 * */
class TopologyView
{
public:
  TopologyView (uint16_t areaId, uint32_t deviceId, double lastUpdateTime, uint32_t ingressRouterId, Ipv4Address contactPoint) : m_areaId (areaId),
                                                                                                                                 m_deviceId (deviceId),
                                                                                                                                 m_lastUpdateTime (lastUpdateTime),
                                                                                                                                 m_ingressRouterId (ingressRouterId),
                                                                                                                                 m_contactPoint (contactPoint)
  {
    /*construct*/
  }              
  
  ~TopologyView ()
  {
    /*destruct*/
  }           

	void SetAreaID (uint16_t id)
  {
    m_areaId = id;
  }  
  uint16_t GetAreaId (void)
  {
    return m_areaId;
  }
	void SetDeviceId (uint32_t id)
  {
    m_deviceId = id;
  }  
	void SetUpdateTime (double time)
  {
    m_lastUpdateTime = time;
  }  
  double GetUpdateTime (void)
  {
    return m_lastUpdateTime;
  }   
	void SetIngressSoRId (uint32_t id)
  {
    m_ingressRouterId = id;
  }  
  uint32_t GetIngressRouterId (void)
  {
    return m_ingressRouterId;
  }  
  void SetContactPoint (Ipv4Address address)
  {
    m_contactPoint = address;
  }
  Ipv4Address GetContactPoint (void)
  {
    return m_contactPoint;
  }   
                                                                                                          
private:
	uint16_t m_areaId; //!< the area of the IoT network
	uint32_t m_deviceId; //!< the device Id of that particular area
	double m_lastUpdateTime; //!< the last time that master router received a data 
	uint32_t m_ingressRouterId; // !< the Ingress router Id of that particular area
	Ipv4Address m_contactPoint; //!< the IP address of that particular Ingress router
};

class SoR : public Object
{
public:
	static TypeId GetTypeId (void);
	SoR ();
	virtual ~SoR ();
	
	void InitializeSoR (Ptr<Node> node);
	void PassPacketToSoR (Ptr<Packet> packet);
	bool AnalyzePacetInformationForIoT (Ptr<Packet> packet, Ptr<NetDevice> interface);
	void ReplyIoTRequest (Ipv4Address packetSource, Ipv4Address packetDestination, uint16_t packetSourcePort, uint16_t packetDestinationPort);
	void CollectNeighborInformation (void);
	void UpdateNeighborInformation (void);
	bool FindIoTDataInCache (std::string data/*uint32_t data*/); //TODOstd::string data ;
	//--
	double GetNextHopCapability (Ptr<NetDevice> device);
	double GetMyCapability (void);	
	uint8_t GetNextHopType (Ptr<NetDevice> device);

	void DoDispose (void);

	typedef std::pair < IoTRecord*, EventId > IoTCacheRecord;
	typedef std::list < std::pair < IoTRecord*, EventId> > IoTCacheTableInstance;
	typedef std::list < std::pair < IoTRecord*, EventId> >::iterator IoTCacherecordI;

  uint16_t GetAvailableMemory (void)
  {
    return (m_maxMemory - m_iotCacheTable.size ());
  }
  
private:
  void RecommandInformation (void);
  void InitializeCommunicationSocket (uint16_t port);  
  void HandleRecommendationPackets (Ptr<Socket> socket);  
  
  bool m_sorEnabled; //!< to enable the SoR
  Ptr<Node> m_node; //!< the base node
  uint64_t m_sorId; //!< SoR ID and this is same as the node ID
  Ptr<Ipv4> m_ipv4; //!< IPv4 reference
  Ipv4Address m_localAddress; //!< the IP address of the SoR. By default this is the IP address of Interface 1
  EventId m_neighborUpdateEvent; //!< Next statistic printing event
  Ptr<UniformRandomVariable> m_rng; //!< Rng stream.
  
  Ptr<Socket> m_commSocket; //!< Socket uses for communication

	std::queue < Ptr<Packet> > packetBuffer; //!< the packet buffer  
  uint16_t m_maxMemory; // !< Maximum number of records that are allowed to store in this SoR//--

	IoTCacheTableInstance m_iotCacheTable;	

	struct NeighborDetails 
	{
		uint32_t neighborID;
		Ptr<NetDevice> neighborDevice;
		uint8_t processorType;
		uint8_t memoryType;
		uint32_t localInterfaceNo;
		Ptr<NetDevice> localInterface;
		uint16_t availableMemory;//--
		double linkCost; //!< this is calculated according to the OSPF method (100Mbps/linkBW)//--
		uint8_t nodeType; //!< The type of the node, router, SoR, Server, or client
	};
	typedef std::list <struct SoR::NeighborDetails> NeighborTable;
	typedef std::list <struct SoR::NeighborDetails>::iterator NeighborTableI;

	NeighborTable m_neighborInfomrationTable;
  
}; // Class SoR
} // namespace ns3
#endif /* SOR_H */
