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
#ifndef NODE_H
#define NODE_H

#include <vector>
#include <map>

#include "ns3/object.h"
#include "ns3/callback.h"
#include "ns3/ptr.h"
#include "ns3/net-device.h"
#include "ns3/event-id.h"
#include "ns3/random-variable-stream.h"

#include "node-packet-queue.h"
#include "sor.h"

namespace ns3 {

class Application;
class Packet;
class Address;
class Internet;


/**
 * \ingroup network
 *
 * \brief A network Node.
 *
 * This class holds together:
 *   - a list of NetDevice objects which represent the network interfaces
 *     of this node which are connected to other Node instances through
 *     Channel instances.
 *   - a list of Application objects which represent the userspace
 *     traffic generation applications which interact with the Node
 *     through the Socket API.
 *   - a node Id: a unique per-node identifier.
 *   - a system Id: a unique Id used for parallel simulations.
 *
 * Every Node created is added to the NodeList automatically.
 */
class Node : public Object
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  Node();
  /**
   * \param systemId a unique integer used for parallel simulations.
   */
  Node(uint32_t systemId);

  virtual ~Node();

  /**
   * \returns the unique id of this node.
   * 
   * This unique id happens to be also the index of the Node into
   * the NodeList. 
   */
  uint32_t GetId (void) const;

  /**
   * \returns the system id for parallel simulations associated
   *          to this node.
   */
  uint32_t GetSystemId (void) const;

  /**
   * \brief Associate a NetDevice to this node.
   *
   * \param device NetDevice to associate to this node.
   * \returns the index of the NetDevice into the Node's list of
   *          NetDevice.
   */
  uint32_t AddDevice (Ptr<NetDevice> device);
  /**
   * \brief Retrieve the index-th NetDevice associated to this node.
   *
   * \param index the index of the requested NetDevice
   * \returns the requested NetDevice.
   */
  Ptr<NetDevice> GetDevice (uint32_t index) const;
  /**
   * \returns the number of NetDevice instances associated
   *          to this Node.
   */
  uint32_t GetNDevices (void) const;

  /**
   * \brief Associate an Application to this Node.
   *
   * \param application Application to associate to this node.
   * \returns the index of the Application within the Node's list
   *          of Application.
   */
  uint32_t AddApplication (Ptr<Application> application);
  /**
   * \brief Retrieve the index-th Application associated to this node.
   *
   * \param index the index of the requested Application
   * \returns the requested Application.
   */
  Ptr<Application> GetApplication (uint32_t index) const;

  /**
   * \returns the number of Application instances associated to this Node.
   */
  uint32_t GetNApplications (void) const;

  /**
   * A protocol handler
   *
   * \param device a pointer to the net device which received the packet
   * \param packet the packet received
   * \param protocol the 16 bit protocol number associated with this packet.
   *        This protocol number is expected to be the same protocol number
   *        given to the Send method by the user on the sender side.
   * \param sender the address of the sender
   * \param receiver the address of the receiver; Note: this value is
   *                 only valid for promiscuous mode protocol
   *                 handlers.  Note:  If the L2 protocol does not use L2
   *                 addresses, the address reported here is the value of 
   *                 device->GetAddress().
   * \param packetType type of packet received
   *                   (broadcast/multicast/unicast/otherhost); Note:
   *                   this value is only valid for promiscuous mode
   *                   protocol handlers.
   */
  typedef Callback<void,Ptr<NetDevice>, Ptr<const Packet>,uint16_t,const Address &,
                   const Address &, NetDevice::PacketType> ProtocolHandler;
  /**
   * \param handler the handler to register
   * \param protocolType the type of protocol this handler is 
   *        interested in. This protocol type is a so-called
   *        EtherType, as registered here:
   *        http://standards.ieee.org/regauth/ethertype/eth.txt
   *        the value zero is interpreted as matching all
   *        protocols.
   * \param device the device attached to this handler. If the
   *        value is zero, the handler is attached to all
   *        devices on this node.
   * \param promiscuous whether to register a promiscuous mode handler
   */
  void RegisterProtocolHandler (ProtocolHandler handler, 
                                uint16_t protocolType,
                                Ptr<NetDevice> device,
                                bool promiscuous=false);
  /**
   * \param handler the handler to unregister
   *
   * After this call returns, the input handler will never
   * be invoked anymore.
   */
  void UnregisterProtocolHandler (ProtocolHandler handler);

  /**
   * A callback invoked whenever a device is added to a node.
   */
  typedef Callback<void,Ptr<NetDevice> > DeviceAdditionListener;
  /**
   * \param listener the listener to add
   *
   * Add a new listener to the list of listeners for the device-added
   * event. When a new listener is added, it is notified of the existence
   * of all already-added devices to make discovery of devices easier.
   */
  void RegisterDeviceAdditionListener (DeviceAdditionListener listener);
  /**
   * \param listener the listener to remove
   *
   * Remove an existing listener from the list of listeners for the 
   * device-added event.
   */
  void UnregisterDeviceAdditionListener (DeviceAdditionListener listener);



  /**
   * \returns true if checksums are enabled, false otherwise.
   */
  static bool ChecksumEnabled (void);
  
 // \name for queue the packets at the nodes
 // \{  
   /**
   * \This method is implemented to return the node queue size.
   * This part is added By janaka Wijekoon
   */  
   uint16_t GetBufferSize(void);
   
 	/**
 	 * \brief get the number of packets that have been transmitted / received from/to a netdevice
 	 * \param device the netdevice
    * \returns the number of packets
    */
   uint64_t GetNofPacketsOfDevice (Ptr<NetDevice> device);
 	
 	/**
 	 * \brief get the average packet size of all received/transmitted packets from/to a netdevice
 	 * \param device the netdevice
 	 * \returns the average packet size
 	 */
 	uint64_t GetAveragePacketSizeOfDevice (Ptr<NetDevice> device);
 	
 	/**
 	 * \brief get the cumulative average packet size of the entire router.
 	 * \returns the average packet size of the router
 	 */	
 	double GetAveragePacketSizeOfRouter (void);	
 	
 	/**
 	 * \brief Get the Mue of the server
 	 * \returns the mue value
 	 */
 	double GetRouterMue(void);
 
 	/**
 	 * \brief Get the packet per second value of the serve
 	 * \returns the service rate
 	 */	
 	double GetRouterServiceRate (void);
 
 	/**
 	 * \brief Get and Set the arrival rate of the router
 	 * \returns the average arrival rate of the router
 	 */
 	double GetRouterLambda (void);
 	
 	void SetRouterLambda (uint16_t lambda);
 // \}

 // \name Since the nodes are generally defined as common nodes, 
 //       we separated the node as follows: 
 // \{ 
  enum NodeType
 	{
 		SERVER_NODE = 0X01,
 		CLIENT_NODE = 0X02,
 		ROUTER = 0X03,
 		SoR = 0x04,
 	};
 	// \}
 
 // \name Catogarize the nodes according to the place, 
 //       we separated the node as follows: 
 // \{ 
  enum NodePlacement
 	{
 		MASTER_ROUTER = 0X01, // !< most of the times Master routers are the gateway routers for cloud networks.
 		                      // These routers are used to take the redirection decisions 
 		INGRESS_ROUTER = 0X02,// !< consideres as the provider edge routers for end-users, applications, or IoT devices
 		CORE_ROUTER = 0X03,// !< Core routers in the middle of providers network
 	};
 	// \} 
 
 // \name The processor type is vary according to the node., 
 //       we defined the processor types as follows: 
 // \{ 
  enum ProcCostFactor /*https://www.cpubenchmark.net/socketType.html*/
 	{
 		I5 = 0X04, /*Intel Core i5-3570 @ 3.40GHz*/
 		I7 = 0X02, /*Intel Core i7-3770K @ 3.50GHz*/
 		XEON = 0x01, /*Intel Xeon E3-1290 V2 @ 3.70GHz*/
 	};
 	// \} 	

 // \name The processor type is vary according to the node., 
 //       we defined the processor types as follows: 
 // \{ 
  enum MemCostFactor /*// calculated according to the 1Mb sequence access time given in https://gist.github.com/jboner/2841832*/
 	{
 		HDD = 0X08, /*HDD*/  
 		SSD = 0X04, /*SSD*/
 		RAM = 0X01, /*RAM*/
 	};
 	// \} 	
 	 
 	/**
 	 * \brief Set the type of the node
 	 * \param nodeType the type of this node suppose to be*/
 	void SetNodeType (NodeType nodeType)
 	{
 		m_nodeType = nodeType;
 	}  
 	NodeType GetNodeType (void)
 	{
 		return m_nodeType;
 	} 
 	void SetNodePlacement (NodePlacement nodePlacement)
 	{
 		m_nodePlacement = nodePlacement;
 	}  
 	NodePlacement GetNodePlacement (void)
 	{
 		return m_nodePlacement;
 	}  	
//-- 	
 	void SetProcCostFactor (ProcCostFactor procType)
 	{
 		m_cpuType = procType;
 	}  
 	ProcCostFactor GetProcCostFactor (void)
 	{
 		return m_cpuType;
 	} 

 	void SetMemCostFactor (MemCostFactor memType)
 	{
 		m_memType = memType;
 	}  
 	MemCostFactor GetMemCostFactor (void)
 	{
 		return m_memType;
 	} 

 	void SetMemorySize (uint32_t memSize)
 	{
 		m_totalMemorySize = memSize;
 	}  
 	uint32_t GetMemorySize (void)
 	{
 		return m_totalMemorySize;
 	} 
protected:
  /**
   * The dispose method. Subclasses must override this method
   * and must chain up to it by calling Node::DoDispose at the
   * end of their own DoDispose method.
   */
  virtual void DoDispose (void);
  virtual void DoInitialize (void);
private:

  /**
   * \brief Notifies all the DeviceAdditionListener about the new device added.
   * \param device the added device to notify.
   */
  void NotifyDeviceAdded (Ptr<NetDevice> device);

  /**
   * \brief Receive a packet from a device in non-promiscuous mode.
   * \param device the device
   * \param packet the packet
   * \param protocol the protocol
   * \param from the sender
   * \returns true if the packet has been delivered to a protocol handler.
   */
  bool NonPromiscReceiveFromDevice (Ptr<NetDevice> device, Ptr<const Packet> packet, uint16_t protocol, const Address &from);
  /**
   * \brief Receive a packet from a device in promiscuous mode.
   * \param device the device
   * \param packet the packet
   * \param protocol the protocol
   * \param from the sender
   * \param to the destination
   * \param packetType the packet type
   * \returns true if the packet has been delivered to a protocol handler.
   */
  bool PromiscReceiveFromDevice (Ptr<NetDevice> device, Ptr<const Packet> packet, uint16_t protocol,
                                 const Address &from, const Address &to, NetDevice::PacketType packetType);
  /**
   * \brief Receive a packet from a device.
   * \param device the device
   * \param packet the packet
   * \param protocol the protocol
   * \param from the sender
   * \param to the destination
   * \param packetType the packet type
   * \param promisc true if received in promiscuous mode
   * \returns true if the packet has been delivered to a protocol handler.
   */
  bool ReceiveFromDevice (Ptr<NetDevice> device, Ptr<const Packet>, uint16_t protocol,
                          const Address &from, const Address &to, NetDevice::PacketType packetType, bool promisc);

  /**
   * \brief Finish node's construction by setting the correct node ID.
   */
  void Construct (void);

  /**
   * \brief Protocol handler entry.
   * This structure is used to demultiplex all the protocols.
   */
  struct ProtocolHandlerEntry {
    ProtocolHandler handler; //!< the protocol handler
    Ptr<NetDevice> device;   //!< the NetDevice
    uint16_t protocol;       //!< the protocol number
    bool promiscuous;        //!< true if it is a promiscuous handler
  };

  /// Typedef for protocol handlers container
  typedef std::vector<struct Node::ProtocolHandlerEntry> ProtocolHandlerList;
  /// Typedef for NetDevice addition listeners container
  typedef std::vector<DeviceAdditionListener> DeviceAdditionListenerList;

  uint32_t    m_id;         //!< Node id for this node
  uint32_t    m_sid;        //!< System id for this node
  std::vector<Ptr<NetDevice> > m_devices; //!< Devices associated to this node
  std::vector<Ptr<Application> > m_applications; //!< Applications associated to this node
  ProtocolHandlerList m_handlers; //!< Protocol handlers in the node
  DeviceAdditionListenerList m_deviceAdditionListeners; //!< Device addition listeners in the node

 	NodeType m_nodeType; //!< This indicate the type of the node.	
  NodePlacement m_nodePlacement; //!< The place where the node is attached to
 	ProcCostFactor m_cpuType; //!< This says what type of CPU a node consists of //--
 	MemCostFactor m_memType; //!< This says what type of memory a node consists of //--
	uint32_t m_totalMemorySize; //!< the memory size of the device (in Mb)
 															//	 This could be RAM, SSD, or HDD size according to the configuration of m_memType
 	
 	
// \name for queue the packets at the nodes
// \{

  /**
   * \brief schedule the next transmit for the protocol layer.
   * this has been implemented to call over every given time interval.
   * \param device the netdevice of the packet transmit scheduler
   */
	void ScheduleTransmit(Ptr<NetDevice> device);
	
	 /**
   * \brief This method is called by the ScheduleTransmit method every given 
   * time interval (The time interval is vary according to the service rate 
   * of the router). This is similar to the default function "ReceiveFromDevice".
   * however, at the time, the ReceiveFromDevice callback is triggered,
   * instead of sending the packet to the protocol handling layer, the packet will be
   * buffered in to the queue. Then, for each service rate of the router (i.e., mue)
   * this method will be called and handle (i.e., forward to the protocol handler) one packet at a time. 
   */
	void ReceiveFromBuffer(void);
	
  /**
  * \brief the function developed for debugin purposes.
  * every m_printDuration the function will output number of protocol messages
  */
  void PrintStats ();		
	
	NodeQueue m_nodePacketBuffer; //!< the object of the packet queue
	EventId m_nextTransmission; //!< event to schedule the next packet transmission to the protocol layer
	uint8_t m_initiator; //!< initiate the packet queue
	uint64_t m_totPacketCount; //!< total packets received to the device
	double m_Mue; //!< service rate
	double m_Lambda; //!< packet arrival rate
	double m_serviceRate; //!< service rate of the router
	
  struct DeviceStats {
    uint64_t RxCount; //!< Number of received packet to the NetDevice 
    uint64_t avgPacketSize; //!< Average packet size handled by the NetDevice
    uint64_t TxCount; //!< Number of packets sent from the NetDevice
  };

  // Container for the NetDevice and its Statistics
  typedef std::list<std::pair <Ptr<NetDevice>, DeviceStats> > deviceStat;

  // Const Iterator for the NetDevice and its Statistics
  typedef std::list<std::pair <Ptr<NetDevice>, DeviceStats> >::const_iterator deviceStatCI;

  // Iterator for the NetDevice and its Statistics
  typedef std::list<std::pair <Ptr<NetDevice>, DeviceStats> >::iterator deviceStatI;
  
  deviceStat m_deviceStats;

  Ptr<UniformRandomVariable> m_rng; //!< Rng stream.
  
// \name for debugging
// \{
  EventId m_outEvent; //!< Next statistic printing even
// \}  
// \}

// \name For SoR implementation
// {
  ns3::SoR sorInstance; //!< the local instance of the SoR

public:
  uint16_t GetAvailableSpace (void)
  {
    return sorInstance.GetAvailableMemory ();
  }
// }  
};

} // namespace ns3

#endif /* NODE_H */
