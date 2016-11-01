/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013 WestLab Keio University
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
 * Author: Janaka Wijekoon <janaka@west.sd.keio.ac.jp>
 */

#ifndef IOT_HEADER_H
#define IOT_HEADER_H

#include "ns3/header.h"
#include "ns3/ipv4-address.h"

namespace ns3 {
/**
 * \brief Packet header for IPv4
 */

class IoTHeader : public Header
{
	public:

	enum PacketType 
	{
    IOT_DATA_PACKET = 0x00, // !< Packets that have the IoT device data
    IOT_REQUEST_PACKET = 0x01, // !< Packet that are requesting the IoT data
    IOT_REPLY_PACKET = 0x02, // !< Packet that is replying the IoT data request  
    IOT_PROCESSED_PACKET = 0x03, // !< A packet that has been processed enroute (Specially for the request packets) 
    IOT_RECOMMENDATION_PACKET = 0x04, //!< A packet used to recommend the data cache points to the Ingress routers.
  };
	enum DataStoreStatus 
	{
    NOT_STORED = 0x00, // !< Data is not stored 
    STORED = 0x01, // !< data is stored in the SoR indicated in m_storedID
  };
  
  uint32_t SoRID; // !< last passed SoR ID
  uint32_t StoredID;  // !< the ID of the SoR which stored the content

	IoTHeader();
	~IoTHeader();

  void SetType (IoTHeader::PacketType type)
  {
    m_type = (uint8_t)type;
  }  
  uint8_t GetType (void)
  {
    return m_type;
  }
  
  void SetStoreStatus (IoTHeader::DataStoreStatus status)
  {
    m_storeStatus = (uint8_t)status;
  }  
  uint8_t GetStoreStatus (void)
  {
    return m_storeStatus;
  }
    
  void SetProcessedSoRId (uint32_t id)
  {
    m_processedSoRID = id;
  }  
  uint32_t GetProcessedSoRId (void)
  {
    return m_processedSoRID;
  }  

	void SetLastSoRId (uint32_t id)
  {
    m_sorID = id;
  }  
  uint32_t GetStoredID (void)
  {
    return m_sorID;
  }  

	void SetHopCount (void)
	{
		m_hopCount += 1;
	}
	uint16_t GetHopCount (void)
	{
		return m_hopCount;
	}

	void SetData (/*uint32_t*/std::string data)
	{
		m_iotData = data;
	}
	/*uint32_t*/std::string GetData (void)
	{
		return m_iotData;
	}

	void SetTimestamp (double time)
	{
		m_timestamp = time;
	}
	double GetTimestamp (void)
	{
		return m_timestamp;
	}

	void SetIngressSoRId (uint32_t id)
  {
    m_ingressSoRId = id;
  }  
  uint32_t GetIngressSoRId (void)
  {
    return m_ingressSoRId;
  }  
  
	void SetIngressSoRAddress (Ipv4Address address)
  {
    m_ingressSoRAddress = address;
  }  
  Ipv4Address GetIngressSoRAddress (void)
  {
    return m_ingressSoRAddress;
  }   
  
	void SetDeviceId (uint32_t id)
  {
    m_deviceId = id;
  }  
  uint32_t GetDeviceId (void)
  {
    return m_deviceId;
  }  

	void SetAreaId (uint16_t id)
	{
		m_areaId = id;
	}
	uint16_t GetAreaId (void)
	{
		return m_areaId;
	}
 

	static TypeId GetTypeId ();
  virtual TypeId GetInstanceTypeId (void) const;
  virtual void Print (std::ostream &os) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);


	private:
	uint8_t m_type; //!< the IoT packet type
	uint8_t m_storeStatus; //!< the store state of the data carrying by this packet
  uint32_t m_sorID; //!< last passed SoR ID
  uint32_t m_ingressSoRId; //!< Ingress SoR ID
  Ipv4Address m_ingressSoRAddress; // !< this is the address bound to the SoR
  uint32_t m_processedSoRID; // !< the ID of the SoR which stored the content or the ID that replied the request
	uint16_t m_hopCount; //!< Number of hops a packet traveled
	/*uint32_t*/std::string m_iotData; //!< IoT data that uses. This has to be AreaID:Dev.ID:DATA
	double m_timestamp; //!< the time stamp indicates the packet generation time
	uint16_t m_areaId; //!< The area that the device belongs to 
	uint32_t m_deviceId; //!< The device Id
  
};
} // namespace ns3
#endif /* IOT_HEADER_H*/ 
