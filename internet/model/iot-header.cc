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
 * Author: Janaka Wijekoon <janaka@west.s.keio.ac.jp>
 */

#include "ns3/iot-header.h"
#include "ns3/header.h"
#include "ns3/log.h"
#include "ns3/ipv4-address.h"

NS_LOG_COMPONENT_DEFINE ("IoTHeader");

namespace ns3
{
	NS_OBJECT_ENSURE_REGISTERED (IoTHeader);
	TypeId
	IoTHeader::GetTypeId (void)
	{
  		static TypeId tid = TypeId ("ns3::IoTHeader")
    		.SetParent<Header> ()
    		.AddConstructor<IoTHeader> ();
  		return tid;
	}
	TypeId
	IoTHeader::GetInstanceTypeId (void) const
	{
  		NS_LOG_FUNCTION (this);
  		return GetTypeId ();
	}
	IoTHeader::IoTHeader(): m_type (1), 
	                        m_storeStatus (0),
	                        m_sorID (0), 
	                        m_ingressSoRId (0),
	                        m_ingressSoRAddress ("0.0.0.0"),
	                        m_processedSoRID (0),
													m_hopCount (0),
													/*m_iotData (0)*/m_iotData (""),
													m_timestamp (0.0),
													m_areaId (0),
													m_deviceId (0)
	{

	}
	
	IoTHeader::~IoTHeader()
	{
	}
	
	void  
	IoTHeader::Print (std::ostream &os) const
	{
    // TODO
	}

	uint32_t 
	IoTHeader::GetSerializedSize (void) const
	{
	  return 30 + 2 + m_iotData.size () + 1 /*size of the data + additional one byte to end*/;
	}
		
  void  IoTHeader::Serialize (Buffer::Iterator start) const
	{
		Buffer::Iterator i = start;
		
		i.WriteU8 (m_type);
		i.WriteU8 (m_storeStatus);
		i.WriteHtonU32 (m_sorID);
		i.WriteHtonU32 (m_ingressSoRId);
    i.WriteHtonU32 (m_ingressSoRAddress.Get ());
		i.WriteHtonU32 (m_processedSoRID);
		i.WriteHtonU16 (m_hopCount);

		i.WriteHtonU32 (m_timestamp*1000); // As it's stored as double variable 

    i.WriteHtonU16 (m_areaId);
		i.WriteHtonU32 (m_deviceId); // As it's stored as double variable 		
				
		i.WriteU16 (m_iotData.size () + 1);
		i.Write ((uint8_t*) m_iotData.c_str (), m_iotData.size () + 1);
	}

  uint32_t
	IoTHeader::Deserialize (Buffer::Iterator start)
	{
		Buffer::Iterator i = start;
    uint16_t receivedSize = 0;
    m_iotData = std::string ("");

		    
		m_type = i.ReadU8();
		m_storeStatus = i.ReadU8();
		m_sorID = i.ReadNtohU32 ();
		m_ingressSoRId = i.ReadNtohU32 ();
    m_ingressSoRAddress.Set (i.ReadNtohU32 ());
		m_processedSoRID = i.ReadNtohU32 ();
		m_hopCount = i.ReadNtohU16 ();
		m_timestamp = (double)i.ReadNtohU32 ()/1000;	
		m_areaId = i.ReadNtohU16 ();
		m_deviceId = i.ReadNtohU32 ();		
				
		receivedSize = i.ReadU16 ();

		char data[receivedSize];
		i.Read ((uint8_t*) data, receivedSize);
		m_iotData = data;

		return GetSerializedSize();
	}
}//Namespace 

