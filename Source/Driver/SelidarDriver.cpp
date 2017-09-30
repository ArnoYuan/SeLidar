#include <stdio.h>
#include <iostream>
#include "SelidarDriver.h"
#include "SelidarTypes.h"
#include <Time/Utils.h>
#include <Console/Console.h>

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

#ifndef _countof
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#endif

using std::cin;
using std::cout;
using std::endl;

namespace NS_Selidar
{
  
// Serial Driver Impl
  
  SelidarDriver::SelidarDriver ()
      : connected (false), scanning (false)
  {
    rxtx = new Serial ();
  }
  
  SelidarDriver::~SelidarDriver ()
  {
    // force disconnection
    disconnect ();
    
    delete rxtx;
  }
  
  int
  SelidarDriver::connect (const char * port_path, unsigned int baudrate,
                          unsigned int flag)
  {
    if (isConnected ())
      return Denied;
    
    if (!rxtx)
      return Invalid;
    
    {
      boost::mutex::scoped_lock auto_lock (rxtx_lock);
      
      // establish the serial connection...
      if (!rxtx->bind (port_path, baudrate) || !rxtx->open ())
      {
        return Invalid;
      }
      
      rxtx->flush (0);
    }
    
    connected = true;
    
    return Success;
  }
  
  void
  SelidarDriver::disconnect ()
  {
    if (!connected)
      return;
    stop ();
    rxtx->close ();
  }
  
  bool
  SelidarDriver::isConnected ()
  {
    return connected;
  }
  
  int
  SelidarDriver::sendCommand (unsigned char cmd)
  {
    unsigned char pkt_header[16] = { 0 };
    unsigned char pkg[128] = { 0 };
    int pkg_size = 0;
    SelidarPacketHead * header =
        reinterpret_cast<SelidarPacketHead *> (pkt_header);
    unsigned char checksum = 0;
    
    if (!connected)
      return Failure;
    
    header->sync_word = SELIDAR_CMD_SYNC_BYTE;
    header->cmd.error = 0;
    header->cmd.cmd_word = cmd;
    header->payload_len = 0;
    header->length = sizeof(SelidarPacketHead) + 1;
    
    for (size_t i = 0; i < sizeof(SelidarPacketHead); i++)
    {
      checksum ^= pkt_header[i];
    }
    
    memcpy (pkg, pkt_header, sizeof(SelidarPacketHead));
    pkg_size += sizeof(SelidarPacketHead);
    
    memcpy (pkg + pkg_size, &checksum, 1);
    pkg_size++;
    
    rxtx->senddata (pkg, pkg_size);
    //console.dump (pkg, pkg_size);
    
    return Success;
  }
  
  /*
  int
  SelidarDriver::reset (unsigned int timeout)
  {
    int ans;
    
    {
      boost::mutex::scoped_lock auto_lock (rxtx_lock);
      
      if (IS_FAIL(ans = sendCommand (ResetReq)))
      {
        return ans;
      }
    }
    
    return Success;
  }
  */

  int
  SelidarDriver::reset (unsigned int timeout)
  {
    /*
    int ans;

    {
      boost::mutex::scoped_lock auto_lock (rxtx_lock);

      if (IS_FAIL(ans = sendCommand (ResetReq)))
      {
        return ans;
      }
    }
    */

    return Success;
  }
  
  /*
  int
  SelidarDriver::stop (unsigned int timeout)
  {
    int ans;
    disableDataGrabbing ();

    {
      boost::mutex::scoped_lock auto_lock (rxtx_lock);

      if (IS_FAIL(ans = sendCommand (StopReq)))
      {
        return ans;
      }
    }

    return Success;
  }
  */

  int
  SelidarDriver::stop (unsigned int timeout)
  {
    disableDataGrabbing ();
    
    return Success;
  }
  
  void
  SelidarDriver::disableDataGrabbing ()
  {
    scanning = false;
    cache_thread.join ();
  }
  
  int
  SelidarDriver::waitResponseHeader (SelidarPacketHead* header,
                                     unsigned int timeout)
  {
    int recvPos = 0;
    unsigned int startTs = NS_NaviCommon::getMs ();
    unsigned char recvBuffer[sizeof(SelidarPacketHead)];
    unsigned char *headerBuffer = reinterpret_cast<unsigned char *> (header);
    unsigned int waitTime;
    
    while ((waitTime = NS_NaviCommon::getMs () - startTs) <= timeout)
    {
      size_t remainSize = sizeof(SelidarPacketHead) - recvPos;
      size_t recvSize;
      int ans = rxtx->waitfordata (remainSize, timeout - waitTime, &recvSize);
      if (ans == Serial::ANS_DEV_ERR)
      {
        printf ("device error!\n");
        return Failure;
      }
      else if (ans == Serial::ANS_TIMEOUT)
      {
    	printf ("wait timeout!\n");
        return Timeout;
      }
      
      if (recvSize > remainSize)
        recvSize = remainSize;
      
      if (rxtx->recvdata (recvBuffer, recvSize) <= 0)
      {
    	  printf ("receive data fail!\n");
    	  return Failure;
      }
      
      for (size_t pos = 0; pos < recvSize; ++pos)
      {
        unsigned char currentByte = recvBuffer[pos];

        if (recvPos == 0)
        {
          if (currentByte != SELIDAR_CMD_SYNC_BYTE)
            continue;
        }
        headerBuffer[recvPos++] = currentByte;
        if (recvPos == sizeof(SelidarPacketHead))
        {
          return Success;
        }
      }
    }
    printf ("device timeout!\n");
    return Timeout;
  }
  
  int
  SelidarDriver::getHealth (SelidarHealth & health_info, unsigned int timeout)
  {
    int ans;
    
    if (!isConnected ())
      return Failure;
    
    disableDataGrabbing ();
    
    {
      boost::mutex::scoped_lock auto_lock (rxtx_lock);
      
      if (IS_FAIL(ans = sendCommand (GetHealthReq)))
      {
        return ans;
      }
      
      SelidarPacketHead response_header;
      if (IS_FAIL(ans = waitResponseHeader (&response_header, timeout)))
      {
        return ans;
      }
      
      if (response_header.cmd.cmd_word != GetHealthRep)
      {
        return Invalid;
      }
      
      size_t data_size = sizeof(SelidarHealth) - sizeof(SelidarPacketHead) + 1;
      
      if (rxtx->waitfordata (data_size, timeout) != Serial::ANS_OK)
      {
        return Timeout;
      }
      
      unsigned char health_data[128] = { 0 };
      
      rxtx->recvdata (health_data, data_size);
      
      health_info.head = response_header;
      memcpy (
          reinterpret_cast<unsigned char *> (&health_info)
              + sizeof(SelidarPacketHead),
          health_data, data_size - 1);
      
      unsigned char checksum = 0;
      
      for (size_t i = 0; i < sizeof(SelidarHealth); i++)
      {
        checksum ^= *((unsigned char*) &health_info + i);
      }
      
      if (checksum != health_data[data_size - 1])
      {
        return BadCRC;
      }
      
    }
    
    return Success;
  }
  
  int
  SelidarDriver::getDeviceInfo (SelidarInfo & info, unsigned int timeout)
  {
    int ans;
    
    if (!isConnected ())
      return Failure;
    
    disableDataGrabbing ();
    
    {
      boost::mutex::scoped_lock auto_lock (rxtx_lock);
      
      if (IS_FAIL(ans = sendCommand (GetInfoReq)))
      {
        return ans;
      }
      
      SelidarPacketHead response_header;
      if (IS_FAIL(ans = waitResponseHeader (&response_header, timeout)))
      {
        return ans;
      }
      
      if (response_header.cmd.cmd_word != GetInfoRep)
      {
        return Invalid;
      }
      
      size_t data_size = sizeof(SelidarInfo) - sizeof(SelidarPacketHead) + 1;
      
      if (rxtx->waitfordata (data_size, timeout) != Serial::ANS_OK)
      {
        return Timeout;
      }
      
      unsigned char info_data[128] = { 0 };
      
      rxtx->recvdata (info_data, data_size);
      
      info.head = response_header;
      memcpy (
          reinterpret_cast<unsigned char *> (&info) + sizeof(SelidarPacketHead),
          info_data, data_size - 1);
      
      unsigned char checksum = 0;
      
      for (size_t i = 0; i < sizeof(SelidarInfo); i++)
      {
        checksum ^= *((unsigned char*) &info + i);
      }
      
      if (checksum != info_data[data_size - 1])
      {
        return BadCRC;
      }
      
    }
    
    return Success;
  }
  
  int
  SelidarDriver::cacheScanData ()
  {
    SelidarMeasurementNode local_buf[360];
    size_t count = 360;
    SelidarMeasurementNode local_scan[MAX_SCAN_NODES];
    size_t scan_count = 0;

    size_t cached_count = 0;

    int ans;
    memset (local_scan, 0, sizeof(local_scan));
    
    bool got_start_range = false;

    while (scanning)
    {
      unsigned short range;
      if (IS_FAIL(ans = waitScanData (range, local_buf, count)))
      {
        if (ans != Timeout)
        {
          scanning = false;
          return Timeout;
        }
      }

      boost::mutex::scoped_lock auto_lock (rxtx_lock);

      bool valid_scan = false;
      bool full_range_scan = false;

      /*
      if (range == SELIDAR_START_RANGES)
      {
        cached_scan_node_count = 0;
        got_start_range = true;
      }
      
      if (cached_scan_node_count >= MAX_SCAN_NODES)
      {
    	printf ("Too many nodes to got! enter next loop!\n");
    	cached_scan_node_count = 0;
        continue;
      }
      
      for (int i = 0; i < count; i++)
      {
        cached_scan_node_buf[cached_scan_node_count++] = local_buf[i];
      }
      
      if (range == SELIDAR_MIDDLE_RANGES && !got_start_range)
      {
        cached_scan_node_count = 0;
        data_cond.set();
      }

      if (range == SELIDAR_END_RANGES)
      {
        got_start_range = false;
        printf ("cache: %d\n", cached_scan_node_count);
        data_cond.set ();
      }
      */
      switch (range)
      {
      case SELIDAR_START_RANGES:
    	  cached_count = 0;
    	  got_start_range = true;
    	  valid_scan = true;
    	  break;
      case SELIDAR_MIDDLE_RANGES:
    	  if (got_start_range)
    	  {
    	    valid_scan = true;
    	  }else{
    	    valid_scan = false;
    	    cached_count = 0;
    	  }
    	  break;
      case SELIDAR_END_RANGES:
    	  got_start_range = false;
    	  valid_scan = true;
    	  full_range_scan = true;
    	  break;
      default:
    	  cached_count = 0;
    	  valid_scan = false;
    	  break;
      }

      if (valid_scan && cached_count < MAX_SCAN_NODES)
      {
        for (int i = 0; i < count; i++)
		{
		  cached_scan_node_buf[cached_count++] = local_buf[i];
		}

        if (full_range_scan)
        {
          printf ("cache: %d, --->%d\n", cached_count, cached_scan_node_count);
          cached_scan_node_count = cached_count;
          data_cond.set ();
        }
      }else{
    	cached_count = 0;
    	cached_scan_node_count = cached_count;
        data_cond.set ();
      }

    }
    scanning = false;
    return Success;
  }
  
  int
  SelidarDriver::waitScanData (unsigned short& angle_range,
                               SelidarMeasurementNode* nodes,
                               size_t& node_count, unsigned int timeout)
  {
    int ans;
    
    unsigned char checksum = 0;
    
    // waiting for confirmation
    SelidarPacketHead response_header;

    if (IS_FAIL(ans = waitResponseHeader (&response_header, timeout)))
    {
      printf ("get response header fail!\n");
      return ans;
    }

    if (response_header.cmd.cmd_word != StartScanRep)
    {
      printf ("command word fail!\n");
      return Invalid;
    }
    
    for (size_t i = 0; i < sizeof(SelidarPacketHead); i++)
    {
      checksum ^= *((unsigned char*) &response_header + i);
    }
    
    //discard first packet
    size_t data_size = response_header.length - sizeof(SelidarPacketHead);

    if (rxtx->waitfordata (data_size, timeout) != Serial::ANS_OK)
    {
      printf ("wait data timeout!\n");
      return Timeout;
    }

    unsigned char scan_data[1024] = { 0 };
    
    if (rxtx->recvdata (scan_data, data_size) <= 0)
    {
      printf ("receive data fail!\n");
      return Failure;
    }

    for (size_t i = 0; i < data_size - 1; i++)
    {
      checksum ^= scan_data[i];
    }
    
    if (checksum != scan_data[data_size - 1])
    {
      printf ("bad data crc!\n");
      return BadCRC;
    }
    
    int data_pos = 0;
    
    memcpy (&angle_range, scan_data, sizeof(angle_range));
    data_pos += sizeof(angle_range);

    node_count = (data_size - 2 - 2 - 1) / 2;
    
    unsigned short start_angle;
    memcpy (&start_angle, scan_data + data_pos, sizeof(start_angle));
    data_pos += sizeof(start_angle);
    
    for (size_t i = 0; i < node_count; i++)
    {
      unsigned short distance;
      
      memcpy (&distance, scan_data + data_pos, sizeof(distance));
      data_pos += sizeof(distance);
      
      nodes[i].angle_scale_100 = start_angle + (i * angle_range) / node_count;
      nodes[i].distance_scale_1000 = distance;
    }
    
    return Success;
  }
  
  /*
   int
   SelidarDriver::startScan (unsigned int timeout)
   {
   int ans;
   if (!connected)
   return Failure;
   if (scanning)
   return Denied;
   
   stop ();
   
   // have to slow down the speed of sending cmd, otherwise next cmd will be discard by radar
   NS_NaviCommon::delay (100);
   
   {
   boost::mutex::scoped_lock auto_lock (rxtx_lock);
   
   if (IS_FAIL(ans = sendCommand (StartScanReq)))
   {
   return ans;
   }
   
   scanning = true;
   cache_thread = boost::thread (
   boost::bind (&SelidarDriver::cacheScanData, this));
   }
   
   return Success;
   }
   */

  int
  SelidarDriver::startScan (unsigned int timeout)
  {
    int ans;
    if (!connected)
      return Failure;
    if (scanning)
      return Denied;
    
    stop ();

    {
      boost::mutex::scoped_lock auto_lock (rxtx_lock);
      
      scanning = true;
      cache_thread = boost::thread (
          boost::bind (&SelidarDriver::cacheScanData, this));
    }
    
    return Success;
  }
  
  int
  SelidarDriver::grabScanData (SelidarMeasurementNode * nodebuffer,
                               size_t & count, unsigned int timeout)
  {
    switch (data_cond.wait (timeout / 1000))
    {
      case NS_NaviCommon::Condition::COND_TIMEOUT:
        count = 0;
        return Timeout;
      case NS_NaviCommon::Condition::COND_OK:
      {
        if (cached_scan_node_count == 0)
          return Timeout; //consider as timeout
          
        boost::mutex::scoped_lock auto_lock (rxtx_lock);
        
        printf ("grab: %d\n", cached_scan_node_count);

        size_t size_to_copy = min(count, cached_scan_node_count);
        
        memcpy (nodebuffer, cached_scan_node_buf,
                size_to_copy * sizeof(SelidarMeasurementNode));
        count = size_to_copy;
        cached_scan_node_count = 0;
      }
        return Success;
        
      default:
        count = 0;
        return Failure;
    }
  }

}
