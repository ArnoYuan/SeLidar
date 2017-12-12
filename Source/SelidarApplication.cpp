/*
 * SelidarApplication.cpp
 *
 *  Created on: 2016年10月12日
 *      Author: lichq
 */

#include "SelidarApplication.h"
#include <Console/Console.h>
//for debugging
#include <assert.h>
#include <Time/Utils.h>
#include <Parameter/Parameter.h>

namespace NS_Selidar
{

#define DEG2RAD(x) ((x)*M_PI/180.)
#define RAD2DEG(x) ((x)*180./M_PI)

#ifndef _countof
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#endif

  SelidarApplication::SelidarApplication()
  {
    publisher = new NS_DataSet::Publisher< NS_DataType::LaserScan >(
        "LASER_SCAN");
    serial_baudrate = 115200;
    frame_id = "laser_frame";
    scan_count = 0;
    scan_timeout = 3;
    inverted = false;
    angle_compensate = true;
  }

  SelidarApplication::~SelidarApplication()
  {
    scan_count = 0;

    running = false;
    scan_thread.join();

    delete publisher;
  }

  void SelidarApplication::loadParameters()
  {
    NS_NaviCommon::Parameter parameter;
    parameter.loadConfigurationFile("selidar.xml");
    serial_port = parameter.getParameter("serial_port", "/dev/ttyUSB0");
    serial_baudrate = parameter.getParameter("serial_baudrate", 115200);
    frame_id = parameter.getParameter("frame_id", "laser_frame");

    if(parameter.getParameter("inverted", 0) == 1)
    {
      inverted = true;
    }
    else
    {
      inverted = false;
    }

    if(parameter.getParameter("compensate", 1)==1)
    {
    	angle_compensate = true;
    }
    else
    {
    	angle_compensate = false;
    }
  }

#ifdef DUPLEX_MODE
  bool
  SelidarApplication::checkSelidarHealth (SelidarDriver * drv)
  {
    int op_result;
    SelidarHealth healthinfo;

    op_result = drv->getHealth (healthinfo);

    if (IS_OK(op_result))
    {
      console.debug ("Selidar health status : %d, errcode: %d",
          healthinfo.status, healthinfo.err_code);

      if (healthinfo.status != StatusFine)
      {
        console.warning ("Selidar's status is not fine! ");
        return false;
      }
      else
      {
        console.message ("Selidar's status is not fine! ");
        return true;
      }

    }
    else
    {
      return false;
    }
  }

  bool
  SelidarApplication::checkSelidarInfo (SelidarDriver * drv)
  {
    int op_result;
    SelidarInfo device_info;

    op_result = drv->getDeviceInfo (device_info);

    if (IS_OK(op_result))
    {
      console.debug ("Selidar device info :");
      console.debug ("\t model : %d ", device_info.model);
      console.debug ("\t hw ver : %d ", device_info.hw_id);
      console.debug ("\t fw ver : %d.%d ", device_info.fw_major,
          device_info.fw_minor);
      return true;
    }
    else
    {
      return false;
    }

  }

  bool
  SelidarApplication::stopScanService (NS_ServiceType::RequestBase* request,
      NS_ServiceType::ResponseBase* response)
  {
    if (!drv.isConnected ())
    return false;

    drv.stop ();

    return true;
  }

  bool
  SelidarApplication::startScanService (NS_ServiceType::RequestBase* request,
      NS_ServiceType::ResponseBase* response)
  {
    if (!drv.isConnected ())
    return false;

    console.message ("Start motor");

    drv.startScan ();
    return true;
  }
#endif

  void SelidarApplication::publishScan(SelidarMeasurementNode *nodes,
                                       size_t node_count,
                                       NS_NaviCommon::Time start,
                                       double scan_time, float angle_min,
                                       float angle_max)
  {
    NS_DataType::LaserScan scan_msg;

    scan_msg.header.stamp = start;
    scan_msg.header.frame_id = frame_id;

    scan_count_lock.lock();
    scan_count++;
    if(scan_count == 1)
    {
      console.debug("Got first scan data!");
      got_first_scan_cond.notify_one();
    }
    scan_count_lock.unlock();

    bool reversed = (angle_max > angle_min);
    if(reversed)
    {
      scan_msg.angle_min = M_PI - angle_max;
      scan_msg.angle_max = M_PI - angle_min;
    }
    else
    {
      scan_msg.angle_min = M_PI - angle_min;
      scan_msg.angle_max = M_PI - angle_max;
    }

    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count - 1);

    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / (double)(node_count - 1);
    scan_msg.range_min = 0.15f;
    scan_msg.range_max = 8.0f;

    scan_msg.intensities.resize(node_count);
    scan_msg.ranges.resize(node_count);

    bool reverse_data = (!inverted && reversed) || (inverted && !reversed);

    if(!reverse_data)
    {
      for(size_t i = 0; i < node_count; i++)
      {
        float read_value = (float)nodes[i].distance_scale_1000 / 1000.0f;
        if(read_value == 0.0)
          scan_msg.ranges[i] = std::numeric_limits< float >::infinity();
        else
          scan_msg.ranges[i] = read_value;
      }
    }
    else
    {
      for(size_t i = 0; i < node_count; i++)
      {
        float read_value = (float)nodes[i].distance_scale_1000 / 1000.0f;
        if(read_value == 0.0)
          scan_msg.ranges[node_count - 1 - i] = std::numeric_limits< float >::infinity();
        else
          scan_msg.ranges[node_count - 1 - i] = read_value;
      }
    }

    /*
     printf ("max: %f, min: %f, increase: %f.\n", scan_msg.angle_max, scan_msg.angle_min, scan_msg.angle_increment);
     printf ("node count: %d\n", node_count);
     for (size_t i = 0; i < node_count; i++)
     {
     printf ("%f,", scan_msg.ranges[i]);
     }
     printf ("\n");
     */

    publisher->publish(scan_msg);
  }

  void SelidarApplication::scanLoop()
  {
    int op_result;
    drv.startScan();
    NS_NaviCommon::Time start_scan_time;
    NS_NaviCommon::Time end_scan_time;
    double scan_duration;

    const int buffer_size = 360 * 10;
    const int pub_nodes_count = 720;
    const float delta_angle = 0.5f;
    SelidarMeasurementNode nodes_pub[pub_nodes_count];
    size_t buffered_nodes = 0;
    memset(nodes_pub, 0, sizeof(nodes_pub));

	while (running) {
		SelidarMeasurementNode nodes[buffer_size];
		size_t count = _countof(nodes);
		start_scan_time = NS_NaviCommon::Time::now();
		op_result = drv.grabScanData(nodes, count);
		end_scan_time = NS_NaviCommon::Time::now();
		scan_duration = (end_scan_time - start_scan_time).toSec() * 1e-3;

		if (op_result == Success) {
			float angle_min = 0;
			float angle_max = 0;

			drv->acsendScanData(nodes, count);

			if (angle_compensate) {
				angle_min = DEG2RAD(0.0f);
				angle_max = DEG2RAD(359.0f);
				memset(nodes_pub, 0, sizeof(nodes_pub));
				int i = 0;
				for (; i < count; i++) {
					if (nodes[i].distance_scale_1000 != 0) {
						float angle =
								(float) (nodes[i].angle_scale_100 / 100.0f);
						int pos = (int) (angle / delta_angle);
						float angle_pre = angle - pos * delta_angle;
						float angle_next = (pos + 1) * delta_angle - angle;
						if (angle_pre < angle_next) {
							if (pos < pub_nodes_count) {
								nodes_pub[pos] = nodes[i];
							} else {
								if (pos < pub_nodes_count - 1) {
									nodes_pub[pos + 1] = nodes[i];
								}
							}
						}
					}
				}
				publishScan(nodes_pub, pub_nodes_count, start_scan_time,
						scan_duration, angle_min, angle_max);
			} else {
				int start_node = 0;
				int end_node = 0;
				int i = 0;
				int pub_nodes_count = 0;
				while (nodes[i++].distance_scale_1000 != 0 && i < count)
					;
				start_node = i - 1;
				angle_min =
						(float) (nodes[start_node].angle_scale_100 / 100.0f);
				angle_max = (float) (nodes[end_node - 1].angle_scale_100
						/ 100.0f);
				pub_nodes_count = end_node - start_node;
				memcpy(nodes_pub, &nodes[start_node],
						pub_nodes_count * sizeof(SelidarMeasurementNode));
				angle_min = DEG2RAD(angle_min);
				angle_max = DEG2RAD(angle_max);
				publishScan(&scan_pub, nodes_pub, start_scan_time,
						scan_duration, angle_min, angle_max);

			}

		}
	}
  }

  void SelidarApplication::run()
  {
    console.message("selidar is running!");

    loadParameters();

    // make connection...
    if(IS_FAIL(
        drv.connect(serial_port.c_str(), (unsigned int )serial_baudrate, 0)))
    {
      console.error("cannot bind to the specified serial port %s.",
                    serial_port.c_str());

      return;
    }

#ifdef DUPLEX_MODE
    // reset lidar
    drv.reset ();
    NS_NaviCommon::delay (5000);

    // check health...
    if (!checkSelidarHealth (&drv))
    {
      return;
    }

    NS_NaviCommon::delay (100);

    // get device info...
    if (!checkSelidarInfo (&drv))
    {
      return;
    }

#endif

    running = true;

    scan_thread = boost::thread(
        boost::bind(&SelidarApplication::scanLoop, this));

    int wait_times = 0;
    scan_count_lock.lock();
    while(wait_times++ <= scan_timeout && scan_count == 0)
    {
      got_first_scan_cond.timed_wait(
          scan_count_lock,
          (boost::get_system_time() + boost::posix_time::seconds(1)));
    }
    scan_count_lock.unlock();

    if(scan_count == 0)
    {
      console.error("Can't got first scan from LIDAR.");
      running = false;
    }

    console.debug("Got scan data from lidar!");

  }

  void SelidarApplication::quit()
  {
    console.message("selidar is quitting!");

#ifdef DUPLEX_MODE
    drv.stop ();
#endif

    running = false;

    scan_thread.join();

    drv.disconnect();
  }

}

