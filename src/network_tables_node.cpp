#include "ros/ros.h"
#include "std_msgs/String.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

#include "network_tables_node/NTGetBool.h"
#include "network_tables_node/NTGetBoolArray.h"
#include "network_tables_node/NTGetDouble.h"
#include "network_tables_node/NTGetDoubleArray.h"
#include "network_tables_node/NTGetString.h"
#include "network_tables_node/NTGetStringArray.h"
#include "network_tables_node/NTGetRaw.h"

#include "network_tables_node/NTSetBool.h"
#include "network_tables_node/NTSetBoolArray.h"
#include "network_tables_node/NTSetDouble.h"
#include "network_tables_node/NTSetDoubleArray.h"
#include "network_tables_node/NTSetString.h"
#include "network_tables_node/NTSetStringArray.h"
#include "network_tables_node/NTSetRaw.h"

#include <thread>
#include <string>
#include <mutex>
#include <vector>

ros::NodeHandle* node;

nt::NetworkTableInstance networkTableInst;

bool nt_getbool(network_tables_node::NTGetBool::Request &request, network_tables_node::NTGetBool::Response &response)
{
	response.output = networkTableInst.GetTable(request.table_name)->GetEntry(request.entry_name).GetBoolean(request.default_value);
	return true;
}

bool nt_getboolarray(network_tables_node::NTGetBoolArray::Request &request, network_tables_node::NTGetBoolArray::Response &response)
{
	std::vector<int> defaultBoolVals;
	for (size_t i = 0; i < request.default_value.size(); i++)
	{
		defaultBoolVals.push_back(request.default_value[i]);
	}
	std::vector<int> resp(networkTableInst.GetTable(request.table_name)->GetEntry(request.entry_name).GetBooleanArray(defaultBoolVals));
	for (size_t i = 0; i < resp.size(); i++)
	{
		response.output.push_back(resp[i]);
	}
	return true;
}

bool nt_getdouble(network_tables_node::NTGetDouble::Request &request, network_tables_node::NTGetDouble::Response &response)
{
	response.output = networkTableInst.GetTable(request.table_name)->GetEntry(request.entry_name).GetDouble(request.default_value);
	return true;
}

bool nt_getdoublearray(network_tables_node::NTGetDoubleArray::Request &request, network_tables_node::NTGetDoubleArray::Response &response)
{
	response.output = networkTableInst.GetTable(request.table_name)->GetEntry(request.entry_name).GetDoubleArray(request.default_value);
	return true;
}

bool nt_getstring(network_tables_node::NTGetString::Request &request, network_tables_node::NTGetString::Response &response)
{
	response.output = networkTableInst.GetTable(request.table_name)->GetEntry(request.entry_name).GetString(request.default_value);
	return true;
}

bool nt_getstringarray(network_tables_node::NTGetStringArray::Request &request, network_tables_node::NTGetStringArray::Response &response)
{
	response.output = networkTableInst.GetTable(request.table_name)->GetEntry(request.entry_name).GetStringArray(request.default_value);
	return true;
}

bool nt_getraw(network_tables_node::NTGetRaw::Request &request, network_tables_node::NTGetRaw::Response &response)
{
	response.output = networkTableInst.GetTable(request.table_name)->GetEntry(request.entry_name).GetRaw(request.default_value);
	return true;
}




bool nt_setbool(network_tables_node::NTSetBool::Request &request, network_tables_node::NTSetBool::Response &response)
{
	response.success = networkTableInst.GetTable(request.table_name)->GetEntry(request.entry_name).SetBoolean(request.value);
	return response.success;
}

bool nt_setboolarray(network_tables_node::NTSetBoolArray::Request &request, network_tables_node::NTSetBoolArray::Response &response)
{
	std::vector<int> boolVals;
	for (size_t i = 0; i < request.value.size(); i++)
	{
		boolVals.push_back(request.value[i]);
	}
	response.success = networkTableInst.GetTable(request.table_name)->GetEntry(request.entry_name).SetBooleanArray(boolVals);
	return response.success;
}

bool nt_setdouble(network_tables_node::NTSetDouble::Request &request, network_tables_node::NTSetDouble::Response &response)
{
	response.success = networkTableInst.GetTable(request.table_name)->GetEntry(request.entry_name).SetDouble(request.value);
	return response.success;
}

bool nt_setdoublearray(network_tables_node::NTSetDoubleArray::Request &request, network_tables_node::NTSetDoubleArray::Response &response)
{
	response.success = networkTableInst.GetTable(request.table_name)->GetEntry(request.entry_name).SetDoubleArray(request.value);
	return response.success;
}

bool nt_setstring(network_tables_node::NTSetString::Request &request, network_tables_node::NTSetString::Response &response)
{
	response.success = networkTableInst.GetTable(request.table_name)->GetEntry(request.entry_name).SetString(request.value);
	return response.success;
}

bool nt_setstringarray(network_tables_node::NTSetStringArray::Request &request, network_tables_node::NTSetStringArray::Response &response)
{
	response.success = networkTableInst.GetTable(request.table_name)->GetEntry(request.entry_name).SetStringArray(request.value);
	return response.success;
}

bool nt_setraw(network_tables_node::NTSetRaw::Request &request, network_tables_node::NTSetRaw::Response &response)
{
	response.success = networkTableInst.GetTable(request.table_name)->GetEntry(request.entry_name).SetRaw(request.value);
	return response.success;
}





int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "network_tables_node");

	ros::NodeHandle n;

	node = &n;

	ros::ServiceServer service_getbool = node->advertiseService("nt_getbool", nt_getbool);
	ros::ServiceServer service_getboolarray = node->advertiseService("nt_getboolarray", nt_getboolarray);
	ros::ServiceServer service_getdouble = node->advertiseService("nt_getdouble", nt_getdouble);
	ros::ServiceServer service_getdoublearray = node->advertiseService("nt_getdoublearray", nt_getdoublearray);
	ros::ServiceServer service_getstring = node->advertiseService("nt_getstring", nt_getstring);
	ros::ServiceServer service_getstringarray = node->advertiseService("nt_getstringarray", nt_getstringarray);
	ros::ServiceServer service_getraw = node->advertiseService("nt_getraw", nt_getraw);

	ros::ServiceServer service_setbool = node->advertiseService("nt_setbool", nt_setbool);
	ros::ServiceServer service_setboolarray = node->advertiseService("nt_setboolarray", nt_setboolarray);
	ros::ServiceServer service_setdouble = node->advertiseService("nt_setdouble", nt_setdouble);
	ros::ServiceServer service_setdoublearray = node->advertiseService("nt_setdoublearray", nt_setdoublearray);
	ros::ServiceServer service_setstring = node->advertiseService("nt_setstring", nt_setstring);
	ros::ServiceServer service_setstringarray = node->advertiseService("nt_setstringarray", nt_setstringarray);
	ros::ServiceServer service_setraw = node->advertiseService("nt_setraw", nt_setraw);

	networkTableInst = nt::NetworkTableInstance::GetDefault();
	ros::spin();
	return 0;
}