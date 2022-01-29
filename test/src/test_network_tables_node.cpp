#include "test_network_tables_node.hpp"
#include "network_tables_node.hpp"
#include "network_tables_node/NTGetDouble.h"
#include "ros/ros.h"

#include <gtest/gtest.h>

ros::NodeHandle* node;

TEST(SampleTest, Test_Test)
{
    ASSERT_TRUE(1);
    ASSERT_FALSE(0);
}

TEST(TestNTService, TestNTGetDouble)
{
    

    //Usage example:
    // ros::ServiceClient client = node->serviceClient<network_tables_node::NTGetDouble>("nt_getdouble", true);
    // if (client)
    // {
    //     network_tables_node::NTGetDouble tx;
    //     tx.request.table_name = "limelight";
    //     tx.request.entry_name = "tx";
    //     tx.request.default_value = 0.1;
    //     if (client.call(tx))
    //     {
    //         ASSERT_EQ(tx.response.output, 0.1);
    //     }
    //     else
    //     {
    //         ROS_ERROR("Failed to run NT service");
    //         ASSERT_TRUE(0); //Fail test
    //     }
    // }
    // else
    // {
    //     //client = node->serviceClient<network_tables_node::NTGetDouble>("nt_getdouble", true);
    //     ROS_ERROR("Failed to connect to NT service");
    //     ASSERT_TRUE(0); //Fail test
    // }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_network_tables_node");
	ros::NodeHandle n;
	node = &n;
    return RUN_ALL_TESTS();
}