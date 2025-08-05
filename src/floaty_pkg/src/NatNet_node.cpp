/* 
Copyright © 2012 NaturalPoint Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. */


/*

SampleClient.cpp

This program connects to a NatNet server, receives a data stream, and writes that data stream
to an ascii file.  The purpose is to illustrate using the NatNetClient class.

Usage [optional]:

	SampleClient [ServerIP] [LocalIP] [OutputFilename]

	[ServerIP]			IP address of the server (e.g. 192.168.0.107) ( defaults to local machine)
	[OutputFilename]	Name of points file (pts) to write out.  defaults to Client-output.pts

*/

#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#   include <conio.h>
#else
#   include <unistd.h>
#   include <termios.h>
#endif

#include <vector>

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>

#include <chrono>
#include <thread>


void NATNET_CALLCONV ServerDiscoveredCallback( const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext );
void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);    // receives data from the server
void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg);      // receives NatNet error messages
void resetClient();
int ConnectClient();

static const ConnectionType kDefaultConnectionType = ConnectionType_Multicast;

NatNetClient* g_pClient = NULL;
// FILE* g_outputFile;

std::vector< sNatNetDiscoveredServer > g_discoveredServers;
sNatNetClientConnectParams g_connectParams;
char g_discoveredMulticastGroupAddr[kNatNetIpv4AddrStrLenMax] = NATNET_DEFAULT_MULTICAST_ADDRESS;
int g_analogSamplesPerMocapFrame = 0;
sServerDescription g_serverDescription;
std::string data_topic;
float fRate;
ros::Publisher pose_pub;
int num_of_bodies;
std::string bodies_ids[10] = {"", "", "", "", "", "", "", "", "", ""};


int main( int argc, char* argv[] )
{

    ros::init(argc, argv, "NatNet_node");
    ros::NodeHandle nh;

    nh.param<std::string>("/NatNet/data_topic", data_topic, "/Optitrack/Floaty/");
    nh.param<float>("/NatNet/default_rate", fRate, 200);


    // Install logging callback
    NatNet_SetLogCallback( MessageHandler );

    // create NatNet client
    g_pClient = new NatNetClient();

    // set the frame callback handler
    g_pClient->SetFrameReceivedCallback( DataHandler, g_pClient );	// this function will receive data from the server

    // If no arguments were specified on the command line,
    // attempt to discover servers on the local network.

    NatNetDiscoveryHandle discovery;
    NatNet_CreateAsyncServerDiscovery( &discovery, ServerDiscoveredCallback );

    // Wait for 50 ms, as 2 ms didn't work everytime
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    const sNatNetDiscoveredServer& discoveredServer = g_discoveredServers[0];
    snprintf(
        g_discoveredMulticastGroupAddr, sizeof g_discoveredMulticastGroupAddr,
        "%" PRIu8 ".%" PRIu8".%" PRIu8".%" PRIu8"",
        discoveredServer.serverDescription.ConnectionMulticastAddress[0],
        discoveredServer.serverDescription.ConnectionMulticastAddress[1],
        discoveredServer.serverDescription.ConnectionMulticastAddress[2],
        discoveredServer.serverDescription.ConnectionMulticastAddress[3]
    );

    g_connectParams.connectionType = discoveredServer.serverDescription.ConnectionMulticast ? ConnectionType_Multicast : ConnectionType_Unicast;
    g_connectParams.serverCommandPort = discoveredServer.serverCommandPort;
    g_connectParams.serverDataPort = discoveredServer.serverDescription.ConnectionDataPort;
    g_connectParams.serverAddress = discoveredServer.serverAddress;
    g_connectParams.localAddress = discoveredServer.localAddress;
    g_connectParams.multicastAddress = g_discoveredMulticastGroupAddr;


    NatNet_FreeAsyncServerDiscovery( discovery );

    int iResult;

    // Connect to Motive
    iResult = ConnectClient();

	sDataDescriptions* pDataDefs = NULL;
	iResult = g_pClient->GetDataDescriptionList(&pDataDefs);

    // Get the number of the published bodies
    num_of_bodies = pDataDefs->nDataDescriptions;
    for(int k=0; k < num_of_bodies; k++){
        sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[k].Data.RigidBodyDescription;
        bodies_ids[k] = pRB->szName;
    }


	// Ready to receive marker stream!


    pose_pub = nh.advertise<geometry_msgs::PoseStamped>(data_topic, fRate);
    // pose_pub = nh.advertise<geometry_msgs::Pose>(data_topic, fRate);
	printf("\nClient is connected to server and listening for data...\n");
	bool bExit = false;
	// while ( const int c = getch() )
	while (true)
	{
		if(bExit)
			break;
	}

	// Done - clean up.
	if (g_pClient)
	{
		g_pClient->Disconnect();
		delete g_pClient;
		g_pClient = NULL;
	}


    ros::waitForShutdown();

    return 0;
}


void NATNET_CALLCONV ServerDiscoveredCallback( const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext )
{
    char serverHotkey = '.';
    if ( g_discoveredServers.size() < 9 )
    {
        serverHotkey = static_cast<char>('1' + g_discoveredServers.size());
    }

    printf( "[%c] %s %d.%d at %s ",
        serverHotkey,
        pDiscoveredServer->serverDescription.szHostApp,
        pDiscoveredServer->serverDescription.HostAppVersion[0],
        pDiscoveredServer->serverDescription.HostAppVersion[1],
        pDiscoveredServer->serverAddress );

    if ( pDiscoveredServer->serverDescription.bConnectionInfoValid )
    {
        printf( "(%s)\n", pDiscoveredServer->serverDescription.ConnectionMulticast ? "multicast" : "unicast" );
    }
    else
    {
        printf( "(WARNING: Legacy server, could not autodetect settings. Auto-connect may not work reliably.)\n" );
    }

    g_discoveredServers.push_back( *pDiscoveredServer );
    printf("Finished servers writing\n");
}

// Establish a NatNet Client connection
int ConnectClient()
{
    // Release previous server
    g_pClient->Disconnect();

    // Init Client and connect to NatNet server
    int retCode = g_pClient->Connect( g_connectParams );


    void* pResult;
    int nBytes = 0;
    g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
    fRate = *((float*)pResult);
 
    return ErrorCode_OK;
}

// DataHandler receives data from the server
// This function is called by NatNet when a frame of mocap data is available
void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData)
{
    NatNetClient* pClient = (NatNetClient*) pUserData;


    int i=0;


    // timecode - for systems with an eSync and SMPTE timecode generator - decode to values
	int hour, minute, second, frame, subframe;
    NatNet_DecodeTimecode( data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe );
	// decode to friendly string
	char szTimecode[128] = "";
    NatNet_TimecodeStringify( data->Timecode, data->TimecodeSubframe, szTimecode, 128 );
	// printf("Timecode : %s\n", szTimecode);

	// Rigid Bodies
	// printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
	for(i=0; i < data->nRigidBodies; i++)
	{
        // params
        // 0x01 : bool, rigid body was successfully tracked in this frame
        bool bTrackingValid = data->RigidBodies[i].params & 0x01;

        geometry_msgs::PoseStamped pose_msg;

        // Set frame id using the name published on the network
        int body_id = (int)data->RigidBodies[i].ID;
        pose_msg.header.frame_id = bodies_ids[body_id-1];

        // Set pose
        pose_msg.pose.position.x = data->RigidBodies[i].x;
        pose_msg.pose.position.y = data->RigidBodies[i].y;
        pose_msg.pose.position.z = data->RigidBodies[i].z;
        pose_msg.pose.orientation.x = data->RigidBodies[i].qx;
        pose_msg.pose.orientation.y = data->RigidBodies[i].qy;
        pose_msg.pose.orientation.z = data->RigidBodies[i].qz;
        pose_msg.pose.orientation.w = data->RigidBodies[i].qw;


        // geometry_msgs::Pose pose_msg;

        // // Set frame id using the name published on the network
        // // int body_id = (int)data->RigidBodies[i].ID;
        // // pose_msg.header.frame_id = bodies_ids[body_id-1];

        // // Set pose
        // pose_msg.position.x = data->RigidBodies[i].x;
        // pose_msg.position.y = data->RigidBodies[i].y;
        // pose_msg.position.z = data->RigidBodies[i].z;
        // pose_msg.orientation.x = data->RigidBodies[i].qx;
        // pose_msg.orientation.y = data->RigidBodies[i].qy;
        // pose_msg.orientation.z = data->RigidBodies[i].qz;
        // pose_msg.orientation.w = data->RigidBodies[i].qw;

        pose_pub.publish(pose_msg);

	}

}


// MessageHandler receives NatNet error/debug messages
void NATNET_CALLCONV MessageHandler( Verbosity msgType, const char* msg )
{
    // Optional: Filter out debug messages
    if ( msgType < Verbosity_Info )
    {
        return;
    }

    printf( "\n[NatNetLib]" );

    switch ( msgType )
    {
        case Verbosity_Debug:
            printf( " [DEBUG]" );
            break;
        case Verbosity_Info:
            printf( "  [INFO]" );
            break;
        case Verbosity_Warning:
            printf( "  [WARN]" );
            break;
        case Verbosity_Error:
            printf( " [ERROR]" );
            break;
        default:
            printf( " [?????]" );
            break;
    }

    printf( ": %s\n", msg );
}


void resetClient()
{
	int iSuccess;

	printf("\n\nre-setting Client\n\n.");

	iSuccess = g_pClient->Disconnect();
	if(iSuccess != 0)
		printf("error un-initting Client\n");

    iSuccess = g_pClient->Connect( g_connectParams );
	if(iSuccess != 0)
		printf("error re-initting Client\n");
}

