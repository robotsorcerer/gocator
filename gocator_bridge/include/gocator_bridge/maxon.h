#ifndef _MAXON_H_
#define _MAXON_H_

#ifndef MMC_SUCCESS
	#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
	#define MMC_FAILED 1
#endif

#include <gocator_bridge/Definitions.h>
#include <sstream>
#include <list>

using namespace std;

typedef void* HANDLE;
typedef int BOOL;

unsigned int ulErrorCode = 0;
int lResult = MMC_FAILED;

class motor_motion
{
public:
	motor_motion()
	: g_pKeyHandle(0), g_usNodeId(1), g_baudrate(0), 
		home_accel(1000), speed_switch(8000), speed_index(1000), 
		home_offset(0), home_pose(0), current_thresh(2000)
	{
		ROS_INFO("Motor Started");
	}

	~motor_motion()
	{
		ROS_INFO("Motor object destroyed. Motor Stopped");
	}

	bool SetDefaultParameters()
	{
		//USB
		g_usNodeId = 1;
		g_deviceName = "EPOS2"; //EPOS
		g_protocolStackName = "MAXON SERIAL V2"; //MAXON_RS232
		g_interfaceName = "USB"; //RS232
		g_portName = "USB0"; // /dev/ttyS1
		g_baudrate = 1000000; //115200
		return true;
	}

	int OpenDevice(unsigned int* p_pErrorCode)
	{
		int lResult = MMC_FAILED;

		char* pDeviceName = new char[255];
		char* pProtocolStackName = new char[255];
		char* pInterfaceName = new char[255];
		char* pPortName = new char[255];

		strcpy(pDeviceName, g_deviceName.c_str());
		strcpy(pProtocolStackName, g_protocolStackName.c_str());
		strcpy(pInterfaceName, g_interfaceName.c_str());
		strcpy(pPortName, g_portName.c_str());

		ROS_INFO("Open device...");

		g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

		if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
		{
			unsigned int lBaudrate = 0;
			unsigned int lTimeout = 0;

			if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
			{
				if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=0)
				{
					if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
					{
						if(g_baudrate==(int)lBaudrate)
						{
							lResult = MMC_SUCCESS;
						}
					}
				}
			}
		}
		else
		{
			g_pKeyHandle = 0;
		}

		delete []pDeviceName;
		delete []pProtocolStackName;
		delete []pInterfaceName;
		delete []pPortName;

		return lResult;
	}

	int close_device(unsigned int* p_pErrorCode)
	{
		int lResult = MMC_FAILED;

		*p_pErrorCode = 0;

		ROS_INFO("Closing device");

		if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0)
		{
			lResult = MMC_SUCCESS;
		}
		return lResult;
	}

	bool down_velocity(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
	{
		int lResult = MMC_SUCCESS;
		stringstream msg;

		msg << "set profile velocity mode, node = " << p_usNodeId;

		ROS_INFO_STREAM(msg.str());

		if(VCS_ActivateProfileVelocityMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
		{
			ROS_ERROR_STREAM("VCS_ActivateProfileVelocityMode " << lResult <<" " << p_rlErrorCode);
			lResult = MMC_FAILED;
		}
		else
		{
			list<long> velocityList;

			velocityList.push_back(1500);
			velocityList.push_back(1500);			
			velocityList.push_back(1500);
			velocityList.push_back(1500);

			for(list<long>::iterator it = velocityList.begin(); it !=velocityList.end(); it++)
			{
				long targetvelocity = (*it);

				stringstream msg;
				msg << "moving to end position w/target velocity = " << targetvelocity << " rpm, node = " << p_usNodeId;
				ROS_INFO_STREAM(msg.str());

				if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, targetvelocity, &p_rlErrorCode) == 0)
				{
					lResult = MMC_FAILED;
					ROS_ERROR_STREAM("VCS_MoveWithVelocity " << lResult << ", " << p_rlErrorCode);
					break;
				}
				sleep(2);
			}

			if(lResult == MMC_SUCCESS)
			{
				ROS_INFO("halt velocity movement");

				if(VCS_HaltVelocityMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
				{
					lResult = MMC_FAILED;
					ROS_ERROR_STREAM("VCS_HaltVelocityMovement " << ", " << lResult <<", " << p_rlErrorCode);
				}
			}
		}		
		return lResult;
	}

	bool up_velocity(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
	{
		int lResult = MMC_SUCCESS;
		stringstream msg;

		msg << "set profile velocity mode, node = " << p_usNodeId;

		ROS_INFO_STREAM(msg.str());

		if(VCS_ActivateProfileVelocityMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
		{
			ROS_ERROR_STREAM("VCS_ActivateProfileVelocityMode " << lResult <<" " << p_rlErrorCode);
			lResult = MMC_FAILED;
		}
		else
		{
			list<long> velocityList;

			velocityList.push_back(-1500);
			velocityList.push_back(-1500);
			velocityList.push_back(-1500);
			velocityList.push_back(-1500);

			for(list<long>::iterator it = velocityList.begin(); it !=velocityList.end(); it++)
			{
				long targetvelocity = (*it);

				stringstream msg;
				msg << "moving to end position w/target velocity = " << targetvelocity << " rpm, node = " << p_usNodeId;
				ROS_INFO_STREAM(msg.str());

				if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, targetvelocity, &p_rlErrorCode) == 0)
				{
					lResult = MMC_FAILED;
					ROS_ERROR_STREAM("VCS_MoveWithVelocity " << lResult << ", " << p_rlErrorCode);
					break;
				}

				sleep(2);
			}

			if(lResult == MMC_SUCCESS)
			{
				ROS_INFO("halt velocity movement");

				if(VCS_HaltVelocityMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
				{
					lResult = MMC_FAILED;
					ROS_ERROR_STREAM("VCS_HaltVelocityMovement " << ", " << lResult <<", " << p_rlErrorCode);
				}
			}
		}		
		return lResult;
	}
	
	bool go_home(unsigned int* p_pErrorCode)
	{
			int lResult;
			int pHomingAttained, pHomingError;

			if(VCS_ActivateHomingMode(g_pKeyHandle, g_usNodeId, p_pErrorCode)!=0)
			{
				lResult = MMC_SUCCESS;
				ROS_INFO_STREAM("Successfully activated homing mode" << lResult << ", " << *p_pErrorCode);
			}

			if(VCS_SetHomingParameter(g_pKeyHandle, g_usNodeId, home_accel, speed_switch,\
									speed_index, home_offset, current_thresh, home_pose, p_pErrorCode)!=0)
			{
				lResult = MMC_SUCCESS;
				ROS_INFO_STREAM("Successfully set homing params " << lResult << ", " << *p_pErrorCode);
			}
			
			signed char homing_method = HM_CURRENT_THRESHOLD_NEGATIVE_SPEED;
			
			if(VCS_FindHome(g_pKeyHandle, g_usNodeId, homing_method, p_pErrorCode)!=0)
			{

				lResult = MMC_SUCCESS;
				ROS_INFO_STREAM("Successfully found home position " << lResult << ", " << *p_pErrorCode);
			}

			if(VCS_GetHomingState(g_pKeyHandle, g_usNodeId, &pHomingAttained, &pHomingError, p_pErrorCode)!=0)
			{

				lResult = MMC_SUCCESS;
		 		ROS_INFO_STREAM("Successfully retrieved homing state " << lResult << ", " << *p_pErrorCode);	
			}

			int timeout = 3000; //wait for 3 ms to reach home position
			if(VCS_WaitForHomingAttained(g_pKeyHandle, g_usNodeId, timeout, p_pErrorCode)!=0)
			{
				ROS_INFO_STREAM("Waiting to attain home position");
			}

		return true;
	}

	int prepare_motion(unsigned int* p_pErrorCode)
	{
		int lResult = MMC_SUCCESS;
		BOOL oIsFault = 0;

		if(VCS_GetFaultState(g_pKeyHandle, g_usNodeId, &oIsFault, p_pErrorCode ) == 0)
		{
			ROS_ERROR_STREAM("VCS_GetFaultState " <<", " << lResult <<", " << *p_pErrorCode);
			lResult = MMC_FAILED;
		}

		if(lResult==0)
		{
			if(oIsFault)
			{
				stringstream msg;
				msg << "clear fault, node = '" << g_usNodeId << "'";
				ROS_INFO_STREAM(msg.str());

				if(VCS_ClearFault(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
				{
					ROS_ERROR_STREAM("VCS_ClearFault" <<", " << lResult <<", " << *p_pErrorCode);
					lResult = MMC_FAILED;
				}
			}

			if(lResult==0)
			{
				BOOL oIsEnabled = 0;

				if(VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, p_pErrorCode) == 0)
				{
					ROS_ERROR_STREAM("VCS_GetEnableState" <<", " << lResult <<", " << *p_pErrorCode);
					lResult = MMC_FAILED;
				}

				if(lResult==0)
				{
					if(!oIsEnabled)
					{
						if(VCS_SetEnableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
						{
							ROS_ERROR_STREAM("VCS_SetEnableState" <<", " << lResult <<", " << *p_pErrorCode);
							lResult = MMC_FAILED;
						}
					}			
				}
			}
		}		

		return lResult;
	}

	int start_motion(unsigned int* p_pErrorCode)
	{
		int lResult = MMC_SUCCESS;
		unsigned int lErrorCode = 0;
		
		//we always first go home
		lResult = down_velocity(g_pKeyHandle, g_usNodeId, lErrorCode);

		if(lResult != MMC_SUCCESS)
		{
			ROS_ERROR_STREAM("Velocity Mode Failed" <<", "<< lResult <<", " << lErrorCode);
		}
		else
		{
			if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId, &lErrorCode) == 0)
			{
				ROS_ERROR_STREAM("VCS_SetDisableState" << ", "<< lResult <<", " << lErrorCode);
				lResult = MMC_FAILED;
			}		
		}
		return lResult;
	}

	int end_motion(unsigned int* p_pErrorCode)
	{
		int lResult = MMC_SUCCESS;
		unsigned int lErrorCode = 0;
		
		lResult = up_velocity(g_pKeyHandle, g_usNodeId, lErrorCode);

		if(lResult != MMC_SUCCESS)
		{
			ROS_ERROR_STREAM("Velocity Mode Failed" <<", "<< lResult <<", " << lErrorCode);
		}
		else
		{
				if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId, &lErrorCode) == 0)
				{
					ROS_ERROR_STREAM("VCS_SetDisableState" << ", "<< lResult <<", " << lErrorCode);
					lResult = MMC_FAILED;
				}			
		}
		return lResult;
	}

	private:
		void* g_pKeyHandle;
		unsigned short g_usNodeId;
		int g_baudrate;
		unsigned short home_accel, speed_switch, speed_index;		
		long home_offset, home_pose;
		unsigned short current_thresh;
		string g_deviceName, g_protocolStackName, g_interfaceName, g_portName;
		BOOL faultState;
};

#endif
