-- Author: Arvin Ignaci <arvin.ignaci@calanalytics.com>
-- NOTE: This software currently only works with MAVLink Protocol Version 1.0

-- Length of different field types in bytes
-- NOTE: Strings are considered 1 byte for the purpose of sorting.
local flength = {
	[ftypes.INT8] = 1,
	[ftypes.UINT8] = 1,
	[ftypes.INT16] = 2,
	[ftypes.UINT16] = 2,
	[ftypes.FLOAT] = 4,
	[ftypes.INT32] = 4,
	[ftypes.UINT32] = 4,
	[ftypes.DOUBLE] = 8,
	[ftypes.INT64] = 8,
	[ftypes.UINT64] = 8,
	[ftypes.STRING] = 1,
}

-- Register MAVLink protocol
local mavlink = Proto("mavlink", "MAVLink Protocol")

-- Maps magic values to MAVLink protocol version
local version_from_magic = {
	[85] = 0.9,
	[253] = 2.0,
	[254] = 1.0,
}

local component_id = {
	[0] = "MAV_COMP_ID_ALL",
	[1] = "MAV_COMP_ID_AUTOPILOT1",
	[100] = "MAV_COMP_ID_CAMERA",
	[101] = "MAV_COMP_ID_CAMERA2",
	[102] = "MAV_COMP_ID_CAMERA3",
	[103] = "MAV_COMP_ID_CAMERA4",
	[104] = "MAV_COMP_ID_CAMERA5",
	[105] = "MAV_COMP_ID_CAMERA6",
	[140] = "MAV_COMP_ID_SERVO1",
	[141] = "MAV_COMP_ID_SERVO2",
	[142] = "MAV_COMP_ID_SERVO3",
	[143] = "MAV_COMP_ID_SERVO4",
	[144] = "MAV_COMP_ID_SERVO5",
	[145] = "MAV_COMP_ID_SERVO6",
	[146] = "MAV_COMP_ID_SERVO7",
	[147] = "MAV_COMP_ID_SERVO8",
	[148] = "MAV_COMP_ID_SERVO9",
	[149] = "MAV_COMP_ID_SERVO10",
	[150] = "MAV_COMP_ID_SERVO11",
	[151] = "MAV_COMP_ID_SERVO12",
	[152] = "MAV_COMP_ID_SERVO13",
	[153] = "MAV_COMP_ID_SERVO14",
	[154] = "MAV_COMP_ID_GIMBAL",
	[155] = "MAV_COMP_ID_LOG",
	[156] = "MAV_COMP_ID_ADSB",
	[157] = "MAV_COMP_ID_OSD",
	[158] = "MAV_COMP_ID_PERIPHERAL",
	[159] = "MAV_COMP_ID_QX1_GIMBAL",
	[160] = "MAV_COMP_ID_FLARM",
	[180] = "MAV_COMP_ID_MAPPER",
	[190] = "MAV_COMP_ID_MISSIONPLANNER",
	[195] = "MAV_COMP_ID_PATHPLANNER",
	[200] = "MAV_COMP_ID_IMU",
	[201] = "MAV_COMP_ID_IMU_2",
	[202] = "MAV_COMP_ID_IMU_3",
	[220] = "MAV_COMP_ID_GPS",
	[221] = "MAV_COMP_ID_GPS2",
	[240] = "MAV_COMP_ID_UDP_BRIDGE",
	[241] = "MAV_COMP_ID_UART_BRIDGE",
	[250] = "MAV_COMP_ID_SYSTEM_CONTROL",
}

local message_id = {
	[0] = "HEARTBEAT",
	[1] = "SYS_STATUS",
	[2] = "SYSTEM_TIME",
	[4] = "PING",
	[5] = "CHANGE_OPERATOR_CONTROL",
	[6] = "CHANGE_OPERATOR_CONTROL_ACK",
	[7] = "AUTH_KEY",
	[11] = "SET_MODE",
	[20] = "PARAM_REQUEST_READ",
	[21] = "PARAM_REQUEST_LIST",
	[22] = "PARAM_VALUE",
	[23] = "PARAM_SET",
	[24] = "GPS_RAW_INIT",
	[25] = "GPS_STATUS",
	[26] = "SCALED_IMU",
	[27] = "RAW_IMU",
	[28] = "RAW_PRESSURE",
	[29] = "SCALED_PRESSURE",
	[30] = "ATTITUDE",
	[31] = "ATTITUDE_QUATERNION",
	[32] = "LOCAL_POSITION_NED",
	[33] = "GLOBAL_POSITION_INT",
	[34] = "RC_CHANNELS_SCALED",
	[35] = "RC_CHANNELS_RAW",
	[36] = "SERVO_OUTPUT_RAW",
	[37] = "MISSION_REQUEST_PARTIAL_LIST",
	[38] = "MISSION_WRITE_PARTIAL_LIST",
	[39] = "MISSION_ITEM",
	[40] = "MISSION_REQUEST",
	[41] = "MISSION_SET_CURRENT",
	[42] = "MISSION_CURRENT",
	[43] = "MISSION_REQUEST_LIST",
	[44] = "MISSION_COUNT",
	[45] = "MISSION_CLEAR_ALL",
	[46] = "MISSION_ITEM_REACHED",
	[47] = "MISSION_ACK",
	[48] = "SET_GPS_GLOBAL_ORIGIN",
	[49] = "GPS_GLOBAL_ORIGIN",
	[50] = "PARAM_MAP_RC",
	[51] = "MISSION_REQUEST_INT",
	[54] = "SAFETY_SET_ALLOWED_AREA",
	[55] = "SAFETY_ALLOWED_AREA",
	[61] = "ATTITUDE_QUATERNION_COV",
	[62] = "NAV_CONTROLLER_OUTPUT",
	[63] = "GLOBAL_POSITION_INT_COV",
	[64] = "LOCAL_POSITION_NED_COV",
	[65] = "RC_CHANNELS",
	[66] = "REQUEST_DATA_STREAM",
	[67] = "DATA_STREAM",
	[69] = "MANUAL_CONTROL",
	[70] = "RC_CHANNELS_OVERRIDE",
	[73] = "MISSION_ITEM_INT",
	[74] = "VFR_HUD",
	[75] = "COMMAND_INT",
	[76] = "COMMAND_LONG",
	[77] = "COMMAND_ACK",
	[81] = "MANUAL_SETPOINT",
	[82] = "SET_ATTITUDE_TARGET",
	[83] = "ATTITUDE_TARGET",
	[84] = "SET_POSITION_TARGET_LOCAL_NED",
	[85] = "POSITION_TARGET_LOCAL_NED",
	[86] = "SET_POSITION_TARGET_GLOBAL_INT",
	[87] = "POSITION_TARGET_GLOBAL_INT",
	[89] = "LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET",
	[90] = "HIL_STATE",
	[91] = "HIL_CONTROLS",
	[92] = "HIL_RC_INPUTS_RAW",
	[93] = "HIL_ACTUATOR_CONTROLS",
	[100] = "OPTICAL_FLOW",
	[101] = "GLOBAL_VISION_POSITION_ESTIMATE",
	[102] = "VISION_POSITION_ESTIMATE",
	[103] = "VISION_SPEED_ESTIMATE",
	[104] = "VICON_POSITION_ESTIMATE",
	[105] = "HIGHRES_IMU",
	[106] = "OPTICAL_FLOW_RAD",
	[107] = "HIL_SENSOR",
	[108] = "SIM_STATE",
	[109] = "RADIO_STATUS",
	[110] = "FILE_TRANSFER_PROTOCOL",
	[111] = "TIMESYNC",
	[112] = "CAMERA_TRIGGER",
	[113] = "HIL_GPS",
	[114] = "HIL_OPTICAL_FLOW",
	[115] = "HIL_STATE_QUATERNION",
	[116] = "SCALED_IMU2",
	[117] = "LOG_REQUEST_LIST",
	[118] = "LOG_ENTRY",
	[119] = "LOG_REQUEST_DATA",
	[120] = "LOG_DATA",
	[121] = "LOG_ERASE",
	[122] = "LOG_REQUEST_END",
	[123] = "GPS_INJECT_DATA",
	[124] = "GPS2_RAW",
	[125] = "POWER_STATUS",
	[126] = "SERIAL_CONTROL",
	[127] = "GPS_RTK",
	[128] = "GPS2_RTK",
	[129] = "SCALED_IMU3",
	[130] = "DATA_TRANSMISSION_HANDSHAKE",
	[131] = "ENCAPSULATED_DATA",
	[132] = "DISTANCE_SENSOR",
	[133] = "TERRAIN_REQUEST",
	[134] = "TERRAIN_DATA",
	[135] = "TERRAIN_CHECK",
	[136] = "TERRAIN_REPORT",
	[137] = "SCALED_PRESSURE2",
	[138] = "ATT_POS_MOCAP",
	[139] = "SET_ACTUATOR_CONTROL_TARGET",
	[140] = "ACTUATOR_CONTROL_TARGET",
	[141] = "ALTITUDE",
	[142] = "RESOURCE_REQUEST",
	[143] = "SCALED_PRESSURE3",
	[144] = "FOLLOW_TARGET",
	[146] = "CONTROL_SYSTEM_STATE",
	[147] = "BATTERY_STATUS",
	[148] = "AUTOPILOT_VERSION",
	[149] = "LANDING_TARGET",
	[150] = "SENSOR_OFFSETS",
	[151] = "SET_MAG_OFFSETS",
	[152] = "MEMINFO",
	[153] = "AP_ADC",
	[154] = "DIGICAM_CONFIGURE",
	[155] = "DIGICAM_CONTROL",
	[156] = "MOUNT_CONFIGURE",
	[157] = "MOUNT_CONTROL",
	[158] = "MOUNT_STATUS",
	[160] = "FENCE_POINT",
	[161] = "FENCE_FETCH_POINT",
	[162] = "FENCE_STATUS",
	[163] = "AHRS",
	[164] = "SIMSTATE",
	[165] = "HWSTATUS",
	[166] = "RADIO",
	[167] = "LIMITS_STATUS",
	[168] = "WIND",
	[169] = "DATA16",
	[170] = "DATA32",
	[171] = "DATA64",
	[172] = "DATA96",
	[173] = "RANGEFINDER",
	[174] = "AIRSPEED_AUTOCAL",
	[175] = "RALLY_POINT",
	[176] = "RALLY_FETCH_POINT",
	[177] = "COMPASSMOT_STATUS",
	[178] = "AHRS2",
	[179] = "CAMERA_STATUS",
	[180] = "CAMERA_FEEDBACK",
	[181] = "BATTERY2",
	[182] = "AHRS3",
	[183] = "AUTOPILOT_VERSION_REQUEST",
	[184] = "REMOTE_LOG_DATA_BLOCK",
	[185] = "REMOTE_LOG_BLOCK_STATUS",
	[186] = "LED_CONTROL",
	[191] = "MAG_CAL_PROGRESS",
	[192] = "MAG_CAL_REPORT",
	[193] = "EKF_STATUS_REPORT",
	[194] = "PID_TUNING",
	[195] = "DEEPSTALL",
	[200] = "GIMBAL_REPORT",
	[201] = "GIMBAL_CONTROL",
	[214] = "GIMBAL_TORQUE_CMD_REPORT",
	[215] = "GOPRO_HEARTBEAT",
	[216] = "GOPRO_GET_REQUEST",
	[217] = "GOPRO_GET_RESPONSE",
	[218] = "GOPRO_SET_REQUEST",
	[219] = "GOPRO_SET_RESPONSE",
	[226] = "RPM",
	[230] = "ESTIMATOR_STATUS",
	[231] = "WIND_COV",
	[232] = "GPS_INPUT",
	[233] = "GPS_RTCM_DATA",
	[234] = "HIGH_LATENCY",
	[235] = "HIGH_LATENCY2",
	[241] = "VIBRATION",
	[242] = "HOME_POSITION",
	[243] = "SET_HOME_POSITION",
	[244] = "MESSAGE_INTERVAL",
	[245] = "EXTENDED_SYS_STATE",
	[246] = "ADSB_VEHICLE",
	[247] = "COLLISION",
	[248] = "V2_EXTENSION",
	[249] = "MEMORY_VECT",
	[250] = "DEBUG_VECT",
	[251] = "NAMED_VALUE_FLOAT",
	[252] = "NAMED_VALUE_INT",
	[253] = "STATUSTEXT",
	[254] = "DEBUG",
	[256] = "SETUP_SIGNING",
	[257] = "BUTTON_CHANGE",
	[258] = "PLAY_TUNE",
	[259] = "CAMERA_INFORMATION",
	[260] = "CAMERA_SETTINGS",
	[261] = "STORAGE_INFORMATION",
	[262] = "CAMERA_CAPTURE_STATUS",
	[263] = "CAMERA_IMAGE_CAPTURED",
	[264] = "FLIGHT_INFORMATION",
	[265] = "MOUNT_ORIENTATION",
	[266] = "LOGGING_DATA",
	[267] = "LOGGING_DATA_ACKED",
	[268] = "LOGGING_ACK",
	[269] = "VIDEO_STREAM_INFORMATION",
	[270] = "SET_VIDEO_STREAM_SETTINGS",
	[299] = "WIFI_CONFIG_AP",
	[300] = "PROTOCOL_VERSION",
	[310] = "UAVCAN_NODE_STATUS",
	[311] = "UAVCAN_NODE_INFO",
	[320] = "PARAM_EXT_REQUEST_READ",
	[321] = "PARAM_EXT_REQUEST_LIST",
	[322] = "PARAM_EXT_VALUE",
	[323] = "PARAM_EXT_SET",
	[324] = "PARAM_EXT_ACK",
	[330] = "OBSTACLE_DISTANCE",
	[331] = "ODOMETRY",
	[332] = "TRAJECTORY",
	[11000] = "DEVICE_OP_READ",
	[11001] = "DEVICE_OP_READ_REPLY",
	[11002] = "DEVICE_OP_WRITE",
	[11003] = "DEVICE_OP_WRITE_REPLY",
	[11010] = "ADAP_TUNING",
	[11011] = "VISION_POSITION_DELTA",
	[11020] = "AOA_SSA",
	[42000] = "ICAROUS_HEARTBEAT",
	[42001] = "ICAROUS_KINEMATIC_BANDS",
}

-- Additional fields that are sent with each message. The fields are mapped
-- to their corresponding message IDs.
-- These fields are contained in the payload.
local message_fields = {
	HEARTBEAT = {
		{ type = ftypes.UINT8, id = "type", enum = "MAV_TYPE" },
		{ type = ftypes.UINT8, id = "autopilot", enum = "MAV_AUTOPILOT" },
		{ type = ftypes.UINT8, id = "base_mode", enum = "MAV_MODE_FLAG" },
		{ type = ftypes.UINT32, id = "custom_mode" },
		{ type = ftypes.UINT8, id = "system_status", enum = "MAV_STATE" },
		{ type = ftypes.UINT8, id = "mavlink_version" },
	},
	SYS_STATUS = {
		{ type = ftypes.UINT32, id = "onboard_control_sensors_present",
		  mask = "MAV_SYS_STATUS_SENSOR" },
		{ type = ftypes.UINT32, id = "onboard_control_sensors_enabled",
		  mask = "MAV_SYS_STATUS_SENSOR" },
		{ type = ftypes.UINT32, id = "onboard_control_sensors_health",
		  mask = "MAV_SYS_STATUS_SENSOR" },
		{ type = ftypes.UINT16, id = "load" },
		{ type = ftypes.UINT16, id = "voltage_battery",
		  name = "Battery Voltage" },
		{ type = ftypes.INT16, id = "current_battery",
		  name = "Battery Current" },
		{ type = ftypes.INT8, id = "battery_remaining",
		  name = "Remaining Battery" },
		{ type = ftypes.UINT16, id = "drop_rate_comm",
		  name = "Communications Drop Rate" },
		{ type = ftypes.UINT16, id = "errors_comm",
		  name = "Communication Errors" },
		{ type = ftypes.UINT16, id = "errors_count1", name = "Errors Count1" },
		{ type = ftypes.UINT16, id = "errors_count2", name = "Errors Count2" },
		{ type = ftypes.UINT16, id = "errors_count3", name = "Errors Count3" },
		{ type = ftypes.UINT16, id = "errors_count4", name = "Errors Count4" },
	},
	PARAM_VALUE = {
		{ type = ftypes.STRING, id = "param_id", name = "ID", length = 16 },
		{ type = ftypes.FLOAT, id = "param_value", name = "Value" },
		{ type = ftypes.UINT8, id = "param_type", name = "Type",
		  enum = "MAV_PARAM_TYPE" },
		{ type = ftypes.UINT16, id = "param_count", name = "Count" },
		{ type = ftypes.UINT16, id = "param_index", name = "Index" },
	},
	MISSION_ITEM = {
		{ type = ftypes.UINT8, id = "target_system", name = "System ID" },
		{ type = ftypes.UINT8, id = "target_component",
		  name = "Component ID", enum = "MAV_COMPONENT" },
		{ type = ftypes.UINT16, id = "seq", name = "Sequence" },
		{ type = ftypes.UINT8, id = "frame", enum = "MAV_FRAME" },
		{ type = ftypes.UINT16, id = "command", enum = "MAV_CMD" },
		{ type = ftypes.UINT8, id = "current" },
		{ type = ftypes.UINT8, id = "autocontinue" },
		{ type = ftypes.FLOAT, id = "param1" },
		{ type = ftypes.FLOAT, id = "param2" },
		{ type = ftypes.FLOAT, id = "param3" },
		{ type = ftypes.FLOAT, id = "param4" },
		{ type = ftypes.FLOAT, id = "x" },
		{ type = ftypes.FLOAT, id = "y" },
		{ type = ftypes.FLOAT, id = "z" },
		{ type = ftypes.UINT8, id = "mission_type",
		  enum = "MAV_MISSION_TYPE" },
	},
	MISSION_REQUEST = {
		{ type = ftypes.UINT8, id = "target_system", name = "System ID" },
		{ type = ftypes.UINT8, id = "target_component",
		  name = "Component ID", enum = "MAV_COMPONENT" },
		{ type = ftypes.UINT16, id = "seq", name = "Sequence" },
		{ type = ftypes.UINT8, id = "mission_type",
		  enum = "MAV_MISSION_TYPE" },
	},
	MISSION_CURRENT ={
		{ type = ftypes.UINT16, id = "seq", name = "Sequence" },
	},
	MISSION_COUNT = { 
		{ type = ftypes.UINT8, id = "target_system", name = "System ID" },
		{ type = ftypes.UINT8, id = "target_component",
		  name = "Component ID", enum = "MAV_COMPONENT" },
		{ type = ftypes.UINT16, id = "count" },
		{ type = ftypes.UINT8, id = "mission_type",
		  enum = "MAV_MISSION_TYPE" },
	},
	MISSION_CLEAR_ALL = { 
		{ type = ftypes.UINT8, id = "target_system", name = "System ID" },
		{ type = ftypes.UINT8, id = "target_component",
		  name = "Component ID", enum = "MAV_COMPONENT" },
		{ type = ftypes.UINT8, id = "mission_type",
		  enum = "MAV_MISSION_TYPE" },
	},
	MISSION_ACK = {
		{ type = ftypes.UINT8, id = "target_system", name = "System ID" },
		{ type = ftypes.UINT8, id = "target_component",
		  name = "Component ID", enum = "MAV_COMPONENT" },
		{ type = ftypes.UINT8, id = "type",
		  enum = "MAV_MISSION_RESULT" },
	},
	MISSION_ITEM_INT = {
		{ type = ftypes.UINT8, id = "target_system", name = "System ID" },
		{ type = ftypes.UINT8, id = "target_component",
		  name = "Component ID", enum = "MAV_COMPONENT" },
		{ type = ftypes.UINT16, id = "seq", name = "Sequence" },
		{ type = ftypes.UINT8, id = "frame", enum = "MAV_FRAME" },
		{ type = ftypes.UINT16, id = "command", enum = "MAV_CMD" },
		{ type = ftypes.UINT8, id = "current" },
		{ type = ftypes.UINT8, id = "autocontinue" },
		{ type = ftypes.FLOAT, id = "param1" },
		{ type = ftypes.FLOAT, id = "param2" },
		{ type = ftypes.FLOAT, id = "param3" },
		{ type = ftypes.FLOAT, id = "param4" },
		{ type = ftypes.INT32, id = "x" },
		{ type = ftypes.INT32, id = "y" },
		{ type = ftypes.FLOAT, id = "z" },
		{ type = ftypes.UINT8, id = "mission_type",
		  enum = "MAV_MISSION_TYPE" },
	},
	STATUSTEXT = {
		{ type = ftypes.UINT8, id = "severity", enum = "MAV_SEVERITY" },
		{ type = ftypes.STRING, id = "text", length = 50 },
	},
}

-- Definitions for enumerated fields
local message_enums = {
	MAV_AUTOPILOT = {
		[0] = "MAV_AUTOPILOT_GENERIC",
		[1] = "MAV_AUTOPILOT_RESERVED",
		[2] = "MAV_AUTOPILOT_SLUGS",
		[3] = "MAV_AUTOPILOT_ARDUPILOTMEGA",
		[4] = "MAV_AUTOPILOT_OPENPILOT",
		[5] = "MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY",
		[6] = "MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY",
		[7] = "MAV_AUTOPILOT_GENERIC_MISSION_FULL",
		[8] = "MAV_AUTOPILOT_INVALID",
		[9] = "MAV_AUTOPILOT_PPZ",
		[10] = "MAV_AUTOPILOT_UDB",
		[11] = "MAV_AUTOPILOT_FP",
		[12] = "MAV_AUTOPILOT_PX4",
		[13] = "MAV_AUTOPILOT_SMACCMPILOT",
		[14] = "MAV_AUTOPILOT_AUTOQUAD",
		[15] = "MAV_AUTOPILOT_ARMAZILA",
		[16] = "MAV_AUTOPILOT_AEROB",
		[17] = "MAV_AUTOPILOT_ASLUAV",
		[18] = "MAV_AUTOPILOT_SMARTAP",
		[19] = "MAV_AUTOPILOT_AIRRAILS",
	},
	MAV_TYPE = {
		[0] = "MAV_TYPE_GENERIC",
		[1] = "MAV_TYPE_FIXED_WING",
		[2] = "MAV_TYPE_QUADROTOR",
		[3] = "MAV_TYPE_COAXIAL",
		[4] = "MAV_TYPE_HELICOPTER",
		[5] = "MAV_TYPE_ANTENNA_TRACKER",
		[6] = "MAV_TYPE_GCS",
		[7] = "MAV_TYPE_AIRSHIP",
		[8] = "MAV_TYPE_FREE_BALLOON",
		[9] = "MAV_TYPE_ROCKET",
		[10] = "MAV_TYPE_GROUND_ROVER",
		[11] = "MAV_TYPE_SURFACE_BOAT",
		[12] = "MAV_TYPE_SUBMARINE",
		[13] = "MAV_TYPE_HEXAROTOR",
		[14] = "MAV_TYPE_OCTOROTOR",
		[15] = "MAV_TYPE_TRICOPTER",
		[16] = "MAV_TYPE_FLAPPING_WING",
		[17] = "MAV_TYPE_KITE",
		[18] = "MAV_TYPE_ONBOARD_CONTROLLER",
		[19] = "MAV_TYPE_VTOL_DUOROTOR",
		[20] = "MAV_TYPE_VTOL_QUADROTOR",
		[21] = "MAV_TYPE_VTOL_TILTROTOR",
		[22] = "MAV_TYPE_VTOL_RESERVED2",
		[23] = "MAV_TYPE_VTOL_RESERVED3",
		[24] = "MAV_TYPE_VTOL_RESERVED4",
		[25] = "MAV_TYPE_VTOL_RESERVED5",
		[26] = "MAV_TYPE_GIMBAL",
		[27] = "MAV_TYPE_ADSB",
		[28] = "MAV_TYPE_PARAFOIL",
		[29] = "MAV_TYPE_DODECAROTOR",
		[30] = "MAV_TYPE_CAMERA",
		[31] = "MAV_TYPE_CHARGING_STATION",
		[32] = "MAV_TYPE_FLARM",
	},
	MAV_MODE_FLAG = {
		[128] = "MAV_MODE_FLAG_SAFETY_ARMED",
		[64] = "MAV_MODE_FLAG_MANUAL_INPUT_ENABLED",
		[32] = "MAV_MODE_FLAG_HIL_ENABLED",
		[16] = "MAV_MODE_FLAG_STABILIZE_ENABLED",
		[8] = "MAV_MODE_FLAG_GUIDED_ENABLED",
		[4] = "MAV_MODE_FLAG_AUTO_ENABLED",
		[2] = "MAV_MODE_FLAG_TEST_ENABLED",
		[1] = "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED",
	},
	MAV_STATE = {
		[0] = "MAV_STATE_UNINIT",
		"MAV_STATE_BOOT",
		"MAV_STATE_CALIBRATING",
		"MAV_STATE_STANDBY",
		"MAV_STATE_ACTIVE",
		"MAV_STATE_CRITICAL",
		"MAV_STATE_EMERGENCY",
		"MAV_STATE_POWEROFF",
		"MAV_STATE_FLIGHT_TERMINATION",
	},
	MAV_COMPONENT = component_id,
	MAV_SYS_STATUS_SENSOR = {
		[1] = "MAV_SYS_STATUS_SENSOR_3D_GYRO",
		[2] = "MAV_SYS_STATUS_SENSOR_3D_ACCEL",
		[4] = "MAV_SYS_STATUS_SENSOR_3D_MAG",
		[8] = "MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE",
		[16] = "MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE",
		[32] = "MAV_SYS_STATUS_SENSOR_GPS",
		[64] = "MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW",
		[128] = "MAV_SYS_STATUS_SENSOR_VISION_POSITION",
		[256] = "MAV_SYS_STATUS_SENSOR_LASER_POSITION",
		[512] = "MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH",
		[1024] = "MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL",
		[2048] = "MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION",
		[4096] = "MAV_SYS_STATUS_SENSOR_YAW_POSITION",
		[8192] = "MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL",
		[16384] = "MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL",
		[32768] = "MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS",
		[65536] = "MAV_SYS_STATUS_SENSOR_RC_RECEIVER",
		[131072] = "MAV_SYS_STATUS_SENSOR_3D_GYRO2",
		[262144] = "MAV_SYS_STATUS_SENSOR_3D_ACCEL2",
		[524288] = "MAV_SYS_STATUS_SENSOR_3D_MAG2",
		[33554432] = "MAV_SYS_STATUS_SENSOR_BATTERY",
		[67108864] = "MAV_SYS_STATUS_SENSOR_PROXIMITY",

	},
	MAV_FRAME = {
		[0] = "MAV_FRAME_GLOBAL",
		"MAV_FRAME_LOCAL_NED",
		"MAV_FRAME_MISSION",
		"MAV_FRAME_GLOBAL_RELATIVE_ALT",
		"MAV_FRAME_LOCAL_ENU",
		"MAV_FRAME_GLOBAL_INT",
		"MAV_FRAME_GLOBAL_RELATIVE_ALT_INT",
		"MAV_FRAME_LOCAL_OFFSET_NED",
		"MAV_FRAME_BODY_NED",
		"MAV_FRAME_BODY_OFFSET_NED",
		"MAV_FRAME_GLOBAL_TERRAIN_ALT",
		"MAV_FRAME_GLOBAL_TERRAIN_ALT_INT",
		"MAV_FRAME_BODY_FRD",
		"MAV_FRAME_BODY_FLU",
		"MAV_FRAME_MOCAP_NED",
		"MAV_FRAME_MOCAP_ENU",
		"MAV_FRAME_VISION_NED",
		"MAV_FRAME_VISION_ENU",
		"MAV_FRAME_ESTIM_NED",
		"MAV_FRAME_ESTIM_ENU",
	},
	MAV_CMD = {
		[16] = "MAV_CMD_NAV_WAYPOINT",
		[17] = "MAV_CMD_NAV_LOITER_UNLIM",
		[18] = "MAV_CMD_NAV_LOITER_TURNS",
		[19] = "MAV_CMD_NAV_LOITER_TIME",
		[20] = "MAV_CMD_NAV_RETURN_TO_LAUNCH",
		[21] = "MAV_CMD_NAV_LAND",
		[22] = "MAV_CMD_NAV_TAKEOFF",
		[23] = "MAV_CMD_NAV_LAND_LOCAL",
		[24] = "MAV_CMD_NAV_TAKEOFF_LOCAL",
		[25] = "MAV_CMD_NAV_FOLLOW",
		[30] = "MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT",
		[31] = "MAV_CMD_NAV_LOITER_TO_ALT",
		[32] = "MAV_CMD_DO_FOLLOW",
		[33] = "MAV_CMD_DO_FOLLOW_REPOSITION",
		[80] = "MAV_CMD_NAV_ROI",
		[81] = "MAV_CMD_NAV_PATHPLANNING",
		[82] = "MAV_CMD_NAV_SPLINE_WAYPOINT",
		[84] = "MAV_CMD_NAV_VTOL_TAKEOFF",
		[85] = "MAV_CMD_NAV_VTOL_LAND",
		[92] = "MAV_CMD_NAV_GUIDED_ENABLE",
		[93] = "MAV_CMD_NAV_DELAY",
		[94] = "MAV_CMD_NAV_PAYLOAD_PLACE",
		[95] = "MAV_CMD_NAV_LAST",
		[112] = "MAV_CMD_CONDITION_DELAY",
		[113] = "MAV_CMD_CONDITION_CHANGE_ALT",
		[114] = "MAV_CMD_CONDITION_DISTANCE",
		[115] = "MAV_CMD_CONDITION_YAW",
		[159] = "MAV_CMD_CONDITION_LAST",
		[176] = "MAV_CMD_DO_SET_MODE",
		[177] = "MAV_CMD_DO_JUMP",
		[178] = "MAV_CMD_DO_CHANGE_SPEED",
		[179] = "MAV_CMD_DO_SET_HOME",
		[180] = "MAV_CMD_DO_SET_PARAMETER",
		[181] = "MAV_CMD_DO_SET_RELAY",
		[182] = "MAV_CMD_DO_REPEAT_RELAY",
		[183] = "MAV_CMD_DO_SET_SERVO",
		[184] = "MAV_CMD_DO_REPEAT_SERVO",
		[185] = "MAV_CMD_DO_FLIGHTTERMINATION",
		[186] = "MAV_CMD_DO_CHANGE_ALTITUDE",
		[189] = "MAV_CMD_DO_LAND_START",
		[190] = "MAV_CMD_DO_RALLY_LAND",
		[191] = "MAV_CMD_DO_GO_AROUND",
		[192] = "MAV_CMD_DO_REPOSITION",
		[193] = "MAV_CMD_DO_PAUSE_CONTINUE",
		[194] = "MAV_CMD_DO_SET_REVERSE",
		[195] = "MAV_CMD_DO_SET_ROI_LOCATION",
		[196] = "MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET",
		[197] = "MAV_CMD_DO_SET_ROI_NONE",
		[200] = "MAV_CMD_DO_CONTROL_VIDEO",
		[201] = "MAV_CMD_DO_SET_ROI",
		[202] = "MAV_CMD_DO_DIGICAM_CONFIGURE",
		[203] = "MAV_CMD_DO_DIGICAM_CONTROL",
		[204] = "MAV_CMD_DO_MOUNT_CONFIGURE",
		[205] = "MAV_CMD_DO_MOUNT_CONTROL",
		[206] = "MAV_CMD_DO_SET_CAM_TRIGG_DIST",
		[207] = "MAV_CMD_DO_FENCE_ENABLE",
		[208] = "MAV_CMD_DO_PARACHUTE",
		[209] = "MAV_CMD_DO_MOTOR_TEST",
		[210] = "MAV_CMD_DO_INVERTED_FLIGHT",
		[213] = "MAV_CMD_NAV_SET_YAW_SPEED",
		[214] = "MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL",
		[220] = "MAV_CMD_DO_MOUNT_CONTROL_QUAT",
		[221] = "MAV_CMD_DO_GUIDED_MASTER",
		[222] = "MAV_CMD_DO_GUIDED_LIMITS",
		[223] = "MAV_CMD_DO_ENGINE_CONTROL",
		[240] = "MAV_CMD_DO_LAST",
		[241] = "MAV_CMD_PREFLIGHT_CALIBRATION",
		[242] = "MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS",
		[243] = "MAV_CMD_PREFLIGHT_UAVCAN",
		[245] = "MAV_CMD_PREFLIGHT_STORAGE",
		[246] = "MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN",
		[252] = "MAV_CMD_OVERRIDE_GOTO",
		[300] = "MAV_CMD_MISSION_START",
		[400] = "MAV_CMD_COMPONENT_ARM_DISARM",
		[410] = "MAV_CMD_GET_HOME_POSITION",
		[500] = "MAV_CMD_START_RX_PAIR",
		[510] = "MAV_CMD_GET_MESSAGE_INTERVAL",
		[511] = "MAV_CMD_SET_MESSAGE_INTERVAL",
		[519] = "MAV_CMD_REQUEST_PROTOCOL_VERSION",
		[520] = "MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES",
		[521] = "MAV_CMD_REQUEST_CAMERA_INFORMATION",
		[522] = "MAV_CMD_REQUEST_CAMERA_SETTINGS",
		[525] = "MAV_CMD_REQUEST_STORAGE_INFORMATION",
		[526] = "MAV_CMD_STORAGE_FORMAT",
		[527] = "MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS",
		[528] = "MAV_CMD_REQUEST_FLIGHT_INFORMATION",
		[529] = "MAV_CMD_RESET_CAMERA_SETTINGS",
		[530] = "MAV_CMD_SET_CAMERA_MODE",
		[2000] = "MAV_CMD_IMAGE_START_CAPTURE",
		[2001] = "MAV_CMD_IMAGE_STOP_CAPTURE",
		[2002] = "MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE",
		[2003] = "MAV_CMD_DO_TRIGGER_CONTROL",
		[2500] = "MAV_CMD_VIDEO_START_CAPTURE",
		[2501] = "MAV_CMD_VIDEO_STOP_CAPTURE",
		[2502] = "MAV_CMD_VIDEO_START_STREAMING",
		[2503] = "MAV_CMD_VIDEO_STOP_STREAMING",
		[2504] = "MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION",
		[2510] = "MAV_CMD_LOGGING_START",
		[2511] = "MAV_CMD_LOGGING_STOP",
		[2520] = "MAV_CMD_AIRFRAME_CONFIGURATION",
		[2600] = "MAV_CMD_CONTROL_HIGH_LATENCY",
		[2800] = "MAV_CMD_PANORAMA_CREATE",
		[3000] = "MAV_CMD_DO_VTOL_TRANSITION",
		[4000] = "MAV_CMD_SET_GUIDED_SUBMODE_STANDARD",
		[4001] = "MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE",
		[4501] = "MAV_CMD_CONDITION_GATE",
		[3001] = "MAV_CMD_ARM_AUTHORIZATION_REQUEST",
		[5000] = "MAV_CMD_NAV_FENCE_RETURN_POINT",
		[5001] = "MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION",
		[5002] = "MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION",
		[5003] = "MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION",
		[5004] = "MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION",
		[5100] = "MAV_CMD_NAV_RALLY_POINT",
		[5200] = "MAV_CMD_UAVCAN_GET_NODE_INFO",
		[30001] = "MAV_CMD_PAYLOAD_PREPARE_DEPLOY",
		[30002] = "MAV_CMD_PAYLOAD_CONTROL_DEPLOY",
		[31000] = "MAV_CMD_WAYPOINT_USER_1",
		[31001] = "MAV_CMD_WAYPOINT_USER_2",
		[31002] = "MAV_CMD_WAYPOINT_USER_3",
		[31003] = "MAV_CMD_WAYPOINT_USER_4",
		[31004] = "MAV_CMD_WAYPOINT_USER_5",
		[31005] = "MAV_CMD_SPATIAL_USER_1",
		[31006] = "MAV_CMD_SPATIAL_USER_2",
		[31007] = "MAV_CMD_SPATIAL_USER_3",
		[31008] = "MAV_CMD_SPATIAL_USER_4",
		[31009] = "MAV_CMD_SPATIAL_USER_5",
		[31010] = "MAV_CMD_USER_1",
		[31011] = "MAV_CMD_USER_2",
		[31012] = "MAV_CMD_USER_3",
		[31013] = "MAV_CMD_USER_4",
		[31014] = "MAV_CMD_USER_5",
	},
	MAV_PARAM_TYPE = {
		"MAV_PARAM_TYPE_UINT8",
		"MAV_PARAM_TYPE_INT8",
		"MAV_PARAM_TYPE_UINT16",
		"MAV_PARAM_TYPE_INT16",
		"MAV_PARAM_TYPE_UINT32",
		"MAV_PARAM_TYPE_INT32",
		"MAV_PARAM_TYPE_UINT64",
		"MAV_PARAM_TYPE_INT64",
		"MAV_PARAM_TYPE_REAL32",
		"MAV_PARAM_TYPE_REAL64",
	},
	MAV_MISSION_RESULT = {
		[0] = "MAV_MISSION_ACCEPTED",
		"MAV_MISSION_ERROR",
		"MAV_MISSION_UNSUPPORTED_FRAME",
		"MAV_MISSION_UNSUPPORTED",
		"MAV_MISSION_NO_SPACE",
		"MAV_MISSION_INVALID",
		"MAV_MISSION_INVALID_PARAM1",
		"MAV_MISSION_INVALID_PARAM2",
		"MAV_MISSION_INVALID_PARAM3",
		"MAV_MISSION_INVALID_PARAM4",
		"MAV_MISSION_INVALID_PARAM5_X",
		"MAV_MISSION_INVALID_PARAM6_Y",
		"MAV_MISSION_INVALID_PARAM7",
		"MAV_MISSION_INVALID_SEQUENCE",
		"MAV_MISSION_DENIED",
	},
	MAV_SEVERITY = {
		[0] = "MAV_SEVERITY_EMERGENCY",
		"MAV_SEVERITY_ALERT",
		"MAV_SEVERITY_CRITICAL",
		"MAV_SEVERITY_ERROR",
		"MAV_SEVERITY_WARNING",
		"MAV_SEVERITY_NOTICE",
		"MAV_SEVERITY_INFO",
		"MAV_SEVERITY_DEBUG",
	},
	MAV_MISSION_TYPE = {
		"MAV_MISSION_TYPE_MISSION",
		"MAV_MISSION_TYPE_FENCE",
		"MAV_MISSION_TYPE_RALLY",
		[256] = "MAV_MISSION_TYPE_ALL",
	},
}


-- Constructing ProtoField objects for MAVLink
local pf_magic = ProtoField.uint8(
	"mavlink.magic", "Version", base.HEX, version_from_magic)
local pf_len = ProtoField.uint8("mavlink.length", "Payload Length", base.DEC)
local pf_incompat_flags = ProtoField.uint8(
	"mavlink.iflags", "Required Flags", base.HEX)
local pf_compat_flags = ProtoField.uint8(
	"mavlink.cflags", "Optional Flags", base.HEX)
local pf_seq = ProtoField.uint8("mavlink.seq", "Sequence Number", base.DEC)
local pf_sysid = ProtoField.uint8("mavlink.sysid", "System ID", base.DEC)
local pf_compid = ProtoField.uint8(
	"mavlink.compid", "Component ID", base.DEC, component_id)
local pf_msgid = ProtoField.uint8(
	"mavlink.msgid", "Message ID", base.DEC, message_id)
local pf_target_sysid = ProtoField.uint8(
	"mavlink.tsysid", "Target System ID", base.DEC)
local pf_target_compid = ProtoField.uint8(
	"mavlink.tcompid", "Target Component ID", base.DEC)
local pf_payload = ProtoField.bytes("mavlink.payload", "Payload")
local pf_checksum = ProtoField.uint16("mavlink.checksum", "Checksum", base.HEX)
local pf_signature = ProtoField.bytes("mavlink.sig", "Signature")

-- Globally visible list of protocol fields
mavlink.fields = {
	pf_magic,
	pf_len,
	pf_incompat_flags,
	pf_compat_flags,
	pf_seq,
	pf_sysid,
	pf_compid,
	pf_msgid,
	pf_target_sysid,
	pf_target_compid,
	pf_payload,
	pf_checksum,
	pf_signature,
}

-- Converts underscores to spaces and capitalizes the beginning of a string
local function prettyprint(str)
	return (str:gsub('_', ' '):gsub("^%l", string.upper))
end

-- Adding ProtoFields for message fields (found in payload)
local pfs_message = {}
for _, t in pairs(message_fields) do
	for _, f in pairs(t) do
		local abbr = "mavlink." .. f.id
		local name = f.name or prettyprint(f.id) or f.enum

		local pf = ProtoField.new(name, abbr, f.type, message_enums[f.enum])
		pfs_message[f.id] = pf
		table.insert(mavlink.fields, pf)

		if f.mask ~= nil then
			local mask = message_enums[f.mask]
			for bit, id in ipairs(mask) do
				local xabbr = abbr .. bit
				local len = f.length or flength[f.type]
				pf = ProtoField.bool(
					xabbr, message_enums[f.mask][bit], len, nil, bit
				)
			end
		end
	end
end

-- Sorts MAVLink message fields (insertion sort)
-- MAVLink wire format orders payload fields by length from largest to
-- smallest. For fields with the same length, the original order is preserved.
-- A stable sorting algorithm is therefore used.
local function sort_fields(f)
	for i = 2, #f do
		for j = i - 1, 1, -1 do
			local k = j + 1
			local function len(a) return flength[a.type] or a.length end
			if len(f[j]) < len(f[k]) then
				local swap = f[j]
				f[j] = f[k]
				f[k] = swap
			else
				break
			end
		end
	end
	return f
end

-- Payload dissector
local function payload_parser(msgid, buffer, pinfo, tree)
	local fields = message_fields[msgid]
	if fields == nil then return end
	local sfields = sort_fields(fields)
	local offset = 0

	for _, f in ipairs(sfields) do
		local pf = pfs_message[f.id]
		local len = f.length or flength[f.type]

		if (pf ~= nil) and (offset + len <= buffer:len()) then
			tree:add(pf, buffer(offset, len))
		end

		offset = offset + len
	end
end

-- Main dissector for MAVLink packets. Called for each packet.
function mavlink.dissector(buffer, pinfo, root)
	-- Overwrite 'Protocol' column in packet list
	pinfo.cols.protocol = "MAVLink"

	-- Add a new TreeInfo block. All decoded fields will be listed
	-- under this block.
	local tree = root:add(mavlink, buffer(), "MAVLink Protocol")

	-- Read magic value and determine MAVLink version
	local magic = buffer(0, 1):uint()
	local version = version_from_magic[magic]
	local version_str = string.format("Version %0.1f", version)
	tree:add(pf_magic, buffer(0, 1))

	-- Decode remaining fields
	local len = buffer(1, 1):uint()
	tree:add(pf_len, buffer(1, 1))
	tree:add(pf_seq, buffer(2, 1))

	local sysid = buffer(3, 1):uint()
	local compid = buffer(4, 1):uint()
	tree:add(pf_sysid, buffer(3, 1))
	tree:add(pf_compid, buffer(4, 1))

	local msgid = message_id[buffer(5, 1):uint()]
	ti_msgid = tree:add(pf_msgid, buffer(5, 1))

	ti_payload = tree:add(pf_payload, buffer(6, len))
	tree:add(pf_checksum, buffer(6 + len, 2))

	-- Add summary of decoded fields to main TreeInfo node
	local summary = " " .. version_str .. ", System: " .. sysid
		.. ", Component: " .. compid .. ", " .. msgid

	tree:append_text(summary)
	pinfo.cols.info = msgid

	-- Call local helper to dissect payload
	payload_parser(msgid, buffer(6, len), pinfo, ti_payload)
end

-- Register protocol to specified UDP ports
local udp_table = DissectorTable.get("udp.port")
for port = 14550, 14553 do
	udp_table:add(port, mavlink)
end
