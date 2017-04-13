#ifndef SETPOINT_MSG_DEFINES
#define SETPOINT_MSG_DEFINES

#define DEFAULT_MASK 

#define IGNORE_PX (1 << 0)	// Position ignore flags
#define IGNORE_PY (1 << 1)
#define IGNORE_PZ (1 << 2)
#define IGNORE_VX (1 << 3)	// Velocity vector ignore flags
#define IGNORE_VY (1 << 4)
#define IGNORE_VZ (1 << 5)
#define IGNORE_AFX (1 << 6)	// Acceleration/Force vector ignore flags
#define IGNORE_AFY (1 << 7)
#define IGNORE_AFZ (1 << 8)
#define FORCE (1 << 9)	// Force in af vector flag
#define IGNORE_YAW (1 << 10)
#define IGNORE_YAW_RATE (1 << 11)
#define SETPOINT_TYPE_TAKEOFF 0x1000
#define SETPOINT_TYPE_LAND 0x2000
#define SETPOINT_TYPE_LOITER 0x3000
#define SETPOINT_TYPE_IDLE 0x4000

constexpr uint16_t default_mask = IGNORE_VX | IGNORE_VY | IGNORE_VZ | 
								  IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | 
								  IGNORE_YAW_RATE;
#endif