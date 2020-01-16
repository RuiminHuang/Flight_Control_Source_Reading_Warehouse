/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright ?2011-2015  Bill Nesbitt
    Copyright 2013-2016 Maxim Paperno
*/

#ifndef _nav_h
#define _nav_h

#include "aq.h"
#include "pid.h"


#define NAV_MIN_GPS_VACC	3.5f//5.0f //����ˮƽ0.8    ��ֱ2.5    �õ�ʱ��ˮƽ0.5  ��ֱ0.9
#define NAV_MIN_GPS_ACC		1.8f//3.0f					    // minimum gps hAcc needed to enter auto nav modes, in meters
#define NAV_MAX_GPS_AGE		1e6					    // maximum age of position update needed to enter auto nav modes, in microseconds
#define NAV_MIN_FIX_ACC		2.5f//4.0f					    // minimum gps hAcc still considered a valid "2D" fix, in meters
#define NAV_MIN_FIX_VACC	5.0f//6.0f	
#define NAV_MAX_FIX_AGE		10e6					    // maximum age of position update still considered a valid "2D" fix, in microseconds

#define NAV_MAX_MISSION_LEGS	100					    // each leg takes 48 bytes CCM RAM, take care if increasing the limit, decrease to regain space

#define NAV_EQUATORIAL_RADIUS	(6378.137 * 1000.0)			    // meters
#define NAV_FLATTENING		(1.0 / 298.257223563)			    // WGS-84
#define NAV_E_2			(NAV_FLATTENING * (2.0 - NAV_FLATTENING))

#define NAV_HF_HOME_DIST_D_MIN	2.0f						// do not compute dynamic bearing when closer than this to home position (zero to never compute)
#define NAV_HF_HOME_DIST_FREQ	4						// update distance to home at this Hz, should be > 0 and <= 400
#define NAV_HF_HOME_BRG_D_MAX	0.1f * DEG_TO_RAD				// re-compute headfree reference angles when bearing to home changes by this many degrees (zero to always re-compute)
#define NAV_HF_DYNAMIC_DELAY	((int)3e6f)					// delay micros before entering dynamic mode after switch is toggled high

#define NAV_DFLT_HOR_SPEED	configGetParamValue(NAV_MAX_SPEED)	    // default horizontal navigation speed
#define NAV_DFLT_VRT_SPEED	configGetParamValue(NAV_MAX_ASCENT)	    // default vertical navigation speed, descent speed is constrained by NAV_MAX_DECENT param
#define NAV_DFLT_TOF_SPEED	configGetParamValue(NAV_LANDING_VEL)	    // default autonomous takeoff speed
#define NAV_DFLT_LND_SPEED	configGetParamValue(NAV_LANDING_VEL)	    // default autonomous landing speed

enum navStatusTypes {
    NAV_STATUS_MANUAL = 0,              // full manual control
    NAV_STATUS_ALTHOLD,                 // altitude hold only
    NAV_STATUS_POSHOLD,                 // altitude & position hold
    NAV_STATUS_DVH,                     // dynamic velocity hold cut through
    NAV_STATUS_MISSION,                 // autonomous mission
};

enum navLegTypes {
    NAV_LEG_HOME = 1,
    NAV_LEG_TAKEOFF,
    NAV_LEG_GOTO,
    NAV_LEG_ORBIT,
    NAV_LEG_LAND,
    NAV_NUM_LEG_TYPES
};

enum headFreeActiveModes {
    NAV_HEADFREE_OFF = 0,               // not active
    NAV_HEADFREE_SETTING,               // active, setting reference point
    NAV_HEADFREE_DYN_DELAY,             // active, waiting for timer to enable dynamic mode
    NAV_HEADFREE_LOCKED,                // active with locked frame reference
    NAV_HEADFREE_DYNAMIC                // active with continually adjusting frame reference
};

typedef struct {
    double targetLat;
    double targetLon;
    float targetAlt;			// either relative or absolute - if absolute, GPS altitude is used
    float targetRadius;			// achievement threshold for GOTO or orbit radius for ORBIT
    float maxHorizSpeed;		// m/s
    float maxVertSpeed;			// m/s
    float poiHeading;			// POI heading (>= 0 is absolute, < 0 is relative to target bearing)
    float poiAltitude;			// altitude of POI - used for camera tilting
    uint32_t loiterTime;		// us
    uint8_t type;
    bool relativeAlt;			// 0 == absolute, 1 == relative
} navMission_t;

typedef struct {
    float poiAngle;			// pitch angle for gimbal to center POI
    float holdAlt;			// altitude to hold
    float targetHeading;		// navigation heading target (>= 0 is absolute, < 0 is relative to target bearing)
    float holdHeading;			// heading to hold
    float holdCourse;			// course to hold position
    float holdDistance;			// distance to hold position (straight line)
    float holdMaxHorizSpeed;		// maximum N/E speed allowed to achieve position
    float holdMaxVertSpeed;		// maximum UP/DOWN speed allowed to achieve altitude
    float holdSpeedN;			// required speed (North/South)
    float holdSpeedE;			// required speed (East/West)
#ifdef _HSPD_SLOW
    float holdSpeedNFilter;	
    float holdSpeedEFilter;	
    float ChPitchFilter;    
    float ChRollFilter;            
#endif

#ifdef _VSPD_SLOW  
    float ChThroFilter; 
#endif
    float holdTiltN;			// required tilt (North/South)
    float holdTiltE;			// required tilt (East/West)
    float holdSpeedAlt;			// required speed (Up/Down)
    float targetHoldSpeedAlt;
    float presAltOffset;
    float distanceToHome;		// current distance to home position in m, if set (only updated when in headfree mode)
    float hfReferenceCos;		// stored reference heading for HF mode
    float hfReferenceSin;
    float hfReferenceYaw;		// stored yaw angle for heading-free modes
    float ceilingAlt;

    pidStruct_t *speedNPID;		// PID to control N/S speed - output tilt in degrees
    pidStruct_t *speedEPID;		// PID to control E/W speed - output tilt in degrees
    pidStruct_t *distanceNPID;		// PID to control N/S distance - output speed in m/s
    pidStruct_t *distanceEPID;		// PID to control E/W distance - output speed in m/s
    pidStruct_t *altSpeedPID;		// PID to control U/D speed - output speed in m/s
    pidStruct_t *altPosPID;		// PID to control U/D distance - output error in meters

    navMission_t missionLegs[NAV_MAX_MISSION_LEGS];
    navMission_t homeLeg;

    uint32_t lastUpdate;
    uint32_t loiterCompleteTime;
    uint32_t wpRecTimer;		// track how long WP recording switch is active before taking action
    uint32_t hfDynamicModeTimer;	// track how long switch is active before entering dynamic HF mode
    uint32_t homeDistanceLastUpdate;	// timestamp of last home position update (only updated when in headfree mode)
    uint32_t ceilingTimer;

    uint8_t missionLeg;			// current mission leg number
    uint8_t mode;			// navigation mode, one of navStatusTypes
    uint8_t spvrModeOverride;		// forced navigation mode desired, regardless of user controls (eg. failsafe mode). 0 = no override
    uint8_t headFreeMode;		// headfree/carefree mode status
    uint8_t fixType;			// GPS fix type, 0 = no fix, 2 = 2D, 3 = 3D (navCapable)

    bool navCapable;
    bool tempMissionLoaded;		// flag indicating that a temporary/default mission has been loaded (not a user-specified one)
    bool verticalOverride;
    bool homeActionFlag;		// flag to avoid repeating set/recall home actions until switch is moved back to midpoint
    bool wpActionFlag;			// flag to avoid repeating waypoint record/skip actions until switch is turned back off
    bool setCeilingFlag;
    bool setCeilingReached;
    bool hasMissionLeg;

//#ifdef _DISABLE_MANUAL	
    bool takeOff;
//#endif
} navStruct_t __attribute__((aligned));

extern navStruct_t navData;

extern void navInit(void);
//extern void navAccelUpdate(void);
//extern void navGpsUpdate(void);
//extern float navGetVel(char direction);
//extern float navGetPos(char direction);
extern unsigned int navGetWaypointCount(void);
extern unsigned char navClearWaypoints(void);
extern navMission_t *navGetWaypoint(int seqId);
extern navMission_t *navGetHomeWaypoint(void);
extern void navSetHomeCurrent(void);
extern navMission_t *navLoadLeg(unsigned char leg);
extern void navNavigate(void);
extern void navResetHoldAlt(float delta);
extern void navSetHoldAlt(float alt, uint8_t relative);
extern void navSetHoldHeading(float targetHeading);
extern float navCalcDistance(double lat1, double lon1, double lat2, double lon2);
extern float navCalcBearing(double lat1, double lon1, double lat2, double lon2);
extern void navPressureAdjust(float altitude);
extern uint8_t navRecordWaypoint(void);

#endif
