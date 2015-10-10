#ifndef __rascal_h
#define __main_h
#define PI 3.14159265359

// Pumpkin Salvo Headers
#include "csk_sd.h"
#include "thin_usr.h"

// fixed point libraries for Boeing Quaternion values
#include <libq.h>

// defines Boeing serial frame constants, taken from PDG
#define FRAME_SIZE 258  //	indicates total frame size (in bytes)
#define DATA_SIZE 256	//	indicates size of data message (in bytes)
#define FLAG 0x7e		//	indicates (byte stuffed) flag for frame start or end
#define DISABLED 0      //  used in conjunction with THRUST_ENABLE_FLAG
#define ENABLED 1       //  used in conjunction with THRUST_ENABLE_FLAG
#define IDLE 0          //  used in conjuction with STATUS_FLAG
#define NAV 1           //  "    "  "          "    "
#define ERROR 2         //  "    "  "          "    "

// struct for all POSE tasks:
// POSE_BOEING, POSE_DESIRED, POSE_EST, POSE_IMG
typedef struct {
  float q1;
  float q2;
  float q3;
  float q4;
  // quaternion velocities
  float q1dot;
  float q2dot;
  float q3dot;
  float q4dot;
  // position values
  float xi;
  float yi;
  float zi;
  // velocity values
  float xidot;
  float yidot;
  float zidot;
} pose;

// In matlab code the x and y axes are swapped, hence body coordinate frame of reference was chosen here
typedef struct {
  int thruster_Azminus; // thruster A points in -z direction in body coordinate system
  int Azminustime; // time thruster A has burned for (in milliseconds)
  int thruster_Byplus; // thruster B points in -x direction in body coordinate system
  int Byplustime; // time thruster B has burned for (in milliseconds)
  int thruster_Cxminus; // thruster C points in -y direction in body coordinate system
  int Cxminustime; // time thruster C has burned for (in milliseconds)
  int thruster_Dzplus; // thruster D points in +z direction in body coordinate system
  int Dzplustime; // time thruster D has burned for (in milliseconds)
  int thruster_Eyminus; // thruster E points in +x direction in body coordinate system
  int Eyminustime; // time thruster E has burned for (in milliseconds)
  int thruster_Fxplus; // thruster F points in +y direction in body coordinate system
  int Fxplustime; // time thruster F has burned for (in milliseconds)
  int thrusterOption;
} thrusterinfo;

typedef struct {
  float w; // angular vel of orbit
  float verror; // allowed vel error
  float xdes; // NOT used as POSE_DESIRED is used: desired final pos coordinates
  float ydes; // NOT used as POSE_DESIRED is used
  float zdes; // NOT used as POSE_DESIRED is used
  float xCruise; // x location where the orbit transfer occurs
} parameters;

typedef struct {
  float data[3][3];
} citob;

//extern int * get_POSE_EST(void);

// declare structs
extern pose POSE_BOEING;  // quaternion data received from Boeing's Colony II Bus
extern pose POSE_EST;
extern pose POSE_EST_PREV;
extern pose POSE_DESIRED;
extern pose POSE_ACTUAL;
extern pose POSE_IMG;
extern thrusterinfo THRUSTER_INFO;
extern citob C_ItoB;  // used in task_nav and task_estimator?

// defines THRUST_ENABLE_FLAG
// DISABLED = 0
// ENABLED = 1;
extern int THRUST_ENABLE_FLAG;

// defines STATUS_FLAG
// IDLE = 0
// NAV = 1
// ERROR = 2
extern int STATUS_FLAG;


#endif /*__rascal_h */
