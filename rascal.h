#ifndef __rascal_h
#define __main_h
#define PI 3.14159265359

// defines Boeing serial frame constants, taken from PDG
#define FRAME_SIZE 258  //	indicates total frame size (in bytes)
#define DATA_SIZE 256	//	indicates size of data message (in bytes)
#define FLAG 0x7e		//	indicates flag for frame start or end
#define OUT_START_FLAG_SIZE 26
#define OUT_END_FLAG_SIZE 18
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

typedef struct {
  int thruster_Azminus; // thruster A points in -z direction in body coordinate system
  int Azminustime; // time thruster A has burned for (in milliseconds)
  int thruster_Bxminus; // thruster B points in -x direction in body coordinate system
  int Bxminustime; // time thruster B has burned for (in milliseconds)
  int thruster_Cyminus; // thruster C points in -y direction in body coordinate system
  int Cyminustime; // time thruster C has burned for (in milliseconds)
  int thruster_Dzplus; // thruster D points in +z direction in body coordinate system
  int Dzplustime; // time thruster D has burned for (in milliseconds)
  int thruster_Explus; // thruster E points in +x direction in body coordinate system
  int Explustime; // time thruster E has burned for (in milliseconds)
  int thruster_Fyplus; // thruster F points in +y direction in body coordinate system
  int Fyplustime; // time thruster F has burned for (in milliseconds)
} thrusterinfo;

/*
typedef struct {
  float x;
  float y;
  float z;
} velocity;
*/

typedef struct {
  float w; // angular vel of orbit
  float verror; // allowed vel error
  float xdes; // NOT used as POSE_DESIRED is used: desired final pos coordinates
  float ydes; // NOT used as POSE_DESIRED is used
  float zdes; // NOT used as POSE_DESIRED is used
  float xCruise; // x location where the orbit transfer occurs
} parameters;

//extern int * get_POSE_EST(void);

// declare structs
extern pose POSE_BOEING;
extern pose POSE_EST;
extern pose POSE_EST_PREV;
extern pose POSE_DESIRED;
extern pose POSE_ACTUAL;
extern pose POSE_IMG;
extern thrusterinfo THRUSTER_INFO;

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
