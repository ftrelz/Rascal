#ifndef __rascal_h
#define __main_h
#define PI 3.14159265359

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
  float xdes; // desired final pos coordinates
  float ydes;
  float zdes;
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

//extern pose POSE_IMG

extern float C_ItoB[3][3];


#endif /*__rascal_h */
