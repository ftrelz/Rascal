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
  float x;
  float y;
  float z;
  float xdot;
  float ydot;
  float zdot;
} pose;

typedef struct {
  float x;
  float y;
  float z;
} velocity;

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

//extern pose POSE_IMG


#endif /*__rascal_h */
