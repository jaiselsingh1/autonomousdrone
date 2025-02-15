/***
 * Copyright 2022, The Pennsylvania State University, All Rights Reserved.
 * Unauthorized use and/or redistribution is disallowed.
 * This library is distributed without any warranty; without even
 * the implied warranty of fitness for a particular purpose.
 *
 * Pennsylvania State University Unmanned Aerial System Research Laboratory (PURL)
 * Department of Aerospace Engineering
 * 229 Hammond
 * The Pennsylvania State University
 * University Park, PA 16802
 * http://purl.psu.edu
 *
 * Contact Information:
 * Dr. Thanakorn Khamvilai Email: thanakorn.khamvilai@ttu.edu
 * Dr. Vitor Valente       Email: vitor.valente@psu.edu
 *
 * EndCopyright
 ***/

#ifndef AERSP_DATALINK_h
#define AERSP_DATALINK_h

#include <stdint.h>
#include <stdlib.h>
#include "wifi.h"
#include "rc_pilot.h"

#define DATALINK_SYNC0 0xa3
#define DATALINK_SYNC1 0xb2
#define DATALINK_SYNC2 0xc1

#define DATALINK_MESSAGE0 0
#define DATALINK_MESSAGE1 1
#define DATALINK_MESSAGE_UP0 12
#define DATALINK_MESSAGE_AUTOPILOTDELS 173
#define DATALINK_MESSAGE_RCCHANNEL 200
#define DATALINK_MESSAGE_MOTOR_CMD 201

struct obDatalink_ref
{
  struct datalinkWork_ref                 *work;          /* working area */
	struct datalinkHeader_ref               *header;        /* raw message header   */
  struct datalinkMessage0_ref             *m0;            /* raw message sent     */
  struct datalinkMessage1_ref             *m1;            /* raw message sent     */
  struct datalinkMessageUp0_ref           *up0;           /* raw message received */
  struct datalinkMessageMotorCmd_ref      *motor;         /* raw message send     */
  struct datalinkMessageRCChannel_ref     *rc;            /* raw message send     */
  struct datalinkMessageAutopilotDels_ref *autopilotDels; /* raw message sent     */
};

struct datalinkWork_ref
{
	uint32_t itime; /* number of messages received */
	int32_t badChecksums; /* */
	int32_t badHeaderChecksums; /*  */
};

struct datalinkHeader_ref
{
	unsigned char sync1; /*  */
	unsigned char sync2; /*  */
	unsigned char sync3; /*  */
	unsigned char spare; /*  */
	int32_t messageID; /* id # */
	int32_t messageSize; /* including header */
	uint32_t hcsum; /*  */
	uint32_t csum; /*  */
};

struct datalinkMessage0_ref {
  unsigned char sync1; /*  */
  unsigned char sync2; /*  */
  unsigned char sync3; /*  */
  unsigned char spare; /*  */
  int messageID; /* id # */
  int messageSize; /* including header */
  unsigned int hcsum; /*  */
  unsigned int csum; /*  */
  char navStatus; /* status of nav system */
  char gpsStatus; /* status of gps */
  char aglStatus; /* status of AGL sensor in bits (0,1), which AGL sensor source in bits (2-4) - see AGL_SENSOR_SOURCE enum in sensors.db */
  unsigned char overrun; /* frame overrun */
  char wow; /* weight on skids (0=weight off of skids = in air, 1=weight on skids = on ground, 2=uncertain) */
  char autopilot; /* autopilot engaged (0=rx,1=rx no integral, 2=auto,3=auto no integral,4=external,5=external no integral,6=auto no outer loop) */
  char LaunchState; /* 1=arm, 0=disarm */
  unsigned char motor; /* motor state */
  float time; /* onboard time */
  float pos[3]; /* position of vehicle */
  float vel[3]; /* velocity of vehicle */
  float q[4]; /* attitude */
  float altitudeAGL; /* altitude above terrain */
};

struct datalinkMessage1_ref {
  unsigned char sync1; /*  */
  unsigned char sync2; /*  */
  unsigned char sync3; /*  */
  unsigned char spare; /*  */
  int messageID; /* id # */
  int messageSize; /* including header */
  unsigned int hcsum; /*  */
  unsigned int csum; /*  */
  float time; /* onboard time */
  char numberOfSats; /* number of sats for GPS */
  char datarecordStatus; /* status of data recording */
  char safemode; /* safe mode status */
  unsigned char type; /* vehicle type */
  float delm[3]; /* actuators */
  float delf[1]; /* actuators */
  float delt[1]; /* actuators */
  float delc[1]; /* actuators */
  int battery; /* battery voltage (mV) */
  int current; /* total current (mA) */
  int rpm; /* rpm from hub */
  char tx; /* transmitter status */
  char fuel; /* fuel status */
  char ycsStatus; /* YCS status */
  unsigned char uniqueID; /* tail number */
  char yrdStatus; /* YRD status */
  char hubStatus; /* RPM status */
  char rangeFinderStatus; /* status by bits (0,1) = number of AGL sensors (0-3), (2,3) = 2nd AGL sensor status,(4,5) = 3rd AGL sensor status, (6,7) = status of a range finder not being used as an AGL sensor (off if there isn't one) */
  char magnetStatus; /* status of magnetometer */
  float traj_x[3]; /* trajectory */
  float traj_v[3]; /* trajectory */
  float traj_a[3]; /* trajectory */
  float traj_q[4]; /* trajectory */
  float traj_psi; /* trajectory */
  float traj_vscale; /* trajectory */
  short traj_manIndex; /* trajectory */
  unsigned char align[1]; /*  */
  unsigned char actuatorInterfaceStatus; /* Status for low-level actuator interface (0=armed,1=unarmed,2=armed without SAS,3=unarmed without SAS,4=failure/no response) */
  unsigned char imuStatus; /* imu status */
  unsigned char traj_status; /* trajectory */
  unsigned char visionStatus; /* vision satus */
  unsigned char missionStatus; /* mision status */
  unsigned char otherStatus; /* other Status */
  unsigned char cameraControlStatus; /* cameraControl Status */
  unsigned char historyStatus; /* onboard computers history status */
  unsigned char batteryStatus; /*  */
  unsigned char uplinkStatus[2]; /* uplink status */
  unsigned char lostComm; /* lost comm triggered */
  unsigned char hokuyoLaserStatus; /* Hokuyo laser status  */
  unsigned char ubloxSNR; /* average */
  unsigned char ubloxHacc; /* (dm) horizontal accuracy */
  unsigned char ubloxSacc; /* (dm/s) speed accuracy */
  unsigned char ubloxPDOP; /* (*10) */
  float pan; /* (deg) */
  float tilt; /* (deg) */
  float roll; /* (deg) */
  float fovy; /* (deg) */
  float G; /* load factor */
  float wind[3]; /* (ft/sec) */
  float pointPos[3]; /* (ft) */
  float datumLat; /* datum latitude (deg-N) */
  float datumLon; /* datum longitude (deg-E) */
  float datumAlt; /* datum altitude (ft) */
};

struct datalinkMessageUp0_ref {
  unsigned char sync1; /*  */
  unsigned char sync2; /*  */
  unsigned char sync3; /*  */
  unsigned char spare; /*  */
  int messageID; /* id # */
  int messageSize; /* including header */
  unsigned int hcsum; /*  */
  unsigned int csum; /*  */
  int k; /* discrete time onboard */
  float time; /* onboard time */
  float throttleLever; /*  */
  float rollStick; /*  */
  float pitchStick; /*  */
  float rudderPedal; /*  */
  char button[16]; /* button flags */
};

struct datalinkMessageMotorCmd_ref {
  unsigned char sync1; /*  */
  unsigned char sync2; /*  */
  unsigned char sync3; /*  */
  unsigned char spare; /*  */
  int messageID; /* id # */
  int messageSize; /* including header */
  unsigned int hcsum; /*  */
  unsigned int csum; /*  */
  int k; /* discrete time onboard */
  float time; /* onboard time */
  unsigned int motor_cmd[4]; /* motor command information */
};

struct datalinkMessageRCChannel_ref {
  unsigned char sync1; /*  */
  unsigned char sync2; /*  */
  unsigned char sync3; /*  */
  unsigned char spare; /*  */
  int messageID; /* id # */
  int messageSize; /* including header */
  unsigned int hcsum; /*  */
  unsigned int csum; /*  */
  int k; /* discrete time onboard */
  float time; /* onboard time */
  unsigned int rc_channels[16]; /* rc channel information */
};

struct datalinkMessageAutopilotDels_ref {
  unsigned char sync1; /*  */
  unsigned char sync2; /*  */
  unsigned char sync3; /*  */
  unsigned char spare; /*  */
  int messageID; /* id # */
  int messageSize; /* including header */
  unsigned int hcsum; /*  */
  unsigned int csum; /*  */
  float time; /* onboard time */
  float c_delm[3]; /*  */
  float c_delf[1]; /*  */
  float c_delt[1]; /*  */
};

extern struct obDatalink_ref obDatalink;
extern struct datalinkHeader_ref obDatalinkMessageHeader;
extern struct datalinkMessage0_ref obDatalinkMessage0;
extern struct datalinkMessage1_ref obDatalinkMessage1;
extern struct datalinkMessageUp0_ref obDatalinkMessageUp0;
extern struct datalinkMessageMotorCmd_ref obDatalinkMessageMotorCmd;
extern struct datalinkMessageRCChannel_ref obDatalinkMessageRCChannel;
extern struct datalinkMessageAutopilotDels_ref obDatalinkMessageAutopilotDels;

uint32_t datalinkCheckSumCompute(unsigned char* buf, int32_t byteCount);
void datalinkCheckSumEncode(unsigned char* buf, uint32_t byteCount);

void readDatalink( WiFiUDP* wf );
void writeM0( WiFiUDP* wf );
void writeM1( WiFiUDP* wf );
void writeAutopilotDels( WiFiUDP* wf );
void writeRcChannels( WiFiUDP* wf, RC_PILOT* rc );

#endif
