/*  ATRVJR constants
 *  Modified from
 *  David Lu!! - 2/2010
 *  Modified from Player Driver
 *
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef ATRVJR_CONFIG_H
#define ATRVJR_CONFIG_H

// Odometery Constants
// ===================
// Arbitrary units per meter
#define ODO_DISTANCE_CONVERSION 90810
// Arbitrary units per radian
#define ODO_ANGLE_CONVERSION 37000

// Sonar Constants
// ===============
// Arbitrary units per meter (for sonar)
#define RANGE_CONVERSION 1476
#define SONAR_ECHO_DELAY 30000
#define SONAR_PING_DELAY 0
#define SONAR_SET_DELAY  0
#define SONAR_MAX_RANGE 100000

#define BODY_INDEX 0
#define BASE_INDEX 1

#warning Sonar, bumper and IR configurations are not updated for use with ATRVJR
const int SONARS_PER_BANK[] = {5, 8, 5, 8};
const int SONAR_RING_BANK_BOUND[] = {0, 4};
#define SONAR_MAX_PER_BANK 16

const int SONARS_PER_RING[] = {26, 0};
const float SONAR_RING_START_ANGLE[] = {45,0};
const float SONAR_RING_ANGLE_INC[] = {-15, 0};
const float SONAR_RING_DIAMETER[] = {.25, 0};
const float SONAR_RING_HEIGHT[] = {0.36, 0};


/* Following variables are defined for ATRVJr and the values are ported from its Player version 
 * Number of Sonars is 26
 *
 */
const float SONAR_RING_ANGLES[] = {-2.617993, -2.879793, 3.141592, 2.879793, 2.617993, 0.785398, 1.047197, 1.308997, 1.570796, 1.570796, 1.832595, 2.094395, 2.356194, 0.523599, 0.261799, 0.000000, -0.261799, -0.523599, -0.785398, -1.047197, -1.308997, -1.570796, -1.570796, -1.832595, -2.094395, -2.356194};

const float SONAR_RING_X_DISTANCE[] = {-334.950, -340.410, -347.060, -340.410, -334.950, 230.230, 172.490, 117.200, 72.260, -72.260, -117.200, -172.490, -230.230, 296.200, 301.500, 308.100, 301.500, 296.200, 230.230, 172.490, 117.200, 72.260, -72.260, -117.200, -172.490, -230.230};

const float SONAR_RING_Y_DISTANCE[] = {-104.390, -49.910, 0.0000, 49.910, 104.390, 175.000, 178.600, 181.100, 181.100, 181.100, 181.100, 178.600, 175.000, 103.600, 49.500, 0.0000, -49.500, -103.600, -175.000, -178.600, -181.100, -181.100, -181.100, -181.100, -178.600, -175.000};


// Digital IO constants
// ====================
#define HOME_BEARING -32500

#define BUMPER_COUNT 0
#define BUMPER_ADDRESS_STYLE 0
#define BUMPER_BIT_STYLE 1
#define BUMPER_STYLE 0
const int BUMPERS_PER[] = {0,0};
const double BUMPER_ANGLE_OFFSET[] = {-1,1,-1,1};
const double BUMPER_HEIGHT_OFFSET[][4] = {{.5,.5,.05,.05},
    {.25,.25,.05,.05}
};

// IR Constants
// ============
#define IR_POSES_COUNT 0
#define IR_BASE_BANK 0
#define IR_BANK_COUNT 0
#define IR_PER_BANK 0

// Misc Constants
// ==============
#define USE_JOYSTICK 0
#define JOY_POS_RATIO 6
#define JOY_ANG_RATIO -0.01
#define POWER_OFFSET 1.2
#define PLUGGED_THRESHOLD 25.0

#endif









