#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
    BaseController::Init();
    
    // variables needed for integral control
    integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
    // Load params from simulator parameter system
    ParamsHandle config = SimpleConfig::GetInstance();
    
    // Load parameters (default to 0)
    kpPosXY = config->Get(_config+".kpPosXY", 0);
    kpPosZ = config->Get(_config + ".kpPosZ", 0);
    KiPosZ = config->Get(_config + ".KiPosZ", 0);
    
    kpVelXY = config->Get(_config + ".kpVelXY", 0);
    kpVelZ = config->Get(_config + ".kpVelZ", 0);
    
    kpBank = config->Get(_config + ".kpBank", 0);
    kpYaw = config->Get(_config + ".kpYaw", 0);
    
    kpPQR = config->Get(_config + ".kpPQR", V3F());
    
    maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
    maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
    maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
    maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);
    
    maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);
    
    minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
    maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
    // load params from PX4 parameter system
    //TODO
    param_get(param_find("MC_PITCH_P"), &Kp_bank);
    param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
    // Convert a desired 3-axis moment and collective thrust command to
    //   individual motor thrust commands
    // INPUTS:
    //   desCollectiveThrust: desired collective thrust [N]
    //   desMoment: desired rotation moment about each axis [N m]
    // OUTPUT:
    //   set class member variable cmd (class variable for graphing) where
    //   cmd.desiredThrustsN[0..3]: motor commands, in [N]
    
    // F1, F2, F3, F4: desired thrust for each rotor
    float l = L * M_SQRT1_2;
    float uBar = collThrustCmd; // F1 + F2 + F3 + F4
    float pBar = momentCmd.x / l; // F1 + F3 - F2 - F4
    float qBar = momentCmd.y / l; // F1 + F2 - F3 - F4
    float rBar = -momentCmd.z / kappa; // F1 - F2 + F4 - F3
    
    float f1 = (uBar + pBar + qBar + rBar) / 4.f; // front left
    float f2 = (uBar - pBar + qBar - rBar) / 4.f; // front right
    float f3 = (uBar + pBar - qBar - rBar) / 4.f; // rear left
    float f4 = (uBar - pBar - qBar + rBar) / 4.f; // rear right
    
    cmd.desiredThrustsN[0] = f1; // front left
    cmd.desiredThrustsN[1] = f2; // front right
    cmd.desiredThrustsN[2] = f3; // rear left
    cmd.desiredThrustsN[3] = f4; // rear right
    
    return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
    // Calculate a desired 3-axis moment given a desired and current body rate
    // INPUTS:
    //   pqrCmd: desired body rates [rad/s]
    //   pqr: current or estimated body rates [rad/s]
    // OUTPUT:
    //   return a V3F containing the desired moments for each of the 3 axes
    
    
    V3F momentCmd;
    
    V3F MOI = V3F(Ixx, Iyy, Izz);
    V3F pqrError = pqrCmd - pqr;
    V3F pqrDot = kpPQR * pqrError;
    momentCmd = MOI * pqrDot;
    
    return momentCmd;
}

// returns a desired roll and pitch rate
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
    // Calculate a desired pitch and roll angle rates based on a desired global
    //   lateral acceleration, the current attitude of the quad, and desired
    //   collective thrust command
    // INPUTS:
    //   accelCmd: desired acceleration in global XY coordinates [m/s2]
    //   attitude: current or estimated attitude of the vehicle
    //   collThrustCmd: desired collective thrust of the quad [N]
    // OUTPUT:
    //   return a V3F containing the desired pitch and roll rates. The Z
    //     element of the V3F should be left at its default value (0)
    
    V3F pqrCmd;
    Mat3x3F R = attitude.RotationMatrix_IwrtB();
    
    if (collThrustCmd > 0) {
        float u1 = collThrustCmd / mass;
        // Gate the tilt angle (approximately)
        float bx_targ = accelCmd.x / u1;
        float by_targ = accelCmd.y / u1;
        bx_targ = -CONSTRAIN(bx_targ, -maxTiltAngle, maxTiltAngle);
        by_targ = -CONSTRAIN(by_targ, -maxTiltAngle, maxTiltAngle);
        // Calculate angular velocity
        float bxd = kpBank * (bx_targ - R(0,2));
        float byd = kpBank * (by_targ - R(1,2));
        pqrCmd.x = (R(1,0) * bxd - R(1,1) * byd) / R(2,2);
        pqrCmd.y = (R(0,0) * bxd - R(0,1) * byd) / R(2,2);
    } else {
        pqrCmd.x = 0.0;
        pqrCmd.y = 0.0;
    }
    
    return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
    // Calculate desired quad thrust based on altitude setpoint, actual altitude,
    //   vertical velocity setpoint, actual vertical velocity, and a vertical
    //   acceleration feed-forward command
    // INPUTS:
    //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
    //   posZ, velZ: current vertical position and velocity in NED [m]
    //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
    //   dt: the time step of the measurements [seconds]
    // OUTPUT:
    //   return a collective thrust command in [N]
    
    
    Mat3x3F R = attitude.RotationMatrix_IwrtB();
    float thrust = 0;
    
    float zError = posZCmd - posZ;
    integratedAltitudeError += zError * dt;
    float zVelError = velZCmd - velZ;
    float p = kpPosZ * zError;
    float i = KiPosZ * integratedAltitudeError;
    float d = kpVelZ * zVelError;
    float u1Bar =  p + i + d + accelZCmd;
    
    thrust = (u1Bar - CONST_GRAVITY) / R(2,2);
    thrust = CONSTRAIN(thrust, -maxAscentRate / dt, maxDescentRate / dt);
    thrust = - mass * thrust;
    
    
    return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmd)
{
    // Calculate a desired horizontal acceleration based on
    //  desired lateral position/velocity/acceleration and current pose
    // INPUTS:
    //   posCmd: desired position, in NED [m]
    //   velCmd: desired velocity, in NED [m/s]
    //   pos: current position, NED [m]
    //   vel: current velocity, NED [m/s]
    //   accelCmd: desired acceleration, NED [m/s2]
    // OUTPUT:
    //   return a V3F with desired horizontal accelerations.
    //     the Z component should be 0
    
    // make sure we don't have any incoming z-component
    accelCmd.z = 0;
    velCmd.z = 0;
    posCmd.z = pos.z;
    
    if (velCmd.mag() > maxSpeedXY) {
        velCmd = velCmd.norm() * maxSpeedXY;
    }
    
    V3F posError = posCmd - pos;
    V3F velError = velCmd - vel;
    accelCmd = kpPosXY * posError + kpVelXY * velError + accelCmd;
    
    if (accelCmd.mag() > maxAccelXY) {
        accelCmd = accelCmd.norm() * maxAccelXY;
    }
    
    // make sure to return horizontal accelerations
    accelCmd.z = 0;
    
    return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
    // Calculate a desired yaw rate to control yaw to yawCmd
    // INPUTS:
    //   yawCmd: commanded yaw [rad]
    //   yaw: current yaw [rad]
    // OUTPUT:
    //   return a desired yaw rate [rad/s]
    
    float yawRateCmd=0;
    if (yawCmd > 0) {
        yawCmd = fmodf(yawCmd, M_2_PI);
    } else {
        yawCmd = fmodf(yawCmd, -M_2_PI);
    }
    
    float yawError = yawCmd - yaw;
    
    if (yawError > M_PI) {
        yawError = yawError - M_2_PI;
    } else if (yawError < -M_PI) {
        yawError = yawError + M_2_PI;
    }
    
    yawRateCmd = kpYaw * yawError;
    
    return yawRateCmd;
    
}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
    curTrajPoint = GetNextTrajectoryPoint(simTime);
    
    float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);
    
    // reserve some thrust margin for angle control
    float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
    collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
    
    V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
    
    V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
    desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());
    
    V3F desMoment = BodyRateControl(desOmega, estOmega);
    
    return GenerateMotorCommands(collThrustCmd, desMoment);
}
