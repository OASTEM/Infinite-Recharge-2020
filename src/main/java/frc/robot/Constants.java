/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double regMode = 1.0;
    public static final double slowMode = 0.45;

    public static final int dPosFactor = 42;

    public static final double dMinOutput = -0.4;
    public static final double dMaxOutput = 0.4;

    public static final int dPos_Slot = 0;
    public static final double dPos_kP = 0.7;
    public static final double dPos_kI = 0.0;
    public static final double dPos_kD = 0.0;
    public static final double dPos_kF = 0.0;

    public static final double dClosedLoop_Ramp = 1.0;
    public static final double dOpenLoop_Ramp = 0.0;

    public static final int dSmart_Motion_Slot = 1;
    public static final double dSmart_Motion_kP = 1.0;
    public static final double dSmart_Motion_kI = 0.0;
    public static final double dSmart_Motion_kD = 0.0;
    public static final double dSmart_Motion_KF = 0.0;

    public static final int dSmart_Motion_Min_Accel = 100;
    public static final int dSmart_Motion_Min_Velocity = 100;
    public static final int dSmart_motion_Max_Velocity = 200; 

    public static final int dSmart_Motion_Allowed_Error = 100;   
    
    public static final int cPos_Slot = 0;
    public static final double cPos_kP = 0.0000001;
    public static final double cPos_kI = 0.0;
    public static final double cPos_kD = 0.0;
    public static final double cPos_kF = 0.0;
}

