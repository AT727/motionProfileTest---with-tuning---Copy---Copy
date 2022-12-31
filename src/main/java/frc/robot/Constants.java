// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //gotta tune all of these hehe

    //WHEEL CONSTRAINTS
    public static final double kDriveWheelDiameterInches = 0; //
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kDriveWheelTrackWidthInches = 27.637795; //from our code
    public static final double kTrackScrubFactor = 1.0; //tune, but all teams have it as 1.0

    //DYNAMIC CONSTRAINTS
    public static final double kDriveLinearKv = 0; // V per rad/s  
    public static final double kDriveLinearKa = 0; // V / rad/s^2 
    public static final double kDriveVIntercept = 0; // V TODO 
    public static final double kRobotLinearInertia = 0; // kg TODO
    public static final double kRobotAngularInertia = 0; // kg m^2 TODO tune
    public static final double kRobotAngularDrag = 0; // N*m / (rad/sec)
    
    //PURE PURSUIT TUNING
    public static final double kPathKX = 4.0; // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4; // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0; // inches

    //TRAJECTORY GENERATOR CONSTRAINTS
    public static final double kMaxVel = 0;
    public static final double kMaxAccel = 0;
    public static final double kMaxCentripetalAccel = 0;
    public static final double kMaxVoltage = 0;

    //can spark and drive
    public static final int kRightDriveMasterId = 1;
    public static final int kRightDriveSlaveId1 = 2;
    public static final int kRightDriveSlaveId2 = 3;
    public static final int kLeftDriveMasterId = 4;
    public static final int kLeftDriveSlaveId1 = 5;
    public static final int kLeftDriveSlaveId2 = 6;
    public static final double kDriveVoltageRampRate = 0.0;

    //team 27 constants. used for limiting motors so they don't go crazy fast, SHOULD TUNE
    public static int MAX_PEAK_CURRENT = 37;
    public static int MAX_CONTINUOUS_CURRENT = 18;


    //control board
    public static final boolean kUseDriveGamepad = false; 
    public static final double kJoystickThreshold = 0.2;
    public static final int kButtonGamepadPort = 2;
    public static final int kMainTurnJoystickPort = 1; 
    public static final int kMainThrottleJoystickPort = 0; 
    public static final int kDriveGamepadPort = 0;


    public static final double kDriveEncoderPPR = 42.0;
    
    //kBeta = 1,50 in DriveMotionPlanner, we used 2.0
}
