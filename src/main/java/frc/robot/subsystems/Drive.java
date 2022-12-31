// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team254.lib.util.Util;
// import com.team254.lib.util.DriveSignal;
import com.spartronics4915.lib.util.DriveSignal;
import frc.robot.Constants;
import frc.robot.Kinematics;
import com.team254.lib.geometry.Twist2d;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import frc.robot.planners.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;


public class Drive extends SubsystemBase {
  
  private final CANSparkMax mLeftMaster, mLeftSlave1, mLeftSlave2, mRightMaster, mRightSlave1, mRightSlave2;
  private boolean mIsBrakeMode;
  private DriveControlState mDriveControlState;
  private PeriodicIO mPeriodicIO;
  private DriveMotionPlanner mMotionPlanner;
  private boolean mOverrideTrajectory = false;
  
  private final RelativeEncoder rightEncoder = mRightMaster.getEncoder();
  private final RelativeEncoder leftEncoder = mLeftMaster.getEncoder();

  public enum DriveControlState {
    OPEN_LOOP, // open loop voltage control
    PATH_FOLLOWING, // velocity PID control
  }

  public Drive() {
    mPeriodicIO = new PeriodicIO();
    mMotionPlanner = new DriveMotionPlanner();

    //left master
    mLeftMaster = new CANSparkMax(Constants.kLeftDriveMasterId, MotorType.kBrushless);
    mLeftMaster.restoreFactoryDefaults();
    mLeftMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);
    mLeftMaster.setOpenLoopRampRate(0.3); //cheesy Poofs have it as 0.0
    mLeftMaster.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0); //should change

    //left slaves
    mLeftSlave1 = new CANSparkMax(Constants.kLeftDriveSlaveId1, MotorType.kBrushless);
    mLeftSlave1.restoreFactoryDefaults();

    mLeftSlave1.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0);
    mLeftSlave1.follow(mLeftMaster);

    mLeftSlave2 = new CANSparkMax(Constants.kLeftDriveSlaveId2, MotorType.kBrushless);
    mLeftSlave2.restoreFactoryDefaults();

    mLeftSlave2.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0);
    mLeftSlave2.follow(mLeftMaster);

    //right master
    mRightMaster = new CANSparkMax(Constants.kRightDriveMasterId, MotorType.kBrushless);
    mRightMaster.restoreFactoryDefaults();
    mRightMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);
    mRightMaster.setOpenLoopRampRate(0.3); //cheesy Poofs have it as 0.0
    mRightMaster.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0); //should change

    //right slaves
    mRightSlave1 = new CANSparkMax(Constants.kRightDriveSlaveId1, MotorType.kBrushless);
    mRightSlave1.restoreFactoryDefaults();

    mRightSlave1.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0);
    mRightSlave1.follow(mRightMaster);

    mRightSlave2 = new CANSparkMax(Constants.kRightDriveSlaveId2, MotorType.kBrushless);
    mRightSlave2.restoreFactoryDefaults();

    mRightSlave2.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0);
    mRightSlave2.follow(mRightMaster);

    //invert drive motors
    mRightMaster.setInverted(true);
    mRightSlave1.setInverted(true);
    mRightSlave2.setInverted(true);
    mLeftMaster.setInverted(false);
    mLeftSlave1.setInverted(false);
    mLeftSlave2.setInverted(false);


    resetEncoders();
    
     // force a CAN message across
     mIsBrakeMode = true;
     setBrakeMode(false); 
     

  };

  public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
    if (mMotionPlanner != null) {
        mOverrideTrajectory = false;
        mMotionPlanner.reset();
        mMotionPlanner.setTrajectory(trajectory);
        mDriveControlState = DriveControlState.PATH_FOLLOWING;
    }
}

public boolean isDoneWithTrajectory() {
  if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
      return false;
  }
  return mMotionPlanner.isDone() || mOverrideTrajectory;
}


public double getLeftVelocityTicksPer100ms()
{
    return mPeriodicIO.leftVelocityTicksPer100ms;
}

public double getRightVelocityTicksPer100ms()
{
    return mPeriodicIO.rightVelocityTicksPer100ms;
}

public double getLeftOutputVoltage()
{
    return mPeriodicIO.leftVoltage;
}

public double getRightOutputVoltage()
{
    return mPeriodicIO.rightVoltage;
}


// private static double rotationsToInches(double rotations) {
//   return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
// }


// public double getLeftVelocityNativeUnits() {
//   return mPeriodicIO.left_velocity_ticks_per_100ms;
// }

//   public double getLeftLinearVelocity() {
//     return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / Constants.kDriveEncoderPPR);
// }

//   public synchronized void setCheesyishDrive(double throttle, double wheel, boolean quickTurn) {

//     if (Util.epsilonEquals(throttle, 0.0, 0.04)) {
//         throttle = 0;
//     }

//     if (Util.epsilonEquals(wheel, 0.0, 0.035)) {
//         wheel = 0.0;
//     }

//     final double kWheelGain = 0.05;
//     final double kWheelNonlinearity = 0.2;
//     final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
//     // Apply a sin function that's scaled to make it feel better.
//     if (!quickTurn) {
//         wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
//         wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
//         wheel = wheel / (denominator * denominator) * Math.abs(throttle);
//     }
    
//     wheel *= kWheelGain;
//     DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
//     double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
//     setOpenLoop(new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
// }

public synchronized void setOpenLoop(DriveSignal signal) {
  if (mDriveControlState != DriveControlState.OPEN_LOOP) {
      setBrakeMode(true);
      System.out.println("switching to open loop");
      System.out.println(signal);
      mDriveControlState = DriveControlState.OPEN_LOOP;
  }

  mPeriodicIO.left_demand = signal.getLeft();
  mPeriodicIO.right_demand = signal.getRight();
  mPeriodicIO.left_feedforward = 0.0;
  mPeriodicIO.right_feedforward = 0.0;
}


private void setCurrentLimiting(boolean shouldCurrentLimit) {

  //we already setSmartCurrentLimit in drive(), but I guess they have this to make sure it's always limited

  if (shouldCurrentLimit) {
    mLeftMaster.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0);
    mLeftSlave1.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0);
    mLeftSlave2.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0);

    mRightMaster.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0);
    mRightSlave1.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0);
    mRightSlave2.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0);
  } else {
    mLeftMaster.setSmartCurrentLimit(100);
    mLeftSlave1.setSmartCurrentLimit(100);
    mLeftSlave2.setSmartCurrentLimit(100);

    mRightMaster.setSmartCurrentLimit(100);
    mRightSlave1.setSmartCurrentLimit(100);
    mRightSlave2.setSmartCurrentLimit(100);
  }
}

public synchronized void setBrakeMode(boolean shouldEnable) {

//mIsBrakeMode is set to false in drive(), so all this runs 

  if (mIsBrakeMode != shouldEnable) {
      mIsBrakeMode = shouldEnable;
      IdleMode mode = shouldEnable ? IdleMode.kBrake : IdleMode.kCoast;
      mRightMaster.setIdleMode(mode);
      mRightSlave1.setIdleMode(mode);
      mRightSlave2.setIdleMode(mode);

      mLeftMaster.setIdleMode(mode);
      mLeftSlave1.setIdleMode(mode);
      mLeftSlave2.setIdleMode(mode);

      //brake mode shorts motor wires -> makes robot stop faster
      //coast mode disconnects motor wires -> motor deccerlates at it's rate

  }
}

// public synchronized void readPeriodicInputs() {
//   mPeriodicIO.left_velocity_ticks_per_100ms = mLeftMaster.getEncoder().getVelocity();
// }


public static class PeriodicIO {
  // INPUTS
  public double timestamp;
  public double leftVoltage;
  public double rightVoltage;
  public int left_position_ticks;
  public int right_position_ticks;
  public double left_distance;
  public double right_distance;
  public int leftVelocityTicksPer100ms;
  public int rightVelocityTicksPer100ms;

  // OUTPUTS
  public double left_demand;
  public double right_demand;
  public double left_accel;
  public double right_accel;
  public double left_feedforward;
  public double right_feedforward;
}


  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }
}
