// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team254.lib.util.Util;
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


public class Drive4915 extends SubsystemBase {
  
  private final CANSparkMax mLeftLeader, mLeftFollower1, mLeftFollower2, mRightLeader, mRightFollower1, mRightFollower2;
  private boolean mIsBrakeMode;
  private DriveControlState mDriveControlState;
  private PeriodicIO mPeriodicIO;
  private DriveMotionPlanner mMotionPlanner;
  private boolean mOverrideTrajectory = false;


  public enum DriveControlState {
    OPEN_LOOP, // open loop voltage control
    PATH_FOLLOWING, // velocity PID control
  }

  public Drive4915() {
    mPeriodicIO = new PeriodicIO();
    mMotionPlanner = new DriveMotionPlanner();

    //left master
    mLeftLeader = new CANSparkMax(Constants.kLeftDriveMasterId, MotorType.kBrushless);
    mLeftLeader.restoreFactoryDefaults();
    mLeftLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);
    mLeftLeader.setOpenLoopRampRate(0.3); //cheesy Poofs have it as 0.0
    mLeftLeader.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0); //should change

    //left slaves
    mLeftFollower1 = new CANSparkMax(Constants.kLeftDriveSlaveId1, MotorType.kBrushless);
    mLeftFollower1.restoreFactoryDefaults();

    mLeftFollower1.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0);
    mLeftFollower1.follow(mLeftLeader);

    mLeftFollower2 = new CANSparkMax(Constants.kLeftDriveSlaveId2, MotorType.kBrushless);
    mLeftFollower2.restoreFactoryDefaults();

    mLeftFollower2.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0);
    mLeftFollower2.follow(mLeftLeader);

    //right master
    mRightLeader = new CANSparkMax(Constants.kRightDriveMasterId, MotorType.kBrushless);
    mRightLeader.restoreFactoryDefaults();
    mRightLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);
    mRightLeader.setOpenLoopRampRate(0.3); //cheesy Poofs have it as 0.0
    mRightLeader.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0); //should change

    //right slaves
    mRightFollower1 = new CANSparkMax(Constants.kRightDriveSlaveId1, MotorType.kBrushless);
    mRightFollower1.restoreFactoryDefaults();

    mRightFollower1.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0);
    mRightFollower1.follow(mRightLeader);

    mRightFollower2 = new CANSparkMax(Constants.kRightDriveSlaveId2, MotorType.kBrushless);
    mRightFollower2.restoreFactoryDefaults();

    mRightFollower2.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0);
    mRightFollower2.follow(mRightLeader);

    //invert drive motors
    mRightLeader.setInverted(true);
    mRightFollower1.setInverted(true);
    mRightFollower2.setInverted(true);
    mLeftLeader.setInverted(false);
    mLeftFollower1.setInverted(false);
    mLeftFollower2.setInverted(false);

    
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
    mLeftLeader.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0);
    mLeftFollower1.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0);
    mLeftFollower2.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0);

    mRightLeader.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0);
    mRightFollower1.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0);
    mRightFollower2.setSmartCurrentLimit(Constants.MAX_PEAK_CURRENT, Constants.MAX_CONTINUOUS_CURRENT, 0);
  } else {
    mLeftLeader.setSmartCurrentLimit(100);
    mLeftFollower1.setSmartCurrentLimit(100);
    mLeftFollower2.setSmartCurrentLimit(100);

    mRightLeader.setSmartCurrentLimit(100);
    mRightFollower1.setSmartCurrentLimit(100);
    mRightFollower2.setSmartCurrentLimit(100);
  }
}

public synchronized void setBrakeMode(boolean shouldEnable) {

//mIsBrakeMode is set to false in drive(), so all this runs 

  if (mIsBrakeMode != shouldEnable) {
      mIsBrakeMode = shouldEnable;
      IdleMode mode = shouldEnable ? IdleMode.kBrake : IdleMode.kCoast;
      mRightLeader.setIdleMode(mode);
      mRightFollower1.setIdleMode(mode);
      mRightFollower2.setIdleMode(mode);

      mLeftLeader.setIdleMode(mode);
      mLeftFollower1.setIdleMode(mode);
      mLeftFollower2.setIdleMode(mode);

      //brake mode shorts motor wires -> makes robot stop faster
      //coast mode disconnects motor wires -> motor deccerlates at it's rate

  }
}

// public synchronized void readPeriodicInputs() {
//   mPeriodicIO.left_velocity_ticks_per_100ms = mLeftLeader.getEncoder().getVelocity();
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
