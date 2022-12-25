// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.planners.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

public class Drive extends SubsystemBase {



  public enum DriveControlState {
    OPEN_LOOP, // open loop voltage control,
    VELOCITY, // velocity control
    PATH_FOLLOWING
}

private DriveControlState mDriveControlState;
private DriveMotionPlanner mMotionPlanner;
private boolean mOverrideTrajectory = false;
  

  public Drive() {
    // private final CANSparkMax mLeftMaster, mLeftSlave1, mLeftSlave2, mRightMaster, mRightSlave1, mRightSlave2;
    mMotionPlanner = new DriveMotionPlanner();
  }

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
