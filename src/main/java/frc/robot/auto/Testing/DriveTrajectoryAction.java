// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.Testing;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;


public class DriveTrajectoryAction extends CommandBase {

    private static final Drive mDrive = new Drive();
    // private static final RobotState mRobotState = RobotState.getInstance();

    private final TrajectoryIterator<TimedState<Pose2dWithCurvature>> mTrajectory;
    private final boolean mResetPose;

    public DriveTrajectoryAction(Trajectory<TimedState<Pose2dWithCurvature>> trajectory) {
        this(trajectory, false);
    }


    public DriveTrajectoryAction(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, boolean resetPose) {
        mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
        mResetPose = resetPose;
    }

  // public DriveTrajectoryAction(Drive drive) {
  //   mDrive = drive;
  //   addRequirements(drive);
  // }


  @Override
  public void initialize() {
    // System.out.println("Starting trajectory! (length=" + mTrajectory.getRemainingProgress() + ")");
    // if (mResetPose) {
    //     mRobotState.reset(Timer.getFPGATimestamp(), mTrajectory.getState().state().getPose());
    // }
    mDrive.setTrajectory(mTrajectory);
  }


  @Override
  public void execute() {
    
  }


  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    if (mDrive.isDoneWithTrajectory()) {
      System.out.println("Trajectory finished");
      return true;
  }
    return false;
  }
}
