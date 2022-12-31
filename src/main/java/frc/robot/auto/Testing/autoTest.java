// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.Testing;

import frc.robot.subsystems.Drive;
import frc.robot.auto.Testing.DriveTrajectoryAction;
import frc.robot.paths.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;


public class autoTest extends SequentialCommandGroup  {
  public autoTest(Drive mDrive){
    addCommands(
    new RunCommand(() -> {
      new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().testTrajectory);
    })
    );
    addRequirements(mDrive);
}
}

//{new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().testTrajectory

