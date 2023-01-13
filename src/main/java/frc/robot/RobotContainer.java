// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.auto.Testing.autoTest;
import frc.robot.subsystems.Drive254;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.Tuning254.CharacterizeDriveBaseMode;
import frc.robot.util.XboxController;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drive254 mDrive = new Drive254();
  private final XboxController driverController = new XboxController(0);
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    this.mDrive.setDefaultCommand(new RunCommand(
      () -> this.mDrive.setCheesyishDrive(driverController.getThrottle(), driverController.getTurn(), driverController.getQuickTurn()),
    mDrive));
    
    this.configureButtonBindings();
    autoChooser.addOption("Characterize Drive Straight", new CharacterizeDriveBaseMode(false, false, mDrive));

  }

  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
