package frc.robot.auto.Testing;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class radiusAuto extends SequentialCommandGroup{
    public radiusAuto(Drive mDrive){
        addCommands(
        new RunCommand(() -> {
         while(true){
            SmartDashboard.putNumber("left encoder: ", mDrive.leftEncoder.getPosition());
            SmartDashboard.putNumber("left encoder: ", mDrive.rightEncoder.getPosition());
            SmartDashboard.putNumber("Left PPR: ", mDrive.leftEncoder.getCountsPerRevolution());
            SmartDashboard.putNumber("right PPR: ", mDrive.rightEncoder.getCountsPerRevolution());
         }
        })
        );

        addRequirements(mDrive);
    }
}
//