// package frc.robot.auto.Testing;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Drive;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class radiusCmd extends CommandBase{

//     private final Drive mDrive;

//     public radiusCmd(Drive mDrive){
//         this.mDrive = mDrive;
//         addRequirements(mDrive);
//     }

//     @Override
//     public void initialize() {}
  
  
//     @Override
//     public void execute() {
//         SmartDashboard.putNumber("left encoder: ", mDrive.leftEncoder.getPosition());
//         SmartDashboard.putNumber("right encoder: ", mDrive.rightEncoder.getPosition());
//         SmartDashboard.putNumber("Left PPR: ", mDrive.leftEncoder.getCountsPerRevolution());
//         SmartDashboard.putNumber("right PPR: ", mDrive.rightEncoder.getCountsPerRevolution());
//     }
  
  
//     @Override
//     public void end(boolean interrupted) {}
  
  
//     @Override
//     public boolean isFinished() {
//       return false;
//     }
// }
