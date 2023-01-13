package frc.robot.auto.Tuning254;

// import frc.robot.auto.Tuning254.AutoModeEndedException;
// import frc.robot.auto.Tuning254.CollectAccelerationDataAction;
// import frc.robot.auto.Tuning254.CollectVelocityDataAction;
// import frc.robot.auto.Tuning254.WaitAction;
import com.team254.lib.physics.DriveCharacterization;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.Tuning4915.CollectAccelerationData;
import frc.robot.auto.Tuning4915.WaitAction;
import frc.robot.subsystems.Drive254;

public class CharacterizeDriveBaseMode extends SequentialCommandGroup {
    private final boolean reverse;
    private final boolean turn;

    public CharacterizeDriveBaseMode(boolean reverse, boolean turn, Drive254 mDrive) {
        this.reverse = reverse;
        this.turn = turn;
        List<DriveCharacterization.DataPoint> velocityData = new ArrayList<>();
        List<DriveCharacterization.DataPoint> accelerationData = new ArrayList<>();

        addCommands(
            new RunCommand(() -> {
                new CollectVelocityDataAction(velocityData, reverse, turn);
            }),

            new WaitCommand(10.0),

            new RunCommand(() -> {
                new CollectAccelerationDataAction(accelerationData, reverse, turn);
            })
        );

        DriveCharacterization.CharacterizationConstants constants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);

        SmartDashboard.putNumber("ks", constants.ks);
        SmartDashboard.putNumber("kv", constants.kv);
        SmartDashboard.putNumber("ka", constants.ka);

        addRequirements(mDrive);
    }

}