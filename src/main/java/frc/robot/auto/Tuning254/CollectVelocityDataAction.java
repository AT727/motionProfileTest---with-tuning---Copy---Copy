package frc.robot.auto.Tuning254;

import frc.robot.Constants;
import frc.robot.subsystems.Drive254;
import com.team254.lib.physics.DriveCharacterization;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.ReflectingCSVWriter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.List;

public class CollectVelocityDataAction extends CommandBase {
    private static final double kMaxPower = 0.25;
    private static final double kRampRate = 0.02;
    private static final Drive254 mDrive = new Drive254();

    private final ReflectingCSVWriter<DriveCharacterization.DataPoint> mCSVWriter;
    private final List<DriveCharacterization.DataPoint> mVelocityData;
    private final boolean mTurn;
    private final boolean mReverse;

    private boolean isFinished = false;
    private double mStartTime = 0.0;

    /**
     * @param data     reference to the list where data points should be stored
     * @param reverse  if true drive in reverse, if false drive normally
     * @param turn     if true turn, if false drive straight
     */
    public CollectVelocityDataAction(List<DriveCharacterization.DataPoint> data, boolean reverse, boolean turn) {
        mVelocityData = data;
        mReverse = reverse;
        mTurn = turn;
        mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/VELOCITY_DATA.csv", DriveCharacterization.DataPoint.class);
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        synchronized (mDrive) {
            double dt = Timer.getFPGATimestamp() - mStartTime;
            double percentPower = kRampRate * dt;
            if (percentPower > kMaxPower) {
                isFinished = true;
                return;
            }
            mDrive.setOpenLoop(new DriveSignal((mReverse ? -1.0 : 1.0) * percentPower, (mReverse ? -1.0 : 1.0) * (mTurn ? -1.0 : 1.0) * percentPower));
            double velocity = Math.abs(mDrive.getLeftLinearVelocity()) / Constants.kDriveWheelRadiusInches;  // rad/s
            if (velocity < 0.5) {
                // Small velocities tend to be untrustworthy.
                return;
            }
            mVelocityData.add(new DriveCharacterization.DataPoint(
                    velocity, // rad/s
                    percentPower * 12.0, // convert to volts
                    dt
            ));
        }
        mCSVWriter.add(mVelocityData.get(mVelocityData.size() - 1));
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interuppted) {
        mDrive.setOpenLoop(DriveSignal.BRAKE);
        mCSVWriter.flush();
    }
}