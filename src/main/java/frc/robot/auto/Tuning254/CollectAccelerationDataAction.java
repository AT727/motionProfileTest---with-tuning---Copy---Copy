package frc.robot.auto.Tuning254;

import frc.robot.Constants;
import frc.robot.subsystems.Drive254;
import com.team254.lib.physics.DriveCharacterization;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.List;

public class CollectAccelerationDataAction extends CommandBase {
    private static final double kStartPower = 0.1;
    private static final double kPower = 0.8;
    private static final double kTotalTime = 2.0; // how long to run the test for
    private static final double kStartTime = 1.0;
    private static final Drive254 mDrive = new Drive254();

    private final ReflectingCSVWriter<DriveCharacterization.DataPoint> mCSVWriter;
    private final List<DriveCharacterization.DataPoint> mAccelerationData;
    private final boolean mTurn;
    private final boolean mReverse;

    private double mStartTime = 0.0;
    private double mPrevVelocity = 0.0;
    private double mPrevTime = 0.0;

    /**
     * @param data     reference to the list where data points should be stored
     * @param reverse  if true drive in reverse, if false drive normally
     * @param turn     if true turn, if false drive straight
     */


    public CollectAccelerationDataAction(List<DriveCharacterization.DataPoint> data, boolean reverse, boolean turn) {
        mAccelerationData = data;
        mReverse = reverse;
        mTurn = turn;
        mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/ACCEL_DATA.csv", DriveCharacterization.DataPoint.class);
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        mDrive.setOpenLoop(new DriveSignal((mReverse ? -1.0 : 1.0) * kStartPower, (mReverse ? -1.0 : 1.0) * (mTurn ? -1.0 : 1.0) * kStartPower));
        mStartTime = Timer.getFPGATimestamp();
        mPrevTime = mStartTime;
    }

    @Override
    public void execute() {
        double currentVelocity;
        double currentTime;
        synchronized (mDrive) {
            currentVelocity = Math.abs(mDrive.getLeftLinearVelocity()) / Constants.kDriveWheelRadiusInches; // rad/s
            currentTime = Timer.getFPGATimestamp();
        }

        // don't calculate acceleration until we've populated prevTime and prevVelocity
        if (mPrevTime == mStartTime) {
            mPrevTime = currentTime;
            mPrevVelocity = currentVelocity;
            return;
        }

        if (currentTime - mStartTime > kStartTime) {
            mDrive.setOpenLoop(new DriveSignal((mReverse ? -1.0 : 1.0) * kPower, (mReverse ? -1.0 : 1.0) * (mTurn ? -1.0 : 1.0) * kPower));
        } else {
            return;
        }

        double acceleration = (currentVelocity - mPrevVelocity) / (currentTime - mPrevTime);

        // ignore accelerations that are too small
        if (acceleration < Util.kEpsilon) {
            mPrevTime = currentTime;
            mPrevVelocity = currentVelocity;
            return;
        }

        mAccelerationData.add(new DriveCharacterization.DataPoint(
                currentVelocity, // convert to radians per second
                kPower * 12.0, // convert to volts
                currentTime - mStartTime
        ));

        mCSVWriter.add(mAccelerationData.get(mAccelerationData.size() - 1));

        mPrevTime = currentTime;
        mPrevVelocity = currentVelocity;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime > kTotalTime + kStartTime;
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.setOpenLoop(DriveSignal.BRAKE);
        mCSVWriter.flush();
    }
}