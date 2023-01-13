package frc.robot.controlboard;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;

public class MainDriveControlBoard implements IDriveControlBoard {
    private static MainDriveControlBoard mInstance = null;

    public static MainDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new MainDriveControlBoard();
        }

        return mInstance;
    }

    private final Joystick mThrottleStick;
    private final Joystick mTurnStick;

    private MainDriveControlBoard() {
        mThrottleStick = new Joystick(Constants.kMainThrottleJoystickPort);
        mTurnStick = new Joystick(Constants.kMainTurnJoystickPort);
    }

    @Override
    public double getThrottle() {
        return mThrottleStick.getRawAxis(1);
    }

    @Override
    public double getTurn() {

        return -mTurnStick.getRawAxis(0);
    }

    @Override
    public boolean getQuickTurn() {

        if(getThrottle() == 0){
            return true;
        }
        
        else{
            return false;
        }
    }

    @Override
    public boolean getShoot() {
        return mTurnStick.getRawButton(2);
    }

    @Override
    public boolean getWantsLowGear() {
        return mThrottleStick.getRawButton(2);
    }

    public boolean getThrust() {
        return mThrottleStick.getRawButton(1);
    }
}