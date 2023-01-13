package frc.robot.controlboard;

public interface IDriveControlBoard {
    double getThrottle();

    double getTurn();

    boolean getQuickTurn();

    boolean getShoot();

    boolean getWantsLowGear();

    boolean getThrust();
}