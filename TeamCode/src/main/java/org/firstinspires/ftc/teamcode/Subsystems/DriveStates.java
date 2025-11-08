package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.PID;


public class DriveStates extends Subsystem {
    private DRIVE_STATE wantedState = DRIVE_STATE.IDLE;
    private DRIVE_STATE currentState = DRIVE_STATE.IDLE;
    private Drive drive;
    private PID xPID = new PID(1, 0, 0);
    private PID thetaPID = new PID(0.01, 0, 0.001);

    private double DISTANCE_TOLERANCE = 0.1;
    private double THETA_TOLERANCE = 2.0;

    private double targetDistance;
    private double targetTheta;

    public DriveStates(String name) {

    }

    public void init(HardwareMap hardwareMap) {
        this.drive = new Drive("drive", hardwareMap);
        Mouse.init(hardwareMap);
        Mouse.configureOtos();

        xPID.setMinOutput(-1);
        xPID.setMaxOutput(1);
        thetaPID.setMinOutput(-1);
        thetaPID.setMaxOutput(1);
    }

    public enum DRIVE_STATE {
        DEFAULT,
        IDLE,
        DRIVE_FORWARD,
        DRIVE_TURN
    }

    public void setWantedState(DRIVE_STATE state) {
        wantedState = state;
    }

    private DRIVE_STATE handleTransitions() {
        currentState = wantedState;
        return currentState;
    }

    public void driveForward(double distanceMeters) {
        Mouse.configureOtos();
        this.targetDistance = distanceMeters;
        setWantedState(DRIVE_STATE.DRIVE_FORWARD);
    }

    public void turnTo(double degrees) {
        targetTheta = normalizeAngle(Mouse.getTheta() + degrees);
        setWantedState(DRIVE_STATE.DRIVE_TURN);
    }

    private void handleDriveForward() {
        xPID.setSetPoint(targetDistance);
        xPID.updatePID(Mouse.getX());
        double power = xPID.getResult();
        drive.drive(power, power, power, power);
    }

    private void handleTurn() {
        thetaPID.setSetPoint(targetTheta);
        thetaPID.updatePID(Mouse.getTheta());
        double power = thetaPID.getResult();
        drive.drive(-power, power, -power, power);
    }

    @Override
    public void periodic() {
        Mouse.update();
        handleTransitions();
        switch (currentState) {
            case DRIVE_FORWARD:
                handleDriveForward();
                break;
            case DRIVE_TURN:
                handleTurn();
                break;
            case IDLE:
            case DEFAULT:
            default:
                drive.stop();
                break;
        }
    }

    public boolean isFinishedForward() {
        return Math.abs(Mouse.getX() - targetDistance) <= DISTANCE_TOLERANCE;
    }

    public boolean isFinishedTurn() {
        double error = angleError(targetTheta, Mouse.getTheta());
        return Math.abs(error) <= THETA_TOLERANCE;
    }

    private double normalizeAngle(double angle) {
        angle %= 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    private double angleError(double target, double current) {
        double error = target - current;
        error = ((error + 180) % 360 + 360) % 360 - 180;
        return error;
    }
}
