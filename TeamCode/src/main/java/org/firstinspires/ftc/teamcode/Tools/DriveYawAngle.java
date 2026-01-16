package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveYawAngle {

    private Drive drive;
    private IMU imu;
    private PID yawPID;
    private double targetYaw;

    public DriveYawAngle(HardwareMap hardwareMap, Drive driveInstance, double initialTargetYaw) {
        drive = driveInstance;

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        yawPID = new PID(0.01, 0, 0.1);
        yawPID.setMinInput(-180);
        yawPID.setMaxInput(180);
        yawPID.setContinuous(true);

        targetYaw = initialTargetYaw;
        yawPID.setSetPoint(targetYaw);
    }

    public void setTargetYaw(double yaw) {
        targetYaw = yaw;
        yawPID.setSetPoint(targetYaw);
    }

    public double getCurrentYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void updateDrive(double forwardInput, double turnInput) {
        double currentYaw = getCurrentYaw();
        double correction;

        if (Math.abs(turnInput) > 0.05) {
            correction = turnInput;
            targetYaw = currentYaw;
            yawPID.setSetPoint(targetYaw);
        } else {
            correction = yawPID.updatePID(currentYaw);
        }

        double leftPower = forwardInput + correction;
        double rightPower = forwardInput - correction;

        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        drive.leftMotor.setPower(leftPower);
        drive.rightMotor.setPower(rightPower);
    }

    public boolean isAtTarget(double toleranceDegrees) {
        return Math.abs(yawPID.getError()) <= toleranceDegrees;
    }

    public void stop() {
        drive.leftMotor.setPower(0);
        drive.rightMotor.setPower(0);
    }
}
