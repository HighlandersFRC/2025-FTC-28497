package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class rotate implements Command {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    public IMU imu;
    double target;
    PID drivePID = new PID(0.02,0.0001,0.002);
    public rotate(double target) {
        this.target = target;
        drivePID.setSetPoint(target);
    }

    @Override
    public void start() {
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double result = drivePID.updatePID(yaw);
        leftFront.setPower(result);
        rightFront.setPower(-result);
        leftBack.setPower(-result);
        rightBack.setPower(-result);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
    @Override
    public boolean isFinished() {
        if (target > 0) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return null;
    }
}
