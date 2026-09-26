package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.SparkFunOTOS;

public class rotate implements Command {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    public SparkFunOTOS mouse;
    double target;
    double heading;
    PID drivePID = new PID(0.02,0.0001,0.002);
    double result;
    public rotate(double target) {
        this.target = target;
        drivePID.setSetPoint(target);
        this.heading = heading;
    }

    @Override
    public void start() {
        heading = mouse.getPosition().h;
        double error = target - heading;
        leftFront.setPower(result);
        rightFront.setPower(-result);
        leftBack.setPower(-result);
        rightBack.setPower(-result);
    }

    @Override
    public void execute() {
         this.result = drivePID.updatePID(heading);
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
        if (heading >= 90) {
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