package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.ARM;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class intake implements Command{
    double startTime = 0;
    double time;
    double power;
    DcMotor intake;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;

    public intake (double power, double time) {
        this.time = time;
        this.power = power;
        DcMotor frontleft = hardwareMap.dcMotor.get("left_front");
        DcMotor frontright = hardwareMap.dcMotor.get("right_front");
        DcMotor backleft = hardwareMap.dcMotor.get("left_back");
        DcMotor backright = hardwareMap.dcMotor.get("right_back");
    }
    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        intake.setPower(power);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {
        intake.setPower(0);
    }

    @Override
    public boolean isFinished() {
        time = System.currentTimeMillis();
        if (time - startTime > this.time) {
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
