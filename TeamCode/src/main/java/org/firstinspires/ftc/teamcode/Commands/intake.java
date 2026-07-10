package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class intake implements Command{
    double startTime = 0;
    double time;
    double power;
    DcMotor intake;
    public intake (double power, double time) {
        this.time = time;
        this.power = power;
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
