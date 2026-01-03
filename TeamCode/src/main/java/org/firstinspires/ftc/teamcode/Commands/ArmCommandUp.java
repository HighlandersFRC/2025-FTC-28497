package org.firstinspires.ftc.teamcode.Commands;

import static android.os.SystemClock.sleep;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class ArmCommandUp implements Command {

    private final ArmSubsystem arm;
    public final double target = 1820;

    public ArmCommandUp(ArmSubsystem arm) {
        this.arm = arm;
    }

    @Override
    public void start() {
        arm.setTargetPosition(-target);
    }

    @Override
    public void execute() {
        arm.runArmPID();
    }

    @Override
    public void end() {
        arm.stop();
        arm.resetEncoders();
        sleep(500);
    }

    @Override
    public boolean isFinished() {
        return   Math.abs(target) - Math.abs(arm.getCurrentPos()) <= 25;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return arm;
    }
}