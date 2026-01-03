package org.firstinspires.ftc.teamcode.Commands;

import static android.os.SystemClock.sleep;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class ArmCommandDown implements Command {

    private final ArmSubsystem arm;
    public final double target = -1700;

    public ArmCommandDown(ArmSubsystem arm) {
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
        sleep(50);

    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getCurrentPos() - target) <= 25;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return arm;
    }
}