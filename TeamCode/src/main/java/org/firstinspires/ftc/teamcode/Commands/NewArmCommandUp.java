package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.NewArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class NewArmCommandUp implements Command{
    NewArmSubsystem armSubsystem;
    public NewArmCommandUp(NewArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void start() {
      armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.ARM_UP);
    }
    @Override
    public void execute() {

    }

    @Override
    public void end() {
    armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.IDLE);
    }
    @Override
    public boolean isFinished() {
        return armSubsystem.isFinished();
    }
    @Override
    public Subsystem getRequiredSubsystem() {
        return armSubsystem;
    }
}
