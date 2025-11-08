package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Commands.Command;

public abstract class Subsystem {
    private final String name;
    private Command defaultCommand;

    public Subsystem() {
        this.name = this.getClass().getSimpleName(); // use class name automatically
    }

    public void periodic() {}

    public void setDefaultCommand(Command command) {
        this.defaultCommand = command;
    }

    public Command getDefaultCommand() {
        return defaultCommand;
    }

    public String getName() {
        return name;
    }
}
