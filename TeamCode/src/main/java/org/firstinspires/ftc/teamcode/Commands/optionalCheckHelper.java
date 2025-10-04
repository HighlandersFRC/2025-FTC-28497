package org.firstinspires.ftc.teamcode.Commands;

public class optionalCheckHelper {
    public final Command command;
    public final VarArgBooleanFunction condition;

    public optionalCheckHelper(Command command, VarArgBooleanFunction condition) {
        this.command = command;
        this.condition = condition;
    }
}
