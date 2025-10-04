package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystem.Subsystem;

import java.util.ArrayList;
import java.util.List;

public interface Command {

    void start();

    void execute();

    void end();

    boolean isFinished();

    Subsystem getRequiredSubsystem();

    /**
     * Optional method that accepts multiple (Command, boolean function) pairs,
     * each with any number of arguments.
     *
     * @param argsForAll   Arguments to pass to all boolean functions.
     * @param conditionals Array of optionalCheckHelper objects.
     * @return true if ALL boolean functions return true, false otherwise.
     */
    default List<Command> optionalCheck(Object[] argsForAll, optionalCheckHelper... conditionals) {
        return optionalCheck(argsForAll, null, conditionals); // defaultCommand is null if not given
    }

    default List<Command> optionalCheck(Object[] argsForAll, Command defaultCommand, optionalCheckHelper... conditionals) {
        List<Command> matchingCommands = new ArrayList<>();

        if (conditionals != null) {
            for (optionalCheckHelper cc : conditionals) {
                if (cc.condition == null || cc.condition.apply(argsForAll)) {
                    matchingCommands.add(cc.command);
                    return matchingCommands;

                }
            }
        }

        if (matchingCommands.isEmpty() && defaultCommand != null) {
            matchingCommands.add(defaultCommand);
        }

        return matchingCommands;
    }


    default List<Command> optionalCheck() {
        return optionalCheck(new Object[]{}, null);
    }
}