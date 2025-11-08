package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;

import java.util.*;

public class CommandScheduler {
    private static CommandScheduler instance;
    private final List<Command> scheduledCommands = new ArrayList<>();
    private final Map<Subsystem, Command> activeSubsystemCommands = new HashMap<>();
    private NewRobot newRobot;

    public static CommandScheduler getInstance() {
        if (instance == null) instance = new CommandScheduler();
        return instance;
    }

    public void setNewRobot(NewRobot robot) {
        this.newRobot = robot;
    }

    public void schedule(Command command) {
        Subsystem requiredSubsystem = command.getRequiredSubsystem();
        if (requiredSubsystem != null) {
            Command active = activeSubsystemCommands.get(requiredSubsystem);
            if (active == command) return;
            if (active != null && !isDefaultCommand(active)) cancel(active);
            activeSubsystemCommands.put(requiredSubsystem, command);
        }
        if (!scheduledCommands.contains(command)) {
            scheduledCommands.add(command);
            command.start();
            RobotLog.d("Scheduled: " + command.getClass().getSimpleName());
        }
    }

    public void run() {
        List<Command> finished = new ArrayList<>();
        for (Command command : new ArrayList<>(scheduledCommands)) {
            if (command.isFinished()) {
                command.end();
                finished.add(command);
                Subsystem subsystem = command.getRequiredSubsystem();
                if (subsystem != null) {
                    activeSubsystemCommands.remove(subsystem);
                    Command def = subsystem.getDefaultCommand();
                    if (def != null && !isCommandScheduled(def)) schedule(def);
                }
            } else command.execute();
        }
        scheduledCommands.removeAll(finished);
    }

    public void cancel(Command command) {
        Subsystem sub = command.getRequiredSubsystem();
        if (sub != null) activeSubsystemCommands.remove(sub);
        command.end();
        scheduledCommands.remove(command);
    }

    public void cancelAll() {
        for (Command c : new ArrayList<>(scheduledCommands)) cancel(c);
        scheduledCommands.clear();
        activeSubsystemCommands.clear();
    }

    private boolean isDefaultCommand(Command c) {
        Subsystem s = c.getRequiredSubsystem();
        return s != null && s.getDefaultCommand() == c;
    }

    public boolean isCommandScheduled(Command c) {
        Subsystem s = c.getRequiredSubsystem();
        return s != null && activeSubsystemCommands.get(s) == c;
    }
}
