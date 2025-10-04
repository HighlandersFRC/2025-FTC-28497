package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.Subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.Robot;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;

import java.util.*;

public class CommandScheduler {
    private static CommandScheduler instance;
    private final List<Command> scheduledCommands = new ArrayList<>();
    private final Map<Subsystem, Command> activeSubsystemCommands = new HashMap<>();
    private Robot robot;
    private NewRobot newRobot;

    private CommandScheduler() {}

    public static CommandScheduler getInstance() {
        if (instance == null) {
            instance = new CommandScheduler();
        }
        return instance;
    }

    public void setRobot(Robot robot) {
        this.robot = robot;
    }

    public void setNewRobot(NewRobot robot) {
        this.newRobot = robot;
    }

    /** Schedule a command, cancelling conflicts and handling subsystems */
    public void schedule(Command command) {
        Subsystem requiredSubsystem = command.getRequiredSubsystem();

        if (requiredSubsystem != null) {
            Command activeCommand = activeSubsystemCommands.get(requiredSubsystem);

            // Prevent duplicate scheduling of the same command
            if (activeCommand == command) {
                RobotLog.d("Command already active, not rescheduling: " + command.getClass().getSimpleName());
                return;
            }

            // Cancel the currently active command if it's not a default
            if (activeCommand != null && !isDefaultCommand(activeCommand)) {
                cancel(activeCommand);
            }

            activeSubsystemCommands.put(requiredSubsystem, command);
        }

        // Cancel conflicting commands of the same class
        cancelConflicting(command);

        // Schedule and start new command if not already scheduled
        if (!scheduledCommands.contains(command)) {
            scheduledCommands.add(command);
            command.start();
            RobotLog.d("Command Scheduled: " + command.getClass().getSimpleName());
        }
    }

    /** Run all scheduled commands and handle lifecycle */
    public void run() {
        List<Command> finishedCommands = new ArrayList<>();

        for (Command command : new ArrayList<>(scheduledCommands)) {
            // Handle optional commands if present
            List<Command> optionalCommands = command.optionalCheck();
            if (optionalCommands != null && !optionalCommands.isEmpty()) {
                for (Command optCmd : optionalCommands) {
                    optCmd.execute();
                    if (optCmd.isFinished()) {
                        optCmd.end();
                        finishedCommands.add(optCmd);
                        RobotLog.d("Optional Command Finished: " + optCmd.getClass().getSimpleName());
                    }
                }
            } else {
                // Normal execution
                command.execute();
                if (command.isFinished()) {
                    command.end();
                    finishedCommands.add(command);
                    RobotLog.d("Command Finished: " + command.getClass().getSimpleName());

                    Subsystem subsystem = command.getRequiredSubsystem();
                    if (subsystem != null) {
                        activeSubsystemCommands.remove(subsystem);

                        // Reschedule default command if needed
                        Command defaultCommand = subsystem.getDefaultCommand();
                        if (defaultCommand != null &&
                                !activeSubsystemCommands.containsKey(subsystem) &&
                                !isCommandScheduled(defaultCommand)) {
                            schedule(defaultCommand);
                        }
                    }
                }
            }
        }

        scheduledCommands.removeAll(finishedCommands);

        // Ensure default commands for all subsystems
        for (Subsystem subsystem : getAllSubsystems()) {
            if (!activeSubsystemCommands.containsKey(subsystem)) {
                Command defaultCommand = subsystem.getDefaultCommand();
                if (defaultCommand != null && !isCommandScheduled(defaultCommand)) {
                    schedule(defaultCommand);
                }
            }
        }
    }

    /** Cancel a single command */
    public void cancel(Command command) {
        Subsystem requiredSubsystem = command.getRequiredSubsystem();
        if (requiredSubsystem != null) {
            activeSubsystemCommands.remove(requiredSubsystem);
        }

        if (scheduledCommands.remove(command)) {
            command.end();
            RobotLog.d("Command Cancelled: " + command.getClass().getSimpleName());
        }
    }

    /** Cancel all running commands */
    public void cancelAll() {
        for (Command command : new ArrayList<>(scheduledCommands)) {
            cancel(command);
        }
    }

    /** Remove duplicates by keeping only one instance of each command class */
    public void removeDuplicateCommands() {
        Set<String> seen = new HashSet<>();
        Iterator<Command> it = scheduledCommands.iterator();
        while (it.hasNext()) {
            Command cmd = it.next();
            String name = cmd.getClass().getSimpleName();
            if (seen.contains(name)) {
                it.remove();
                cmd.end();
                RobotLog.d("Duplicate Command Removed: " + name);
            } else {
                seen.add(name);
            }
        }
    }

    /** Print active subsystem-command pairs */
    public void printCurrentCommands() {
        RobotLog.d("===== Current Commands =====");
        for (Map.Entry<Subsystem, Command> entry : activeSubsystemCommands.entrySet()) {
            RobotLog.d("Subsystem: " + entry.getKey().getClass().getSimpleName() +
                    ", Command: " + entry.getValue().getClass().getSimpleName());
        }
        RobotLog.d("============================");
    }

    /** Cancel commands with the same class as newCommand */
    private void cancelConflicting(Command newCommand) {
        String newName = newCommand.getClass().getSimpleName();
        Iterator<Command> it = scheduledCommands.iterator();
        while (it.hasNext()) {
            Command existing = it.next();
            if (existing.getClass().getSimpleName().equalsIgnoreCase(newName)) {
                existing.end();
                it.remove();
                RobotLog.d("Conflicting Command Cancelled: " + existing.getClass().getSimpleName());
            }
        }
    }

    /** Collect all subsystems from both Robot and NewRobot */
    private Set<Subsystem> getAllSubsystems() {
        Set<Subsystem> subsystems = new HashSet<>();
        if (robot != null) {
            if (robot.drive != null) subsystems.add(robot.drive);

        }
        if (newRobot != null) {
            // add subsystems from NewRobot when defined
            // subsystems.add(newRobot.someSubsystem);
        }
        return subsystems;
    }

    public boolean isCommandScheduled(Command command) {
        Subsystem subsystem = command.getRequiredSubsystem();
        return subsystem != null && activeSubsystemCommands.get(subsystem) == command;
    }

    private boolean isDefaultCommand(Command command) {
        Subsystem subsystem = command.getRequiredSubsystem();
        return subsystem != null && subsystem.getDefaultCommand() == command;
    }
}
