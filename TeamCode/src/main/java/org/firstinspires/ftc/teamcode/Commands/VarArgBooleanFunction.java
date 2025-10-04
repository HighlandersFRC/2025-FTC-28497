package org.firstinspires.ftc.teamcode.Commands;

/**
 * Functional interface for a boolean-returning function
 * that can take any number of arguments.
 */
@FunctionalInterface
public interface VarArgBooleanFunction {
    boolean apply(Object... args);
}
