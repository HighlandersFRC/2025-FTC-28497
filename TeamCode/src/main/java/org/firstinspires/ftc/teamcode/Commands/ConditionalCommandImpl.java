package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import java.util.function.BooleanSupplier;


public class ConditionalCommandImpl extends ConditionalCommand {
    public ConditionalCommandImpl(Command onTrue, Command onFalse, BooleanSupplier condition) {
        super( onTrue, onFalse, condition);
    }
}
