package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Subsystem {
    private String name;

    public Subsystem(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }
    public void periodic(){}

    public abstract void initialize(HardwareMap hardwareMap);
}