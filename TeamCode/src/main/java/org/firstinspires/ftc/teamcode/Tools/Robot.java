
package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystem.Drive;



import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Robot {

    // Instance variables for subsystems
    public Drive drive;


    public Robot(HardwareMap hardwareMap) {
        this.drive = new Drive("drive", hardwareMap);

    }

    public void run() {

    }

    // Initialize hardware for all subsystems
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.drive = new Drive("drive", hardwareMap);

    }

}