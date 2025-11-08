package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;

public class NewRobot {

    public Drive drive;
    private Telemetry telemetry;

    public NewRobot(HardwareMap hardwareMap) {
        // Initialize subsystems directly here if you want minimal setup
        drive = new Drive(hardwareMap);
    }

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Create subsystem instances here if not in constructor
        if (drive == null) {
            drive = new Drive(hardwareMap);
        }

        telemetry.addLine("Robot Initialized");
        telemetry.update();
    }

    public void run() {
        // Called in loop (can add periodic calls here)
        if (drive != null) drive.periodic();
    }
}

