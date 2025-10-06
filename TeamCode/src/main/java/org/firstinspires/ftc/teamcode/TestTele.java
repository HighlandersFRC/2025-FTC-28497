package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystem.Drive;
import org.firstinspires.ftc.teamcode.Subsystem.Superstructure;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
@TeleOp(name="Safe Test TeleOp")
public class TestTele extends LinearOpMode {

    private Drive drive;
    private Superstructure superstructure;

    @Override
    public void runOpMode() {

        // Initialize hardware safely
        drive = new Drive("Drive", hardwareMap);
        superstructure = new Superstructure("Structure");

        

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        drive.setWantedState(Drive.Drive_State.DEFAULT);

        while (opModeIsActive()) {

            // Do NOT block here
            drive.periodic();
            superstructure.periodic();

            // Always safe, fast field-centric control
            drive.FeildCentric(gamepad1);

            // Optional telemetry
            telemetry.update();
        }
    }
}
