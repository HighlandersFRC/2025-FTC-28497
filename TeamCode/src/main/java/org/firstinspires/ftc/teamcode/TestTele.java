package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystem.Drive;
import org.firstinspires.ftc.teamcode.Subsystem.Superstructure;

@TeleOp
public class TestTele extends LinearOpMode {

    Superstructure superstructure = new Superstructure("Structure");

    @Override
    public void runOpMode() throws InterruptedException {

        Drive drive = new Drive("Drive",hardwareMap);

        waitForStart();
        drive.setWantedState(Drive.Drive_State.DEFAULT);

        while (opModeIsActive()) {

            drive.periodic();
            superstructure.periodic();

            drive.FeildCentric(gamepad1);

        }
    }
}
