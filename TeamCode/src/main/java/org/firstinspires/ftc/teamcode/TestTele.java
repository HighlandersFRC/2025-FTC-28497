package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystem.Drive;
import org.firstinspires.ftc.teamcode.Subsystem.Superstructure;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Vector;
@TeleOp(name="Safe Test TeleOp")
public class TestTele extends LinearOpMode {

    private Drive drive;
    private Superstructure superstructure;

    private PID xPID = new PID(3.6, 0, 1.9);
    private PID yPID = new PID(3.6, 0, 1.9);

    @Override
    public void runOpMode() {

        // Initialize hardware safely
        drive = new Drive("Drive", hardwareMap);
        superstructure = new Superstructure("Structure");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        xPID.setSetPoint(1);
        yPID.setSetPoint(0);


        waitForStart();
        Mouse.configureOtos();



        //drive.setWantedState(Drive.Drive_State.DEFAULT);

        while (opModeIsActive()) {

            // Do NOT block here
            //drive.periodic();
            superstructure.periodic();

            Mouse.update();


            xPID.updatePID(Mouse.getX());
            yPID.updatePID(Mouse.getY());

            // Always safe, fast field-centric control
            Vector vector = new Vector(xPID.getResult(),yPID.getResult());

            drive.resetMouse(gamepad1);
            drive.autoDrive(telemetry,vector,Mouse.getTheta());

            //


            telemetry.addData("mouse x",Mouse.getX());
            telemetry.addData("mouse Y",Mouse.getY());
            telemetry.addData("targetX",gamepad1.left_stick_x);
            telemetry.addData("targetY",gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}
