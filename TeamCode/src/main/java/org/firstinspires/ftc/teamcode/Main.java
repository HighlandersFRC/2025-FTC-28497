package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Main extends LinearOpMode {
    private DcMotor motor;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "motor");

        waitForStart();

        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y; // invert so pushing up is positive
            motor.setPower(power);

            telemetry.addData("Motor Power", power);
            telemetry.update();
        }
    }
}
