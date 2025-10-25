package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class drivingGeorge extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize motors
        DcMotor left = hardwareMap.get(DcMotor.class, "left");
        DcMotor right = hardwareMap.get(DcMotor.class, "right");

        waitForStart();

        while(opModeIsActive()) {

            double drive = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;

            double leftPower = drive + turn;
            double rightPower = drive - turn;

            left.setPower(leftPower);
            right.setPower(rightPower);
        }
    }
}