package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class runner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.dcMotor.get("left_front");
        DcMotor frontRight = hardwareMap.dcMotor.get("right_front");
        DcMotor backLeft = hardwareMap.dcMotor.get("left_back");
        DcMotor backRight = hardwareMap.dcMotor.get("right_back");

        waitForStart();
        while (opModeIsActive()) {

        }
    }
}
