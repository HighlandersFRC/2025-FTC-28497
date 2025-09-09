package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp
public class mecanumDrive extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontleft = hardwareMap.dcMotor.get("left_front");
        DcMotor frontright = hardwareMap.dcMotor.get("right_front");
        DcMotor backleft = hardwareMap.dcMotor.get("left_back");
        DcMotor backright = hardwareMap.dcMotor.get("right_back");


        backleft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if(isStopRequested()) return;

        while (opModeIsActive()){
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x*1.1;
            double rx = gamepad1.right_stick_x;


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx) , 1);

            double flp = (x+y+rx)/denominator;
            double blp = (y-x+rx)/denominator;
            double frp = (y-x-rx)/denominator;
            double brp = (y+x-rx)/denominator;

            frontleft.setPower(flp);
            frontright.setPower(frp);
            backleft.setPower(blp);
            backright.setPower(brp);


        }
    }
}