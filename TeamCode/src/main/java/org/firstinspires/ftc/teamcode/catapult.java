package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class catapult extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor left = hardwareMap.dcMotor.get("left");
        DcMotor right = hardwareMap.dcMotor.get("right");
        DcMotor shooter = hardwareMap.dcMotor.get("shooter");

        waitForStart();

        if(isStopRequested()) return;

        while (opModeIsActive()){

            if(gamepad1.a){
                shooter.setPower(-1);
            } else if(gamepad1.b){
                shooter.setPower(1);
            } else {
                shooter.setPower(0);
            }

            double y = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_y;


            double denominator = Math.max(Math.abs(y) + Math.abs(rx) , 1);

            double lp = (y+rx)/denominator;
            double rp = (y-rx)/denominator;

            left.setPower(lp);
            right.setPower(rp);

        }
    }
}