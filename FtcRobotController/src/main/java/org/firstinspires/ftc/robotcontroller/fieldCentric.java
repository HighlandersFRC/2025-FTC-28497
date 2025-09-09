package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Mouse;
import org.firstinspires.ftc.teamcode.PID;


@TeleOp
public class fieldCentric extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontleft = hardwareMap.dcMotor.get("left_front");
        DcMotor frontright = hardwareMap.dcMotor.get("right_front");
        DcMotor backleft = hardwareMap.dcMotor.get("left_back");
        DcMotor backright = hardwareMap.dcMotor.get("right_back");


        backleft.setDirection(DcMotorSimple.Direction.REVERSE);

        Mouse.init(hardwareMap);

        waitForStart();

        if(isStopRequested()) return;

        while (opModeIsActive()){
            double x = -gamepad1.left_stick_x*2;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            double botHeading = -Math.toRadians(Mouse.getTheta());
            Mouse.update();
            if (gamepad1.options) {
                Mouse.configureOtos();
            }
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            double frontLeftPower = (-rotY + rotX + rx);
            double backLeftPower = (rotY + rotX - rx);
            double frontRightPower = (rotY + rotX + rx);
            double backRightPower = (rotY - rotX + rx);
            frontleft.setPower(frontLeftPower);
            backleft.setPower(-backLeftPower);
            frontright.setPower(-frontRightPower);
            backright.setPower(-backRightPower);

            telemetry.addData("mouse data", Mouse.getTheta());
            telemetry.update();


        }
    }


}