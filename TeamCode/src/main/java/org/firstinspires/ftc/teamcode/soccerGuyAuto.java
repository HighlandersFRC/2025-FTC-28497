package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.Tools.PID;

@Autonomous
public class soccerGuyAuto extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    public SparkFunOTOS Otos;
    public HardwareDevice SparkFunOTOS;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");

        PID drivePID = new PID(0.000272, 0, 0.0003);
        double target = 893;
        double tolerance = 10;
        drivePID.setSetPoint(target);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SparkFunOTOS = hardwareMap.get("Otos");
        Otos.initialize();
        double otos = Otos.getAngularScalar();


        waitForStart();

        while (opModeIsActive()) {

            double leftPosRaw = leftMotor.getCurrentPosition();
            double rightPosRaw = rightMotor.getCurrentPosition();

            double leftPos = -leftPosRaw;
            double rightPos = rightPosRaw;

            double avgPos = (leftPos + rightPos) / 2.0;
            double basePower = drivePID.updatePID(avgPos);

            double correctionGain = 0.0002;
            double correction = correctionGain * (leftPos - rightPos);

            double leftPower = basePower - correction;
            double rightPower = basePower + correction;

            if (Math.abs(target - avgPos) < tolerance) {
                leftMotor.setPower(0.5);
                rightMotor.setPower(0.5);
                telemetry.addData("Status", "Reached target, stopped.");
                telemetry.update();
                break;
            }

            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            telemetry.addData("Avg Pos", avgPos);
            telemetry.addData("Left Raw", leftPosRaw);
            telemetry.addData("Right Raw", rightPosRaw);
            telemetry.addData("Left Pos", leftPos);
            telemetry.addData("Right Pos", rightPos);
            telemetry.addData("Base Power", basePower);
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Otos", otos);
            telemetry.update();
        }
    }
}

