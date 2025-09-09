package org.firstinspires.ftc.robotcontroller;

import com.acmerobotics.dashboard.FtcDashboard;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class hardwear extends LinearOpMode {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "left_front");
        frontRightMotor = hardwareMap.get(DcMotor.class, "right_front");
        backLeftMotor = hardwareMap.get(DcMotor.class, "left_back");
        backRightMotor = hardwareMap.get(DcMotor.class, "right_back");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        int target_encoder_counts = 1658;
        frontLeftMotor.setTargetPosition(target_encoder_counts);
        frontRightMotor.setTargetPosition(target_encoder_counts);
        backLeftMotor.setTargetPosition(target_encoder_counts);
        backRightMotor.setTargetPosition(target_encoder_counts);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);
        backRightMotor.setPower(0.5);

        while (opModeIsActive() && (
                frontLeftMotor.isBusy() ||
                        frontRightMotor.isBusy() ||
                        backLeftMotor.isBusy() ||
                        backRightMotor.isBusy())) {

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("FL Encoder", frontLeftMotor.getCurrentPosition());
            packet.put("FR Encoder", frontRightMotor.getCurrentPosition());
            packet.put("BL Encoder", backLeftMotor.getCurrentPosition());
            packet.put("BR Encoder", backRightMotor.getCurrentPosition());

            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("FL Encoder", frontLeftMotor.getCurrentPosition());
            telemetry.addData("FR Encoder", frontRightMotor.getCurrentPosition());
            telemetry.addData("BL Encoder", backLeftMotor.getCurrentPosition());
            telemetry.addData("BR Encoder", backRightMotor.getCurrentPosition());
            telemetry.update();
        }
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }


}
