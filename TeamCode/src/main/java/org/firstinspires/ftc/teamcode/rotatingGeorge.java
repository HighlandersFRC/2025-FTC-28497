package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.rotate;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
@TeleOp
public class rotatingGeorge extends LinearOpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    public IMU imu;
    double heading;
    public SparkFunOTOS mouse;
    CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        //imu = hardwareMap.get(IMU.class, "imu");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        PID drivePID = new PID(0.02, 0.0001, 0.002);

        double target = 91;
        drivePID.setSetPoint(target);
        mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        waitForStart();
        mouse.resetTracking();
        while (opModeIsActive()) {
            heading = mouse.getPosition().h;
           // double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = target - heading;
            double result = drivePID.updatePID(heading);

            if (gamepad1.a) {


                leftFront.setPower(result);
                rightFront.setPower(-result);
                leftBack.setPower(-result);
                rightBack.setPower(-result);

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);

                telemetry.addData("yaw", heading);
                telemetry.addData("result", result);
                telemetry.addData("error", error);
                telemetry.update();
            }
            if (gamepad1.right_bumper) {
                mouse.resetTracking();
            }
            if (gamepad1.b) {
                target = -90;
                drivePID.setSetPoint(target);
                error = target - heading;
                result = drivePID.updatePID(heading);

                leftFront.setPower(result);
                rightFront.setPower(-result);
                leftBack.setPower(-result);
                rightBack.setPower(-result);

                telemetry.update();
                telemetry.addData("yaw", heading);
                telemetry.addData("result", result);
                telemetry.addData("error", error);
                telemetry.update();

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
            }
            if (gamepad1.x) {
                scheduler.schedule(new rotate(90));

                mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");
                leftFront = hardwareMap.get(DcMotor.class, "left_front");
                rightFront = hardwareMap.get(DcMotor.class, "right_front");
                leftBack = hardwareMap.get(DcMotor.class, "left_back");
                rightBack = hardwareMap.get(DcMotor.class, "right_back");

                scheduler.run();
            }
        }
    }
}