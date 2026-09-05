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
    //public SparkFunOTOS mouse;
    CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        imu = hardwareMap.get(IMU.class, "imu");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        PID drivePID = new PID(0.02, 0.0001, 0.002);

        double target = 92;
        drivePID.setSetPoint(target);
        //mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        waitForStart();
        imu.resetYaw();
        while (opModeIsActive()) {

            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = target - yaw;
            double result = drivePID.updatePID(yaw);

            if (gamepad1.a) {

                //double heading = mouse.getPosition().h;
                //mouse.resetTracking();

                leftFront.setPower(result);
                rightFront.setPower(-result);
                leftBack.setPower(-result);
                rightBack.setPower(-result);

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);

                telemetry.addData("Yaw", yaw);
                telemetry.addData("result", result);
                telemetry.addData("error", error);
                telemetry.update();
            }
            if (gamepad1.right_bumper) {
                imu.resetYaw();
            }
            if (gamepad1.b) {
                target = -90;
                drivePID.setSetPoint(target);
                error = target - yaw;
                result = drivePID.updatePID(yaw);

                leftFront.setPower(result);
                rightFront.setPower(-result);
                leftBack.setPower(-result);
                rightBack.setPower(-result);

                telemetry.update();
                telemetry.addData("Yaw", yaw);
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
                scheduler.run();
            }
        }
    }
}
//