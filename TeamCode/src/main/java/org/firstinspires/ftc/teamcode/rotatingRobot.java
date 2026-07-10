package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Tools.PID;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
@Autonomous
public class rotatingRobot extends LinearOpMode {
    DcMotor leftFront = hardwareMap.get(DcMotor.class, "left_front");
    DcMotor rightFront = hardwareMap.get(DcMotor.class, "right_front");
    DcMotor leftBack = hardwareMap.get(DcMotor.class,"left_back");
    DcMotor rightBack = hardwareMap.get(DcMotor.class, "right_back");
    public IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();
        while (opModeIsActive()) {

        }
    }
}