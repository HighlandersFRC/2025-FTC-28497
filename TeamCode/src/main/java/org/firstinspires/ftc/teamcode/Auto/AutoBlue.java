package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutoBlue extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;

    private DcMotor armMotor;

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        armMotor = hardwareMap.get(DcMotor.class,"shooter");

        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        shoot();

        forward(0.5, 1000);
        turn(0.5, 500);
        forward(0.7, 700);




    }

    private void forward(double power, long ms) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        sleep(ms);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }


    private void turn(double power, long ms) {
        leftDrive.setPower(power);
        rightDrive.setPower(-power);
        sleep(ms);
        leftDrive.setPower(0);
        rightDrive.setPower(0);

    }


    private void shoot(){

        armMotor.setPower(1);
        sleep(2500);
        armMotor.setPower(-1);
        sleep(2500);
        armMotor.setPower(1);
        sleep(2500);
        armMotor.setPower(1);
        sleep(2500);
        armMotor.setPower(-1);
        sleep(2500);
        armMotor.setPower(1);
        sleep(2500);
        armMotor.setPower(-1);
        sleep(2500);
        armMotor.setPower(1);
        sleep(2500);
        armMotor.setPower(-1);
        sleep(2500);

        armMotor.setPower(0);






    }

}
