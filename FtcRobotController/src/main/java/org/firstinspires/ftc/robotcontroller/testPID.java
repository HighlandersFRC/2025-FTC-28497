package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class testPID extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor motor = hardwareMap.dcMotor.get("pivotMotor");

        PID testTwo = new PID(0.5, 0, 0.6);

        testTwo.setMaxOutput(1);

        testTwo.setMinOutput(-1);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double target = -1440 * 5;

        testTwo.setSetPoint(target);

        waitForStart();

        while (opModeIsActive()) {

            double currentPos = motor.getCurrentPosition();
            double error = testTwo.getError();
            double power = testTwo.updatePID(currentPos);

            motor.setPower(power);


            System.out.println("Current Pos" + motor.getCurrentPosition());
            telemetry.addData("Current Postion", motor.getCurrentPosition());
            telemetry.addData("Error", testTwo.getError());
            telemetry.addData("Power", motor.getPower());
            telemetry.update();

        }
    }
}
