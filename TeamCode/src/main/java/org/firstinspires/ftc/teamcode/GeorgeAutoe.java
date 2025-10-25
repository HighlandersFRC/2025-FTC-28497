package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="GeorgeAuto")
public class GeorgeAutoe extends LinearOpMode {
    PID drivePID = new PID(0.004,0,0);
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor left = hardwareMap.get(DcMotor.class, "left");
        DcMotor right = hardwareMap.get(DcMotor.class, "right");

        drivePID.setMinOutput(-0.5);
        drivePID.setMaxOutput(0.5);
        
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drivePID.setSetPoint(1000);
        waitForStart();
        while (opModeIsActive()){
            drivePID.updatePID(left.getCurrentPosition());
            double power = drivePID.getResult();

//        left.setTargetPosition(500);
//        right.setTargetPosition(500);
//
//        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            left.setPower(-power);
            right.setPower(-power);
        }

        left.setPower(0);
        right.setPower(0);
    }
}
