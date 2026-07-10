package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.intake;
@TeleOp
public class robot extends LinearOpMode {
    CommandScheduler scheduler = new CommandScheduler();
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontleft = hardwareMap.dcMotor.get("left_front");
        DcMotor frontright = hardwareMap.dcMotor.get("right_front");
        DcMotor backleft = hardwareMap.dcMotor.get("left_back");
        DcMotor backright = hardwareMap.dcMotor.get("right_back");
        DcMotor intake = hardwareMap.dcMotor.get("IntakeMotor");
        DcMotor indexer = hardwareMap.dcMotor.get("IndexerMotor");
        DcMotor queue = hardwareMap.dcMotor.get("QueueMotor");
        DcMotor shooter = hardwareMap.dcMotor.get("ShooterMotor");
        waitForStart();
        while (opModeIsActive()) {

//            scheduler.schedule(
//                    new intake(1,10)
//            );
//            scheduler.run();

            double y = gamepad1.left_stick_y;
            double rx = -gamepad1.left_stick_x * 1.1;
            double x = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            if (gamepad1.right_bumper) {
                intake.setPower(1);

            } else if (gamepad1.right_trigger > 0) {

                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.a) {
                indexer.setPower(1);
                queue.setPower(1);
            } else if (gamepad1.x) {
                indexer.setPower(-1);
                queue.setPower(-1);
            } else {
                indexer.setPower(0);
                queue.setPower(0);
            }
            if (gamepad1.left_bumper) {
                shooter.setPower(-0.7999);
            } else {
                shooter.setPower(0);
            }

            double flp = (-y + x - rx) / denominator;
            double blp = (-y + x + rx) / denominator;
            double frp = (y + x - rx) / denominator;
            double brp = (-y - x - rx) / denominator;

            frontleft.setPower(flp);
            frontright.setPower(frp);
            backleft.setPower(blp);
            backright.setPower(brp);
        }
    }
}