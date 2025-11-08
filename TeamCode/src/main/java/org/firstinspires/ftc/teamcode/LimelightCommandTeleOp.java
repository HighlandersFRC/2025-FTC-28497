package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;

@TeleOp
public class LimelightCommandTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fl = hardwareMap.dcMotor.get("left_front");
        DcMotor bl = hardwareMap.dcMotor.get("left_back");
        DcMotor fr = hardwareMap.dcMotor.get("right_front");
        DcMotor br = hardwareMap.dcMotor.get("right_back");

        LimelightSubsystem limelightSubsystem = new LimelightSubsystem(hardwareMap);
        FollowAprilTagCommand followCmd = new FollowAprilTagCommand(limelightSubsystem, fl, fr, bl, br);

        CommandScheduler scheduler = CommandScheduler.getInstance();

        telemetry.addLine("Press X to start AprilTag Follow");
        telemetry.update();
        waitForStart();

        boolean running = false;

        while (opModeIsActive()) {
            if (gamepad1.x && !running) {
                scheduler.schedule(followCmd);
                running = true;
            }

            scheduler.run();
            telemetry.addData("Command Running", running);
            telemetry.update();
        }

        scheduler.cancelAll();
    }
}
