package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ArmCommandDown;
import org.firstinspires.ftc.teamcode.Commands.ArmCommandUp;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Tools.Drive;

@TeleOp
public class drivetain extends LinearOpMode {

    private Drive drive;
    private ArmSubsystem arm;
    private CommandScheduler scheduler;

    @Override
    public void runOpMode() {

        drive = new Drive("Drive", hardwareMap);

        arm = new ArmSubsystem();
        arm.initialize(hardwareMap);

        scheduler = CommandScheduler.getInstance();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                scheduler.schedule(new ArmCommandUp(arm));
            }

            if (gamepad1.b) {
                scheduler.schedule(new ArmCommandDown(arm));
            }

            scheduler.run();

            drive.teleopDrive(gamepad1);
        }
    }
}
