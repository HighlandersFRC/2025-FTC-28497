package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.NewArmCommandUp;
import org.firstinspires.ftc.teamcode.Tools.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.NewArmSubsystem;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;

@TeleOp
public class Robot extends LinearOpMode {
    NewArmSubsystem armStates = new NewArmSubsystem("armStates");

    private Drive drive;
    CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new Drive("Drive", hardwareMap);

        armStates.initialize(hardwareMap);

        NewRobot robot = new NewRobot(hardwareMap);
        robot.armStates = armStates;
        scheduler.setNewRobot(robot);

        waitForStart();

        while (opModeIsActive()) {

            armStates.periodic();

            if (gamepad1.a) {

                scheduler.schedule(new NewArmCommandUp(robot.armStates));

            }
            scheduler.run();

            drive.teleopDrive(gamepad1);





        }
    }
}