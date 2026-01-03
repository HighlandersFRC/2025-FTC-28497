package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.NewArmSubsystem;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;

@TeleOp
public class Robot extends LinearOpMode {
    NewArmSubsystem armStates = new NewArmSubsystem("armStates");
    CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException {

        armStates.init(hardwareMap);

        NewRobot robot = new NewRobot(hardwareMap);
        robot.armStates = armStates;
        scheduler.setNewRobot(robot);

        Drive drive = new Drive("drive", hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            armStates.periodic();

            scheduler.run();

            if (gamepad1.a) {
                armStates.setWantedState(NewArmSubsystem.ARM_STATE.ARM_UP);
            } else if (gamepad1.b) {
                armStates.setWantedState(NewArmSubsystem.ARM_STATE.Arm_Down);
            } else {
                armStates.setWantedState(NewArmSubsystem.ARM_STATE.DEFAULT);
            }

            drive.teleopDrive(gamepad1);


        }
    }
}