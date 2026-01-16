package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Shoot;
import org.firstinspires.ftc.teamcode.Tools.Drive;
;

@TeleOp
public class backup extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private Shoot shooter;

    private Drive drive;




    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        drive = new Drive("Drive", hardwareMap);

        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter = new Shoot(hardwareMap);

        telemetry.addLine("Arcade Drive + Shooter Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {


            if (gamepad1.a) {

                shooter.throwBall(1.0);

            } else if (gamepad1.b) {

                shooter.throwBall(-1.0);

            } else {
                shooter.throwBall(0.0
                );
            }

            drive.teleopDrive(gamepad1);




        }
    }

}
