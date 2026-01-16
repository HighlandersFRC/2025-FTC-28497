package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Tools.Drive;
import org.firstinspires.ftc.teamcode.Tools.DriveYawAngle;

@Autonomous
public class RotateTo170 extends LinearOpMode {

    private Drive drive;
    private DriveYawAngle yawDrive;

    @Override
    public void runOpMode() {

        drive = new Drive("Drive", hardwareMap);


        yawDrive = new DriveYawAngle(hardwareMap, drive, 0);

        waitForStart();

        yawDrive.setTargetYaw(170);
        while (opModeIsActive() && !yawDrive.isAtTarget(1.0)) {
            yawDrive.updateDrive(0, 0);
            telemetry.addData("Yaw", yawDrive.getCurrentYaw());
            telemetry.update();
        }

        yawDrive.stop();
    }
}
