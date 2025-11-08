package org.firstinspires.ftc.teamcode.CompVIsion;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp
public class limelight extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(1);

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("", "Robot Ready. Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", status.getName());
            telemetry.addData("Pipeline", "Index: %d",
                    status.getPipelineIndex());

            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();


                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                double z = botpose.getPosition().z;


                double roll  = botpose.getOrientation().getRoll();
                double pitch = botpose.getOrientation().getPitch();
                double yaw   = botpose.getOrientation().getYaw();


                telemetry.addData("MegaTag1 Pose",
                        "X: %.2f m, Y: %.2f m, Z: %.2f m",
                        x, y, z);
                telemetry.addData("Orientation",
                        "Roll: %.1f°, Pitch: %.1f°, Yaw: %.1f°",
                        Math.toDegrees(roll),
                        Math.toDegrees(pitch),
                        Math.toDegrees(yaw));

                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("April Tag",
                            "ID: %d, Family: %s, ",
                            fr.getFiducialId(),
                            fr.getFamily());

                    telemetry.addData("Limelight int",status);
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
