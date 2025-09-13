package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp
public class AprilTagFollowing extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontright");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backright");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.setMsTransmissionInterval(50);
        telemetry.addData("", "Robot Ready. Press Play.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        boolean autoMode = false;

        while (opModeIsActive()) {

            if (gamepad1.x) {
                autoMode = !autoMode;
            }

            LLResult result = limelight.getLatestResult();
            boolean hasValidPose = result.isValid() && result.getBotpose() != null;


            if (autoMode && hasValidPose) {
                Pose3D botpose = result.getBotpose();

                double targetYaw = botpose.getOrientation().getYaw();

                // Use yaw only for rotation correction
                double rotate = -targetYaw * 0.01;

                // Set motor powers for pure rotation (spin in place)
                frontLeftMotor.setPower(rotate);
                backLeftMotor.setPower(rotate);
                frontRightMotor.setPower(-rotate);
                backRightMotor.setPower(-rotate);

                telemetry.addData("Mode", "Auto Follow");

            } else if (!autoMode) {

                double y = gamepad1.left_stick_y;
                double x = -gamepad1.left_stick_x * 1.1;
                double rx = -gamepad1.right_stick_x;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double fl = (y + x + rx) / denominator;
                double bl = (y - x + rx) / denominator;
                double fr = (y - x - rx) / denominator;
                double br = (y + x - rx) / denominator;

                frontLeftMotor.setPower(fl);
                backLeftMotor.setPower(bl);
                frontRightMotor.setPower(fr);
                backRightMotor.setPower(br);

                telemetry.addData("Mode", "Manual");
            } else {

                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                telemetry.addData("Mode", "Auto Follow - No Target");
            }


            if (hasValidPose) {
                Pose3D botpose = result.getBotpose();
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                double z = botpose.getPosition().z;
                double roll = botpose.getOrientation().getRoll();
                double pitch = botpose.getOrientation().getPitch();
                double yaw = botpose.getOrientation().getYaw();

                telemetry.addData("BotPose",
                        "X: %.2f m, Y: %.2f m, Z: %.2f m",
                        x, y, z);
                telemetry.addData("Orientation",
                        "Roll: %.1f°, Pitch: %.1f°, Yaw: %.1f°",
                        Math.toDegrees(roll),
                        Math.toDegrees(pitch),
                        Math.toDegrees(yaw));

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducials) {
                    telemetry.addData("April Tag", "ID: %d, Family: %s",
                            fr.getFiducialId(),
                            fr.getFamily());
                }
            } else {
                telemetry.addData("Limelight", "No valid data");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
