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
    private double lastTx = 0; // for smoothing

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("left_front");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("left_back");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("right_front");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("right_back");

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Robot Ready. Press PLAY.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        boolean autoMode = false;

        while (opModeIsActive()) {

            if (gamepad1.x) {
                autoMode = !autoMode;
                sleep(300);
            }

            LLResult result = limelight.getLatestResult();
            boolean hasValidPose = result.isValid() && result.getBotpose() != null;

            if (autoMode && hasValidPose) {
                Pose3D botpose = result.getBotpose();

                double x = botpose.getPosition().x;
                double z = botpose.getPosition().z;
                double tx = result.getTx();

                double smoothTx = 0.7 * lastTx + 0.3 * tx;
                lastTx = smoothTx;

                double desiredDistance = 0.2;


                double forwardK = -0.45;
                double strafeK  = -0.25;
                double turnK    = -0.02;

                double forwardPower = (z - desiredDistance) * forwardK;
                double strafePower  = x * strafeK;
                double turnPower    = smoothTx * turnK;

                if (Math.abs(smoothTx) < 1.5) turnPower = 0;
                if (z < 0.4) {
                    forwardPower *= 0.5;
                    turnPower *= 0.5;
                }


                forwardPower = Math.max(-0.5, Math.min(0.5, forwardPower));
                strafePower  = Math.max(-0.5, Math.min(0.5, strafePower));
                turnPower    = Math.max(-0.4, Math.min(0.4, turnPower));


                double denominator = Math.max(Math.abs(forwardPower) + Math.abs(strafePower) + Math.abs(turnPower), 1);
                double fl = (forwardPower + strafePower + turnPower) / denominator;
                double bl = (forwardPower - strafePower + turnPower) / denominator;
                double fr = (forwardPower - strafePower - turnPower) / denominator;
                double br = (forwardPower + strafePower - turnPower) / denominator;

                frontLeftMotor.setPower(-fl);
                backLeftMotor.setPower(-bl);
                frontRightMotor.setPower(-fr);
                backRightMotor.setPower(-br);

                telemetry.addData("Mode", "AUTO - Tag Detected");
                telemetry.addData("TX (deg)", "%.2f", tx);
                telemetry.addData("Forward", "%.2f", forwardPower);
                telemetry.addData("Strafe", "%.2f", strafePower);
                telemetry.addData("Turn", "%.2f", turnPower);

            } else if (!autoMode) {
                // Manual control
                double y = gamepad1.left_stick_y;
                double x = -gamepad1.left_stick_x * 1.1;
                double rx = -gamepad1.right_stick_x;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double fl = (y + x + rx) / denominator;
                double bl = (y - x + rx) / denominator;
                double fr = (y - x - rx) / denominator;
                double br = (y + x - rx) / denominator;

                frontLeftMotor.setPower(-fl);
                backLeftMotor.setPower(-bl);
                frontRightMotor.setPower(-fr);
                backRightMotor.setPower(-br);

                telemetry.addData("Mode", "MANUAL");

            } else {
                // Auto mode, but no tag detected
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                telemetry.addData("Mode", "AUTO - No Tag");
            }

            // --- Orientation + Tag info ---
            if (hasValidPose) {
                Pose3D botpose = result.getBotpose();

                double bx = botpose.getPosition().x;
                double by = botpose.getPosition().y;
                double bz = botpose.getPosition().z;

                double rawYaw = botpose.getOrientation().getYaw();
                double yawDeg = Math.toDegrees(rawYaw);
                yawDeg = ((yawDeg + 180) % 360 + 360) % 360 - 180;

                double roll = Math.toDegrees(botpose.getOrientation().getRoll());
                double pitch = Math.toDegrees(botpose.getOrientation().getPitch());

                telemetry.addData("BotPose", "X: %.2f, Y: %.2f, Z: %.2f", bx, by, bz);
                telemetry.addData("Orientation", "Roll: %.1f°, Pitch: %.1f°, Yaw: %.1f°", roll, pitch, yawDeg);

                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                for (LLResultTypes.FiducialResult tag : tags) {
                    telemetry.addData("Tag", "ID: %d, Family: %s", tag.getFiducialId(), tag.getFamily());
                }
            } else {
                telemetry.addData("Limelight", "No valid pose");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
