/*package org.firstinspires.ftc.teamcode.CompVIsion;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp
public class AprilTagFinal extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private Limelight3A limelight;

    private final PID forwardPID = new PID(5.0, 0.0, 0.095);
    private final PID turnPID = new PID(0.095, 0.0, 0.019);
    private double lastTx = 0;

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Limelight Tank Drive Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        boolean autoMode = false;
        double desiredDistance = 10.0;
        double desiredTx = 0.0;

        forwardPID.setSetPoint(desiredDistance);
        turnPID.setSetPoint(desiredTx);

        while (opModeIsActive()) {

            if (gamepad1.x) {
                autoMode = !autoMode;
                sleep(300);
            }

            LLResult result = limelight.getLatestResult();
            boolean hasValidPose = result.isValid() && result.getBotpose() != null;

            if (autoMode && hasValidPose) {
                Pose3D botpose = result.getBotpose();
                double z = botpose.getPosition().z;
                double tx = result.getTx();
                double smoothTx = 0.3 * lastTx + 0.7 * tx;
                lastTx = smoothTx;

                double forwardPower = forwardPID.updatePID(-z);
                double turnPower = turnPID.updatePID(smoothTx);

                if (z < 0.4) {
                    forwardPower *= 0.5;
                    turnPower *= 0.5;
                }


                turnPower = Math.max(-0.5, Math.min(0.5, turnPower));

                double leftPower = -forwardPower - turnPower;
                double rightPower = -forwardPower + turnPower;

                double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
                if (max > 1.0) {

                    leftPower /= max;

                    rightPower /= max;

                }

                leftDrive.setPower(leftPower);
                rightDrive.setPower(rightPower);

                telemetry.addData("Mode", "AUTO - PID Active");
                telemetry.addData("TX", "%.2f", tx);
                telemetry.addData("Forward", "%.2f", forwardPower);
                telemetry.addData("Turn", "%.2f", turnPower);
            }

            else if (!autoMode) {
                double drive = gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;

                double leftPower = drive + turn;
                double rightPower = drive - turn;

                double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
                if (max > 1.0) {
                    leftPower /= max;
                    rightPower /= max;
                }

                leftDrive.setPower(leftPower);
                rightDrive.setPower(rightPower);

                telemetry.addData("Mode", "MANUAL");
            }

            else {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                telemetry.addData("Mode", "AUTO - No Tag");
            }

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
                telemetry.addData("BotPose", "X: %.2f Y: %.2f Z: %.2f", bx, by, bz);
                telemetry.addData("Orientation", "Roll: %.1f Pitch: %.1f Yaw: %.1f", roll, pitch, yawDeg);
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                for (LLResultTypes.FiducialResult tag : tags) {
                    telemetry.addData("Tag", "ID: %d Family: %s", tag.getFiducialId(), tag.getFamily());
                }
            } else {
                telemetry.addData("Limelight", "No valid pose");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
*/