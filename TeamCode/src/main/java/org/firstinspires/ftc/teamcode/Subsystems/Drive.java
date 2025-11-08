package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Vector;

/**
 * Drive subsystem for full mecanum control, field-centric or robot-centric.
 * Includes teleop and autonomous drive functions.
 */
public class Drive extends Subsystem {

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;

    private final PID xPID = new PID(1, 0, 0);
    private final PID yPID = new PID(0.5, 0, 0);
    private final PID thetaPID = new PID(5, 0, 1);

    public Drive(String name, HardwareMap hardwareMap) {
        super(name);
        initialize(hardwareMap);
        Mouse.init(hardwareMap);
    }

    /** Initialize all motors and basic settings. */
    private void initialize(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "left_front");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "left_back");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "right_front");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "right_back");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor.ZeroPowerBehavior zpb = DcMotor.ZeroPowerBehavior.BRAKE;
        frontLeftMotor.setZeroPowerBehavior(zpb);
        frontRightMotor.setZeroPowerBehavior(zpb);
        backLeftMotor.setZeroPowerBehavior(zpb);
        backRightMotor.setZeroPowerBehavior(zpb);

        resetEncoders();
    }

    /** Drive robot with direct motor powers (used in PID and TeleOp). */
    public void drive(double lf, double rf, double lb, double rb) {
        frontLeftMotor.setPower(lf);
        frontRightMotor.setPower(rf);
        backLeftMotor.setPower(lb);
        backRightMotor.setPower(rb);
    }

    /** Stops all motors completely. */
    public void stop() {
        drive(0, 0, 0, 0);
    }

    /** Resets encoders and ensures run mode is without encoders. */
    public void resetEncoders() {
        DcMotorEx[] motors = {frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};
        for (DcMotorEx m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /** Standard robot-centric drive using left stick for motion, right stick for turn. */
    public void teleopDrive(Gamepad gamepad) {
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x * 1.5;
        double rx = gamepad.right_stick_x;

        double frontLeftPower = y + x + rx;
        double backLeftPower = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower = y + x - rx;

        drive(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    /** Field-centric drive using IMU heading and odometry (Mouse). */
    public void fieldCentricDrive(Gamepad gamepad) {
        double x = -gamepad.left_stick_x * 2;
        double y = -gamepad.left_stick_y;
        double rx = -gamepad.right_stick_x;

        double botHeading = Math.toRadians(Mouse.getTheta());
        Mouse.update();

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double frontLeftPower = (-rotY + rotX + rx);
        double backLeftPower = (rotY - rotX + rx);
        double frontRightPower = (-rotY - rotX - rx);
        double backRightPower = (rotY + rotX - rx);

        drive(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
}
