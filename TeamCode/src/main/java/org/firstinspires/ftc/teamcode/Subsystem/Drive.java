package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.DefaultCommands.DriveDefault;
import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Vector;


public class Drive extends Subsystem {

    private Drive.Drive_State wantedSuperState= Drive_State.IDLE;
    private Drive.Drive_State currentSuperState= Drive_State.IDLE;

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;

    private final double TICKS_PER_REV = 2000;
    private final double WHEEL_DIAMETER = 0.048; // meters
    private final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    private double x = 0.0;
    private double y = 0.0;
    private double theta = 0.0;

    private double lastLeftPos = 0;
    private double lastRightPos = 0;
    private double lastCenterPos = 0;

    private final PID xPID = new PID(1, 0, 0);
    private final PID yPID = new PID(1, 0, 0.5);
    private final PID thetaPID = new PID(5, 0, 1);

    private long lastUpdateTime = 0;

    private double totalXTraveled = 0.0;
    private double totalYTraveled = 0.0;

    private final double L = 0.4064;
    private final double W = 0.4064;

    public Drive(String name, HardwareMap hardwareMap) {
        super(name);

        // Initialize motors using the HardwareMap
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "left_front");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "left_back");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "right_front");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "right_back");
        Mouse.init(hardwareMap);
// Set motor directions (if needed)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

// Set zero power behavior for all motors
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

// Reset encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

// Set to RUN_WITHOUT_ENCODER for odometry
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public enum Drive_State{
        DEFAULT,
        IDLE,
    }

    public void setWantedState(Drive_State driveState){
        wantedSuperState = driveState;
    }

    private void initialize(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "left_front");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "left_back");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "right_front");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "right_back");

        Mouse.init(hardwareMap);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.resetEncoder();
        lastUpdateTime = System.currentTimeMillis();
//        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void teleopDrive(Gamepad gamepad1) {
        double x = -gamepad1.left_stick_x*2;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double frontLeftPower = (-y + x + rx);
        double backLeftPower = (y + x - rx);
        double frontRightPower = (y + x + rx);
        double backRightPower = (y - x + rx);

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    public  void stop() {
        drive(0,0,0,0);

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void BRAKE() {

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void drive(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        frontLeftMotor.setPower(-leftFrontPower);
        frontRightMotor.setPower(-rightFrontPower);
        backLeftMotor.setPower(-leftBackPower);
        backRightMotor.setPower(-rightBackPower);
    }

    public void FeildCentric(Gamepad gamepad) {

        double x = -gamepad.left_stick_x*2;
        double y = gamepad.left_stick_y;
        double rx = -gamepad.right_stick_x;

        double botHeading = -Math.toRadians(Mouse.getTheta());
        Mouse.update();

        if (gamepad.options) {
            gamepad.setLedColor(255,0,0, 1000);
            Mouse.configureOtos();
        }

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        double frontLeftPower  = rotY + rotX + rx;
        double backLeftPower   = rotY - rotX + rx;
        double frontRightPower = rotY - rotX - rx;
        double backRightPower  = rotY + rotX - rx;

        frontLeftMotor.setPower(-frontLeftPower);
        backLeftMotor.setPower(-backLeftPower);
        frontRightMotor.setPower(-frontRightPower);
        backRightMotor.setPower(-backRightPower);

        System.out.println(botHeading);
    }

    public void resetEncoder() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        x = 0.0;
        y = 0.0;
        theta = 0.0;

        lastLeftPos = 0;
        lastRightPos = 0;
        lastCenterPos = 0;
    }

    public  double direction() {
        return direction();
    }

    public void setPosition(double fieldX, double fieldY, double fieldTheta) {
        x = fieldX;
        y = fieldY;
        theta = fieldTheta;

        lastLeftPos = getLeftEncoder();
        lastRightPos = getRightEncoder();
        lastCenterPos = getCenterEncoder();
    }

    public int getLeftEncoder() {
        return backRightMotor.getCurrentPosition();
    }
    public  int getRightEncoder() {
        return frontLeftMotor.getCurrentPosition();
    }
    public  int getBackEncoder(){
        return backLeftMotor.getCurrentPosition();
    }

    public  int getCenterEncoder() {
        return frontRightMotor.getCurrentPosition();
    }

    public void autoDrive(Vector vector, double angle) {

        double vx = vector.getX();
        double vy = vector.getY();

        double vtheta = Math.toRadians(angle);

        double rotX = vx * Math.cos(-vtheta) - vy * Math.sin(-vtheta);
        double rotY = vx * Math.sin(-vtheta) + vy * Math.cos(-vtheta);

        Vector robotVector = new Vector(rotX,rotY);

        robotVector = robotVector.normalize();

        double vrx = vtheta/(Math.PI/2);

        double frontLeftPower = (-rotY + rotX +vrx);
        double backLeftPower = (-rotY - rotX + vrx);
        double frontRightPower = (-rotY - rotX - vrx);
        double backRightPower = (rotY - rotX + vrx);

        frontLeftMotor.setPower(-frontLeftPower);
        backLeftMotor.setPower(-backLeftPower);
        frontRightMotor.setPower(-frontRightPower);
        backRightMotor.setPower(-backRightPower);

    }

    protected Drive_State handleStateTransitions(){
        switch (wantedSuperState) {
            case DEFAULT:
                currentSuperState = Drive_State.DEFAULT;
                break;
            case IDLE:
                currentSuperState = Drive_State.IDLE;
                break;
        }
        return currentSuperState;
    }

    private void handleDefaultState(){
        BRAKE();
    }
    private void handleIdleState() {
        BRAKE();
    }

    @Override
    public void periodic() {
        handleStateTransitions();
        switch (currentSuperState) {
            case DEFAULT:
                handleDefaultState();
                break;
            case IDLE:
                handleIdleState();
                break;
        }
    }

    @Override
    public void setDefaultCommand(Command command) {
        super.setDefaultCommand(command);
    }

    @Override
    public Command getDefaultCommand() {
        return new DriveDefault(this);
    }
}