package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.DefaultCommands.DriveDefault;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Vector;

public class Drive extends Subsystem {

    private Drive.Drive_State wantedSuperState = Drive_State.IDLE;
    private Drive.Drive_State currentSuperState = Drive_State.IDLE;

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

    double targetRobotHeading = 0;
    double targetMovementDirection = 0;

    // Velocity tracking variables
    private static final double UPDATE_INTERVAL = 0.020; // 20ms (50Hz)
    private static final int MOVING_AVERAGE_SIZE = 5; // Number of samples to average
    private static final double DECAY_RATE = 0.95; // Decay when stationary
    private static final double VELOCITY_THRESHOLD = 0.5; // Noise filter threshold

    private double velX = 0;
    private double velY = 0;
    private double speed = 0;
    public double actualMovementDirection = 0; // Public for field-centric use

    private double[] velXHistory = new double[MOVING_AVERAGE_SIZE];
    private double[] velYHistory = new double[MOVING_AVERAGE_SIZE];
    private int historyIndex = 0;
    private int historySamples = 0; // Track how many samples we have

    private double lastPosX = 0;
    private double lastPosY = 0;
    private double accumulator = 0;

    private ElapsedTime velocityTimer = new ElapsedTime();
    private double lastVelocityUpdateTime = 0;

    public Drive(String name, HardwareMap hardwareMap) {
        super(name);

        // Initialize motors
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "left_front");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "left_back");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "right_front");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "right_back");
        Mouse.init(hardwareMap);

        // Set motor directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior
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

        // Initialize velocity timer
        velocityTimer.reset();
    }

    public enum Drive_State {
        DEFAULT,
        IDLE,
    }

    public void setWantedState(Drive_State driveState) {
        wantedSuperState = driveState;
    }

    public void teleopDrive(Gamepad gamepad1) {
        double x = -gamepad1.left_stick_x * 2;
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

    public void stop() {
        drive(0, 0, 0, 0);

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

    public void FeildCentric(Gamepad gamepad, Telemetry telemetry) {
        // Read joystick inputs
        double y = -gamepad.left_stick_y; // forward/back
        double x = gamepad.left_stick_x;  // strafe (removed the *2 multiplier)
        double rx = -gamepad.right_stick_x; // rotation

        // Update OTOS sensor first
        Mouse.update();

        // Update velocity and direction
        updateVelocity(telemetry);

        // Robot heading for field-centric rotation
        double botHeading = -Math.toRadians(Mouse.getTheta());

        // Handle options button
        if (gamepad.options) {
            gamepad.setLedColor(255, 0, 0, 1000);
            Mouse.configureOtos();
            targetRobotHeading = 0;
        } else if (Math.abs(rx) >= 0.05) {
            targetRobotHeading = botHeading;
        }

        // Target movement direction based on joystick
        if (Math.abs(x) < 0.001 && Math.abs(y) < 0.001) {
            targetMovementDirection = 0;
        } else {
            targetMovementDirection = Math.atan2(x, y); // 0° = forward (y+)
        }

        // --- FIELD-CENTRIC TRANSFORM ---
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // --- STRAFE BOOST (compensate for slower strafe speed) ---
        double magnitude = Math.hypot(rotX, rotY);
        if (magnitude > 0.05) { // avoid dividing by zero
            double kStrafe = 1.1; // adjust this value based on testing
            rotX *= kStrafe;
            rotY *= kStrafe;
        }

        // --- WHEEL MIXING ---
        double frontLeftPower = rotY + rotX + rx;
        double backLeftPower = rotY - rotX + rx;
        double frontRightPower = rotY - rotX - rx;
        double backRightPower = rotY + rotX - rx;

        // Normalize so no motor exceeds ±1
        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Set motor powers
        frontLeftMotor.setPower(-frontLeftPower);
        backLeftMotor.setPower(-backLeftPower);
        frontRightMotor.setPower(-frontRightPower);
        backRightMotor.setPower(-backRightPower);
    }
    /**
     * Updates velocity and actual movement direction using fixed timestep.
     * This tells you what direction the robot is ACTUALLY moving based on
     * optical mouse sensor readings.
     */
    public void updateVelocity(Telemetry telemetry) {
        double now = velocityTimer.seconds();

        // First call initialization
        if (lastVelocityUpdateTime == 0) {
            lastPosX = Mouse.getX();
            lastPosY = Mouse.getY();
            lastVelocityUpdateTime = now;
            return;
        }

        // Calculate frame time and add to accumulator
        double frameTime = now - lastVelocityUpdateTime;
        lastVelocityUpdateTime = now;
        accumulator += frameTime;

        // Process updates in fixed timesteps for stable calculations
        while (accumulator >= UPDATE_INTERVAL) {
            updateVelocityFixed(UPDATE_INTERVAL);
            accumulator -= UPDATE_INTERVAL;
        }

        // Print to console (single updating line)
        System.out.print("\r");
        System.out.printf(
                "VelX: %.2f | VelY: %.2f | Speed: %.2f | Dir: %.1f°    ",
                velX, velY, speed, Math.toDegrees(actualMovementDirection)
        );
        System.out.flush();

        // Also add to telemetry for Driver Station
        telemetry.addData("Movement Speed", "%.2f in/s", speed);
        telemetry.addData("Movement Direction", "%.1f°", Math.toDegrees(actualMovementDirection));
        telemetry.addData("Velocity X", "%.2f", velX);
        telemetry.addData("Velocity Y", "%.2f", velY);
    }

    /**
     * Internal fixed timestep velocity update.
     * Guarantees consistent calculations regardless of loop timing.
     */
    private void updateVelocityFixed(double dt) {
        // Read current mouse sensor positions
        double currentX = Mouse.getX();
        double currentY = Mouse.getY();

        // Calculate deltas (change in position)
        double dx = currentX - lastPosX;
        double dy = currentY - lastPosY;

        // Detect sensor overflow (prevents direction from jumping randomly)
        final double MAX_REASONABLE_DELTA = 100.0;
        if (Math.abs(dx) > MAX_REASONABLE_DELTA) dx = 0;
        if (Math.abs(dy) > MAX_REASONABLE_DELTA) dy = 0;

        // Update stored positions
        lastPosX = currentX;
        lastPosY = currentY;

        // Calculate raw velocity (distance / time)
        double rawVelX = dx / dt;
        double rawVelY = dy / dt;

        // Store in circular buffer for moving average
        velXHistory[historyIndex] = rawVelX;
        velYHistory[historyIndex] = rawVelY;
        historyIndex = (historyIndex + 1) % MOVING_AVERAGE_SIZE;
        if (historySamples < MOVING_AVERAGE_SIZE) historySamples++;

        // Calculate moving average
        double sumVelX = 0;
        double sumVelY = 0;
        for (int i = 0; i < historySamples; i++) {
            sumVelX += velXHistory[i];
            sumVelY += velYHistory[i];
        }
        velX = sumVelX / historySamples;
        velY = sumVelY / historySamples;

        // Apply decay when stationary to reduce sensor noise
        if (Math.abs(dx) < 0.1) velX *= DECAY_RATE;
        if (Math.abs(dy) < 0.1) velY *= DECAY_RATE;

        // Snap very small values to zero (noise threshold)
        if (Math.abs(velX) < VELOCITY_THRESHOLD) velX = 0;
        if (Math.abs(velY) < VELOCITY_THRESHOLD) velY = 0;

        // Calculate speed and direction
        speed = Math.hypot(velX, velY);

        // Only update direction if robot is actually moving
        if (velX != 0 || velY != 0) {
            actualMovementDirection = Math.atan2(velY, velX);
        }
        // If stopped, keep last known direction
    }

    /**
     * Get movement direction in degrees (0° = right, 90° = up, 180° = left, 270° = down)
     */
    public double getMovementDirectionDegrees() {
        return Math.toDegrees(actualMovementDirection);
    }

    /**
     * Get movement direction in radians
     */
    public double getMovementDirectionRadians() {
        return actualMovementDirection;
    }

    /**
     * Check if robot is moving
     */
    public boolean isMoving() {
        return speed > VELOCITY_THRESHOLD;
    }

    /**
     * Reset velocity tracking (call when re-enabling OpMode)
     */
    public void resetVelocityTracking() {
        lastPosX = Mouse.getX();
        lastPosY = Mouse.getY();
        lastVelocityUpdateTime = velocityTimer.seconds();
        accumulator = 0;
        velX = velY = 0;
        speed = 0;
        actualMovementDirection = 0;

        // Clear moving average history
        for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
            velXHistory[i] = 0;
            velYHistory[i] = 0;
        }
        historyIndex = 0;
        historySamples = 0;
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

        // Also reset velocity tracking
        resetVelocityTracking();
    }

    public double direction() {
        return actualMovementDirection;
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

    public int getRightEncoder() {
        return frontLeftMotor.getCurrentPosition();
    }

    public int getBackEncoder() {
        return backLeftMotor.getCurrentPosition();
    }

    public int getCenterEncoder() {
        return frontRightMotor.getCurrentPosition();
    }

    public void resetMouse(Gamepad gamepad){
        if (gamepad.options){
            Mouse.configureOtos();
        }
    }

    public void autoDrive(Telemetry telemetry,Vector vector, double angle) {
        double vx = vector.getX();
        double vy = vector.getY();

        double vtheta = Math.toRadians(angle);

        double rotX = vx * Math.cos(-vtheta) - vy * Math.sin(-vtheta);
        double rotY = vx * Math.sin(-vtheta) + vy * Math.cos(-vtheta);

        Vector robotVector = new Vector(rotX, rotY);

        robotVector = robotVector.normalize();


        double vrx = vtheta / (Math.PI / 2);



        double frontLeftPower = rotY + rotX + vrx;
        double backLeftPower = rotY - rotX + vrx;
        double frontRightPower = rotY - rotX - vrx;
        double backRightPower = rotY + rotX - vrx;

        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Set motor powers
        frontLeftMotor.setPower(-frontLeftPower);
        backLeftMotor.setPower(-backLeftPower);
        frontRightMotor.setPower(-frontRightPower);
        backRightMotor.setPower(backRightPower);
         telemetry.addData("vx",robotVector.getX());
         telemetry.addData("vy",robotVector.getY());
         telemetry.addData("vrx",vrx);
    }


    protected Drive_State handleStateTransitions() {
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

    private void handleDefaultState() {
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