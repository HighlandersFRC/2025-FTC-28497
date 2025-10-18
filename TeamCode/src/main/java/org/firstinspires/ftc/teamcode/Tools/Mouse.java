package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mouse {

    private static double fieldX;
    private static double fieldY;
    private static double theta;
    private static double accelX;
    private static double accelY;
    private static double accelTheta;
    private static SparkFunOTOS mouse;
    private static SparkFunOTOS.Pose2D field;
    private static SparkFunOTOS.Pose2D accel;

    private static long lastUpdateTime = 0;
    private static final long UPDATE_INTERVAL_MS = 20; // 50Hz max

    public static void init(HardwareMap hardwareMap) {
        try {
            mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");
        } catch (Exception e) {
            System.out.println("Mouse init failed: " + e.getMessage());
            mouse = null;
        }
    }

    public static void configureOtos() {
        if (mouse == null) return;

        try {
            mouse.setLinearUnit(SparkFunOTOS.LinearUnit.METERS);
            mouse.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES);
            SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-0.15875, 0, 90);//0
            mouse.setOffset(offset);
            mouse.setLinearScalar(1.005809562240364);
            mouse.setAngularScalar(0.989932511851);
            mouse.calibrateImu();
            mouse.resetTracking();

            System.out.println("OTOS configured");
        } catch (Exception e) {
            System.out.println("Failed to configure OTOS: " + e.getMessage());
        }
    }

    public static void setPosition(double x, double y, double theta) {
        if (mouse == null) return;

        try {
            SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(x, y, theta);
            mouse.setPosition(currentPosition);
        } catch (Exception e) {
            System.out.println("Failed to set position: " + e.getMessage());
        }
    }

    // Non-blocking update
    public static void update() {
        if (mouse == null) return;

        long now = System.currentTimeMillis();
        if (now - lastUpdateTime < UPDATE_INTERVAL_MS) return; // skip to limit read rate

        try {
            field = mouse.getPosition();
            accel = mouse.getAcceleration();

            accelX = accel.x;
            accelY = accel.y;
            accelTheta = accel.h;

            fieldX = field.x;
            fieldY = field.y;
            theta = field.h;

        } catch (Exception e) {
            // log error but don’t crash
            System.out.println("Mouse update failed: " + e.getMessage());
        }

        lastUpdateTime = now;
    }

    public static double getX() {
        return -fieldX;
    }

    public static double getY() {
        return fieldY;
    }

    public static double getTheta() {
        return theta;
    }

    public static double getAccelX(){
        return  accelX;
    }
    public static double getAccelY(){
        return  accelY;
    }

    public static double getAccelTheta(){
        return accelTheta;
    }
}
