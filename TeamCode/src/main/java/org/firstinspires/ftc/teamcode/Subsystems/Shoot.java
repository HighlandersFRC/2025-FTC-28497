package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shoot {
    private DcMotorEx shooter;

    public Shoot(HardwareMap hardwareMap){
         shooter= hardwareMap.get(DcMotorEx.class, "shooter");
    }

    public void throwBall(double power){
        shooter.setPower(power);
    }
}