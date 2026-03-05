package org.firstinspires.ftc.teamcode.Subsystems;
import org.firstinspires.ftc.teamcode.Tools.PID;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Tools.Mouse;

public class ARM extends Subsystem{
    public DcMotor arm;
    public double target = 0;

    public void initialize(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotor.class, "shooter");
        Mouse.init(hardwareMap);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public enum STATES {

        ARM_UP,
        ARM_DOWN,
        IDLE,
        
    }

    private void handleArmUp() {
        

    }

    private void handleArmDown() {


    }

    private void handleIdle() {


    }
}
