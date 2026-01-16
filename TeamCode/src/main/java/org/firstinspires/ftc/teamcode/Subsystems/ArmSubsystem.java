package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class ArmSubsystem extends Subsystem {

    private static final String name = "name";
    public DcMotor armMotor;
    private PID armPID;
    private double currentTarget = 0;

    public ArmSubsystem() {
        super(name);
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotor.class, "shooter");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armPID = new PID(0.095, 0.35, 0.07);
        armPID.setMinOutput(-1.0);
        armPID.setMaxOutput(1.0);
    }

    public void setTargetPosition(double targetpos) {
        armPID.setSetPoint(targetpos);
        this.currentTarget = targetpos;
    }

    public void resetEncoders(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getCurrentPos() {
        return armMotor.getCurrentPosition();

    }
    public double getTargetPos() {
        return this.currentTarget;
    }

    public void runArmPID() {
        double power = armPID.updatePID(armMotor.getCurrentPosition());

        armMotor.setPower(-power);
    }

    public void stop() {
        armMotor.setPower(0);
    }
}