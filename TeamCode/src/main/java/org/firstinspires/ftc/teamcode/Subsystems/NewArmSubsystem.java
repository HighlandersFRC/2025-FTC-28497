package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Tools.PID;
public class NewArmSubsystem extends Subsystem {

    private ARM_STATE wantedSuperState = ARM_STATE.IDLE;
    private ARM_STATE currentSuperState = ARM_STATE.IDLE;
    private DcMotor ArmMotor;
    private PID armPID;
    private double targetPOS = 1820;

    public NewArmSubsystem(String name) {
        super(name);
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        ArmMotor = hardwareMap.dcMotor.get("shooter");
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armPID = new PID(0.5,0,0);
        armPID.setMaxOutput(1);
        armPID.setMinOutput(-1);
    }

    public void setWantedState(ARM_STATE armState){
        wantedSuperState = armState;
    }

    public enum ARM_STATE {
        DEFAULT,
        IDLE,
        ARM_UP,
        ARM_DOWN
    }

    private ARM_STATE handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT:
                currentSuperState = ARM_STATE.DEFAULT;
                break;
            case IDLE:
                currentSuperState = ARM_STATE.IDLE;
                break;
            case ARM_UP:
                currentSuperState = ARM_STATE.ARM_UP;
                break;
            case ARM_DOWN:
                currentSuperState = ARM_STATE.ARM_DOWN;
                break;
        }
        return currentSuperState;
    }

    private void handleDefaultState() {
        ArmMotor.setPower(0.1);
    }

    private void handleIdleState() {
        ArmMotor.setPower(0);
    }

    private void handleArmUpState() {
       targetPOS = 0;
       armPID.setSetPoint(targetPOS);
       armPID.updatePID(ArmMotor.getCurrentPosition());
       ArmMotor.setPower(armPID.getResult());
    }

    private void handleArmDownState() {
        targetPOS = 0;
        armPID.setSetPoint(targetPOS);
        armPID.updatePID(ArmMotor.getCurrentPosition());
        ArmMotor.setPower(armPID.getResult());
    }
public boolean isFinished() {

        return Math.abs(targetPOS) - Math.abs(ArmMotor.getCurrentPosition()) <= 25;
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
            case ARM_UP:
                handleArmUpState();
                break;
            case ARM_DOWN:
                handleArmDownState();
                break;
        }
    }
}