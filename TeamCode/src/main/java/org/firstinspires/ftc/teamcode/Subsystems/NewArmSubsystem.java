package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.ArmCommandDown;
import org.firstinspires.ftc.teamcode.Commands.ArmCommandUp;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;

public class NewArmSubsystem extends Subsystem {

    private ARM_STATE wantedSuperState = ARM_STATE.IDLE;
    private ARM_STATE currentSuperState = ARM_STATE.IDLE;
    private DcMotor ArmMotor;
    private ArmSubsystem arm;
    private CommandScheduler scheduler;

    public NewArmSubsystem(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {
        ArmMotor = hardwareMap.dcMotor.get("shooter");
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setWantedState(ARM_STATE armState){
        wantedSuperState = armState;
    }

    public enum ARM_STATE {
        DEFAULT,
        IDLE,
        ARM_UP,
        Arm_Down
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
            case Arm_Down:
                currentSuperState = ARM_STATE.Arm_Down;
                break;
        }
        return currentSuperState;
    }

    private void handleDefaultState() {
        ArmMotor.setPower(0);
    }

    private void handleIdleState() {

        ArmMotor.setPower(0.1);

    }

    private void handleArmUpState() {
        scheduler.schedule(new ArmCommandUp(arm));
    }

    private void handleArmDownState() {
        scheduler.schedule(new ArmCommandDown(arm));
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
            case Arm_Down:
                handleArmDownState();
                break;
        }
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {

    }

    public NewArmSubsystem(ArmSubsystem arm, CommandScheduler scheduler) {
        super("NewArmSubsystem");
        this.arm = arm;
        this.scheduler = scheduler;
    }



}