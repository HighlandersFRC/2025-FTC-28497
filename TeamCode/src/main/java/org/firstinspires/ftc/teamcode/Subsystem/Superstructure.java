package org.firstinspires.ftc.teamcode.Subsystem;


import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Tools.Constants.pivotPID;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Superstructure extends Subsystem {



    private Superstructure.SUPER_STATE wantedSuperState = Superstructure.SUPER_STATE.IDLE;
    private Superstructure.SUPER_STATE currentSuperState = Superstructure.SUPER_STATE.IDLE;
    public Superstructure(String name) {
        super(name);
    }

    public void setWantedState(Superstructure.SUPER_STATE superState){
        wantedSuperState = superState;
    }

    public void init(HardwareMap hardwareMap) {

    }


    public enum SUPER_STATE {
        DEFAULT,
        IDLE,
        ARM_UP,
        ARM_DOWN,
        ARM_FULLY_UP,
        ARM_FULLY_DOWN,
        ARM_SPECIMEN,
        ARM_HIGH_BUCKET,
        WRIST_UP,
        WRIST_DOWN,
        INTAKE,
        OUTTAKE,
        ELEVATOR_EXTEND,
        ELEVATOR_RETRACT
    }

    private SUPER_STATE handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT:
                currentSuperState = SUPER_STATE.DEFAULT;
                break;
            case IDLE:
                currentSuperState = SUPER_STATE.IDLE;
                break;
            case ARM_UP:
                currentSuperState = SUPER_STATE.ARM_UP;
                break;
            case ARM_DOWN:
                currentSuperState = SUPER_STATE.ARM_DOWN;
                break;
            case ARM_FULLY_UP:
                currentSuperState = SUPER_STATE.ARM_FULLY_UP;
                break;
            case ARM_FULLY_DOWN:
                currentSuperState = SUPER_STATE.ARM_FULLY_DOWN;
                break;
            case ARM_SPECIMEN:
                currentSuperState = SUPER_STATE.ARM_SPECIMEN;
                break;
            case ARM_HIGH_BUCKET:
                currentSuperState = SUPER_STATE.ARM_HIGH_BUCKET;
                break;
            case WRIST_UP:
                currentSuperState = SUPER_STATE.WRIST_UP;
                break;
            case WRIST_DOWN:
                currentSuperState = SUPER_STATE.WRIST_DOWN;
                break;
            case INTAKE:
                currentSuperState = SUPER_STATE.INTAKE;
                break;
            case OUTTAKE:
                currentSuperState = SUPER_STATE.OUTTAKE;
                break;
            case ELEVATOR_EXTEND:
                currentSuperState = SUPER_STATE.ELEVATOR_EXTEND;
                break;
            case ELEVATOR_RETRACT:
                currentSuperState = SUPER_STATE.ELEVATOR_RETRACT;
                break;
        }
        return currentSuperState;
    }

    private void handleDefaultState() {
//        armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.DEFAULT);
//        elevatorSubsystem.setWantedState(NewElevatorSubsystem.ELEVATOR_STATE.DEFAULT);
    }
    private void handleIdleState() {

    }


//    private void handleArmFullyUpState() {
//        armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.ARM_FULLY_UP);
//    }
//
//    private void handleArmFullyDownState() {
//       armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.ARM_FULLY_DOWN);
//    }
//
//    private void handleSpecimenState() {
//        armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.SPECIMEN);
//    }
//
//    private void handleHighBucketState() {
//        armSubsystem.setWantedState(NewArmSubsystem.ARM_STATE.HIGH_BUCKET);
//    }
//
//    private void handleWristUpState() {
//        wristSubsystem.setWantedState(NewWristSubsystem.WRIST_STATE.WRIST_UP);
//    }
//
//    private void handleWristDownState() {
//        wristSubsystem.setWantedState(NewWristSubsystem.WRIST_STATE.WRIST_DOWN);
//    }


    }
