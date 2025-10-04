package org.firstinspires.ftc.teamcode.Subsystem;


import org.firstinspires.ftc.teamcode.Subsystem.Drive;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Superstructure extends Subsystem {

Drive drive;

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
    }

    private SUPER_STATE handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT:
                currentSuperState = SUPER_STATE.DEFAULT;
                break;
            case IDLE:
                currentSuperState = SUPER_STATE.IDLE;
                break;

        }
        return currentSuperState;
    }

    private void handleDefaultState() {
        drive.setWantedState(Drive.Drive_State.DEFAULT);

    }
    private void handleIdleState() {
        drive.setWantedState(Drive.Drive_State.IDLE);
    }





}
