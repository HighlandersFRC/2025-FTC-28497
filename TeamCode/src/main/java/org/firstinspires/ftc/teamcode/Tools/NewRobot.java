package org.firstinspires.ftc.teamcode.Tools;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.NewArmSubsystem;
public class NewRobot {

    public Drive drive;
    public NewArmSubsystem armStates;
    public NewRobot(HardwareMap hardwareMap) {
        this.armStates = new NewArmSubsystem("shooter");
        this.drive = new Drive("drive", hardwareMap);
    }

}