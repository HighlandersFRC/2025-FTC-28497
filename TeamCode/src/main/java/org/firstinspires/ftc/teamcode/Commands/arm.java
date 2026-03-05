//package org.firstinspires.ftc.teamcode.Commands;
//
//import static org.firstinspires.ftc.teamcode.Subsystems.ARM.target;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.Subsystems.ARM;
//import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
//
//public class arm implements Command{
//    ARM arm;
//
//    @Override
//    public void start() {
//        arm.setTargetPosition(target);
//    }
//
//    @Override
//    public void execute() {
//        arm.getCurrentPosition();
//    }
//
//    @Override
//    public void end() {
//        arm.setPower(0);
//    }
//
//    @Override
//    public boolean isFinished() {
//        return Math.abs(arm.getCurrentPosition()) >= Math.abs(target);
//    }
//
//    @Override
//    public Subsystem getRequiredSubsystem() {
//        return null;
//    }
//}
