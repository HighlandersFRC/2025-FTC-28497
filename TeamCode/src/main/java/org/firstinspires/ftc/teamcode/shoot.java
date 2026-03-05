package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.Tools.PID;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Autonomous
public class shoot extends LinearOpMode {
    DcMotor shooter;

    @Override
    public void runOpMode() throws InterruptedException {

        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        PID armPID = new PID(0.1,0.0001,0.001);

        armPID.setMaxOutput(1);
        armPID.setMinOutput(-1);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double target = 1440;

        armPID.setSetPoint(target);

        waitForStart();

        while (opModeIsActive()) {
            shooter.setPower(target);
            double currentPos = shooter.getCurrentPosition();
            double error = armPID.getError();
            double power = armPID.updatePID(currentPos);

            System.out.println("Current Pos" + shooter.getCurrentPosition());
            telemetry.addData("Current Postion", shooter.getCurrentPosition());
            telemetry.addData("Error", armPID.getError());
            telemetry.addData("Power", shooter.getPower());
            telemetry.update();
        }
    }
}