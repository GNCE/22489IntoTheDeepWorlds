import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import subsystems.OuttakeLiftSubsys;


@TeleOp (name = " PID Tuning", group = "Tuning")
public class LiftPIDTuning extends OpMode {
    OuttakeLiftSubsys outtakeLift;
    @Override
    public void init(){
        outtakeLift = new OuttakeLiftSubsys();
        outtakeLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void loop(){
        outtakeLift.HoldLift();
        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.BACK_SCORE);
    }
}
