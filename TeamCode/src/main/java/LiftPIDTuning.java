import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import subsystems.OuttakeLiftSubsys;
import subsystems.SubsysCore;


@TeleOp (name = " PID Tuning", group = "Tuning")
public class LiftPIDTuning extends OpMode {
    OuttakeLiftSubsys outtakeLift;
    @Override
    public void init(){
        SubsysCore.setGlobalParameters(hardwareMap, this);
        outtakeLift = new OuttakeLiftSubsys();
        outtakeLift.init();
    }
    @Override
    public void loop(){
        outtakeLift.holdLift();
        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.BACK_SCORE);
    }
}
