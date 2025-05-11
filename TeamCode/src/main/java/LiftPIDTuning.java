import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import subsystems.OuttakeLiftSubsys;
import subsystems.SubsysCore;
import subsystems.UnifiedTelemetry;


@TeleOp (name = " PID Tuning", group = "Tuning")
public class LiftPIDTuning extends OpMode {
    OuttakeLiftSubsys outtakeLift;
    private UnifiedTelemetry tel;
    @Override
    public void init(){
        tel = new UnifiedTelemetry();
        tel.init(this.telemetry);
        SubsysCore.setGlobalParameters(hardwareMap, this);
        outtakeLift = new OuttakeLiftSubsys();
        outtakeLift.init();
    }
    @Override
    public void loop(){
        tel.update();
        outtakeLift.loop();
        outtakeLift.holdLift();
        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.BACK_SCORE);
    }
}
