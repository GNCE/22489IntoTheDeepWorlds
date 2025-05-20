import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import config.subsystems.Lift;
import config.core.utils.SubsystemCore;
import config.subsystems.UnifiedTelemetry;


@TeleOp (name = " PID Tuning", group = "Tuning")
public class LiftPIDTuning extends OpMode {
    Lift outtakeLift;
    private UnifiedTelemetry tel;
    @Override
    public void init(){
        tel = new UnifiedTelemetry();
        tel.init(this.telemetry);
        SubsystemCore.setGlobalParameters(hardwareMap, this);
        outtakeLift = new Lift();
        outtakeLift.init();
    }
    @Override
    public void loop(){
        tel.update();
        outtakeLift.loop();
        outtakeLift.holdLift();
        outtakeLift.LiftTo(Lift.OuttakeLiftPositions.BACK_SCORE);
    }
}
