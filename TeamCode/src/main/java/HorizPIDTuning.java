import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import subsystems.SubsysCore;
import subsystems.UnifiedTelemetry;


@Disabled
@TeleOp (name = "Horiz PID Tuning", group = "Tuning")
public class HorizPIDTuning extends OpMode {
    Intake_DiffyClaw intakeDiffyClaw;
    private UnifiedTelemetry tel;

    @Override
    public void init(){
        SubsysCore.setGlobalParameters(hardwareMap, this);
        tel = new UnifiedTelemetry();
        tel.init(this.telemetry);
        intakeDiffyClaw = new Intake_DiffyClaw();
        intakeDiffyClaw.init();
    }
    @Override
    public void loop(){
        intakeDiffyClaw.HoldExtension();
        intakeDiffyClaw.loop();
        intakeDiffyClaw.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.FULL_EXTENSION);
        tel.update();
    }
}
