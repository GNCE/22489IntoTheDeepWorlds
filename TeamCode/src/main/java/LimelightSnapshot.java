import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import subsystems.IntakeLimelightSubsys;
import subsystems.Intake_DiffyClaw;
import subsystems.SubsysCore;
import subsystems.UnifiedTelemetry;


@TeleOp(name="sigma", group = "sigma")
public class LimelightSnapshot extends OpMode {
    private Intake_DiffyClaw intakeDiffyClaw;
    private UnifiedTelemetry tel;
    private IntakeLimelightSubsys ll;
    private ToggleButton snapshotButton;

    @Override
    public void init() {
        SubsysCore.setGlobalParameters(hardwareMap, this);
        tel.init(this.telemetry);

        intakeDiffyClaw = new Intake_DiffyClaw();
        ll = new IntakeLimelightSubsys();
        intakeDiffyClaw.init();
        ll.init();
        ll.turnOn();
        intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.VISION);
    }
    @Override
    public void loop(){
        if(snapshotButton.input(gamepad1.circle)) ll.captureSnapshot("");
        ll.loop();
        intakeDiffyClaw.loop();
        tel.update();
    }
}