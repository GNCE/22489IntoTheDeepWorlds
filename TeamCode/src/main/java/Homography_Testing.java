import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import config.subsystems.IntakeLimelightSubsys;
import config.core.utils.SubsystemCore;
import config.subsystems.UnifiedTelemetry;

@TeleOp(name = "Homography Vision Testing")
public class Homography_Testing extends OpMode {
    private IntakeLimelightSubsys ll;
    private UnifiedTelemetry tel;

    @Override
    public void init(){
        tel = new UnifiedTelemetry();
        tel.init(this.telemetry);
        SubsystemCore.setGlobalParameters(hardwareMap, this);

        ll = new IntakeLimelightSubsys();
        ll.init();
    }
    @Override
    public void start(){
        ll.turnOn();
    }
    @Override
    public void loop(){
        ll.loop();
    }
}