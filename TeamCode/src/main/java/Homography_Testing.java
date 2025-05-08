import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import subsystems.IntakeLimelightSubsys;
import subsystems.SubsysCore;
import subsystems.UnifiedTelemetry;

@TeleOp(name = "Homography Vision Testing")
public class Homography_Testing extends OpMode {
    private IntakeLimelightSubsys ll;
    private UnifiedTelemetry tel;

    @Override
    public void init(){
        tel = new UnifiedTelemetry();
        tel.init(this.telemetry);
        SubsysCore.setGlobalParameters(hardwareMap, this);

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