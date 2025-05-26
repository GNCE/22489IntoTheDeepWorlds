import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import subsystems.IntakeLimelightSubsys;
import subsystems.Intake_DiffyClaw;
import subsystems.SubsysCore;
import subsystems.UnifiedTelemetry;

@Autonomous(name = "Autonomous Vision Alignment")
public class Autonomous_Vision_Alignment extends OpMode {
    private Follower follower;
    private IntakeLimelightSubsys ll;
    private UnifiedTelemetry tel;
    private Intake_DiffyClaw diffyClawIntake;
    public static double mx =  -0.008, my =  -0.021;
    public static double targetX = 16, targetY = 0;
    private double targetHeading = 180, headingError, headingCorrection;

    public static double hp = 0.4, hi = 0, hd = 0.00008;
    public static double angleThreshold = 0.05;
    PIDController headingPIDController = new PIDController(hp, hi, hd);
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    @Override
    public void init(){
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        tel = new UnifiedTelemetry();
        tel.init(this.telemetry);
        SubsysCore.setGlobalParameters(hardwareMap, this);

        diffyClawIntake = new Intake_DiffyClaw();
        diffyClawIntake.init();
        ll = new IntakeLimelightSubsys();
        ll.init();
    }
    @Override
    public void start(){
        ll.turnOn();
        diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.VISION);
        diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
        ll.setPipelineNumber(4);
    }
    boolean validResult = false;
    double tx, ty, angle;

    @Override
    public void loop(){
        if(!validResult && ll.isResultValid()) {
            validResult = true;
            tx = ll.getTx();
            ty = ll.getTy();
            ll.turnOff();
        }
        if(validResult){
            diffyClawIntake.ExtendTo((ll.getTx() + 23) * 4, Intake_DiffyClaw.ExtensionUnits.ticks);
        }


        ll.loop();
        diffyClawIntake.HoldExtension();
        diffyClawIntake.loop();
        tel.update();
        follower.update();
    }
}
