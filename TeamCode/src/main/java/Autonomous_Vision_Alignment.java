import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import subsystems.IntakeLimelightSubsys;
import subsystems.Intake_DiffyClaw;
import subsystems.SubsysCore;
import subsystems.UnifiedTelemetry;
import utils.MedianSmoother;

@Config
@Autonomous(name = "Autonomous Vision Alignment")
public class Autonomous_Vision_Alignment extends OpMode {
    private Follower follower;
    private IntakeLimelightSubsys ll;
    private UnifiedTelemetry tel;
    private Intake_DiffyClaw diffyClawIntake;
    private MedianSmoother medianSmoother;
    private Timer pathTimer;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    @Override
    public void init(){
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        tel = new UnifiedTelemetry();
        tel.init(this.telemetry);
        SubsysCore.setGlobalParameters(hardwareMap, this);

        pathTimer = new Timer();

        medianSmoother = new MedianSmoother(300);
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
        pathTimer.resetTimer();
    }

    double x, y, angle;
    boolean resultFound = false;

    enum VisionStates {
        DETECTING,
        DETECTED,
        AFTER_MOVING,
        WAITBEFOREDETECTAGAIN,
    }

    VisionStates visionState = VisionStates.DETECTING;

    public void setVisionState(VisionStates visionState) {
        this.visionState = visionState;
        pathTimer.resetTimer();
    }

    public static double inchesToTicks = 20, vertOffset = -1;
    public static double horizScale = 0.7;

    @Override
    public void loop(){
        switch(visionState){
            case DETECTING:
                if(ll.isResultValid()) {
                    medianSmoother.update(ll.getHoriz(), ll.getVert(), ll.getAngle());
                }
                if(pathTimer.getElapsedTimeSeconds() > 5){
                    if(medianSmoother.getSize() > 0){
                        follower.followPath(
                                follower.pathBuilder()
                                        .addPath(new BezierLine(follower.getPose(), new Pose(follower.getPose().getX(), follower.getPose().getY()- medianSmoother.getSmoothedX()*horizScale)))
                                        .setZeroPowerAccelerationMultiplier(2)
                                        .setConstantHeadingInterpolation(Math.toRadians(0))
                                        .build(),
                                true
                        );
                        diffyClawIntake.ExtendTo((medianSmoother.getSmoothedY()-vertOffset)* inchesToTicks, Intake_DiffyClaw.ExtensionUnits.ticks);
                        diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                        Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = medianSmoother.getSmoothedAngle()*110/90;
                        diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                        ll.turnOff();
                        medianSmoother.clearVals();
                        setVisionState(VisionStates.DETECTED);
                    } else {
                        pathTimer.resetTimer();
                    }
                }
                break;
            case DETECTED:
                if(!follower.isBusy() && diffyClawIntake.extensionReachedTarget()){
                    setVisionState(VisionStates.AFTER_MOVING);
                }
                break;
            case AFTER_MOVING:
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
                    if(pathTimer.getElapsedTimeSeconds() > 0.5){
                        diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.CLOSED);
                        if(pathTimer.getElapsedTimeSeconds() > 0.7){
                            diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_RETRACT_HOLD);
                            diffyClawIntake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                            setVisionState(VisionStates.WAITBEFOREDETECTAGAIN);
                        }
                    }
                }
                break;
            case WAITBEFOREDETECTAGAIN:
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                    if(pathTimer.getElapsedTimeSeconds() > 3){
                        diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.VISION);
                        ll.turnOn();
                        setVisionState(VisionStates.DETECTING);
                    }
                }
                break;
        }


        ll.loop();
        diffyClawIntake.HoldExtension();
        diffyClawIntake.loop();
        tel.update();
        follower.update();
    }
}
