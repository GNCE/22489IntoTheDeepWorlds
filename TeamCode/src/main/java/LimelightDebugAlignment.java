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
import subsystems.LimelightDebug;
import subsystems.LynxModules;
import subsystems.SubsysCore;
import subsystems.UnifiedTelemetry;
import utils.MedianSmoother;
import utils.Storage;

@Config
@Autonomous(name = "Limelight Debug Alignment")
public class LimelightDebugAlignment extends OpMode {
    private Follower follower;
    private LimelightDebug ll;
    private UnifiedTelemetry tel;
    private Intake_DiffyClaw diffyClawIntake;
    private MedianSmoother medianSmoother;
    private Timer pathTimer;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private LynxModules lynxModules;
    @Override
    public void init(){
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        tel = new UnifiedTelemetry();
        tel.init(this.telemetry);
        SubsysCore.setGlobalParameters(hardwareMap, this);

        pathTimer = new Timer();
        lynxModules = new LynxModules();
        lynxModules.init();

        medianSmoother = new MedianSmoother(300);
        diffyClawIntake = new Intake_DiffyClaw();
        diffyClawIntake.init();
        ll = new LimelightDebug();
        ll.init();
    }
    @Override
    public void start(){
        ll.turnOn();
        diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.VISION);
        diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
        ll.setPipelineNumber(7);
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
    public static int startingOffset = 0;
    public static double visionScanTime = 1;

    public static IntakeLimelightSubsys.Alliance alliance = IntakeLimelightSubsys.Alliance.RED;
    public static IntakeLimelightSubsys.SampleType sampleType = IntakeLimelightSubsys.SampleType.ALLIANCE;

    @Override
    public void loop(){
        Storage.isRed = alliance == IntakeLimelightSubsys.Alliance.RED;
        lynxModules.resetCache();
        switch(visionState){
            case DETECTING:
                if(ll.isResultValid()) {
                    medianSmoother.add(ll.getHoriz(), ll.getVert(), ll.getAngle());
                }
                if(pathTimer.getElapsedTimeSeconds() > visionScanTime){
                    MedianSmoother.Sample detectedSample = medianSmoother.getMedian();

                    if(medianSmoother.getSize() > 0){
                        follower.followPath(
                                follower.pathBuilder()
                                        .addPath(new BezierLine(follower.getPose(), new Pose(follower.getPose().getX(), follower.getPose().getY()- detectedSample.getX())))
                                        .setZeroPowerAccelerationMultiplier(2)
                                        .setConstantHeadingInterpolation(Math.toRadians(0))
                                        .build(),
                                true
                        );
                        diffyClawIntake.ExtendTo(diffyClawIntake.getCurrentPosition() + detectedSample.getY(), Intake_DiffyClaw.ExtensionUnits.ticks);
                        diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                        Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = detectedSample.getAngle()*110/90;
                        diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                        ll.turnOff();
                        medianSmoother.clear();
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
                            diffyClawIntake.setUseColorSensor(true);
                            if(pathTimer.getElapsedTimeSeconds() > 0.8){
                                if(diffyClawIntake.getCurrentSampleState(sampleType == IntakeLimelightSubsys.SampleType.ALLIANCE) == Intake_DiffyClaw.SENSOR_READING.CORRECT){
                                    diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_RETRACT_HOLD);
                                } else {
                                    diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.VISION);
                                    diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                                }
                                diffyClawIntake.ExtendTo(startingOffset, Intake_DiffyClaw.ExtensionUnits.ticks);
                                setVisionState(VisionStates.WAITBEFOREDETECTAGAIN);
                            }
                        }
                    }
                }
                break;
            case WAITBEFOREDETECTAGAIN:
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                    if(pathTimer.getElapsedTimeSeconds() > 0.5){
                        diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.VISION);
                        if(pathTimer.getElapsedTimeSeconds() > 2){
                            ll.turnOn();
                            setVisionState(VisionStates.DETECTING);
                        }
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
