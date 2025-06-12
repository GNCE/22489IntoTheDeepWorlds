import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants_samples;
import pedroPathing.constants.LConstants;
import subsystems.IntakeLimelightSubsys;
import subsystems.Intake_DiffyClaw;
import subsystems.LynxModules;
import subsystems.Outtake;
import subsystems.OuttakeLiftSubsys;
import subsystems.SubsysCore;
import subsystems.UnifiedTelemetry;
import utils.MedianSmoother;
import utils.Storage;

@Autonomous (name = "0+4 Vision Spikes")
public class Auto_0_4 extends OpMode{
    private Follower follower;
    private Intake_DiffyClaw intake;
    private OuttakeLiftSubsys outtakeLift;
    private Outtake outtake;
    private IntakeLimelightSubsys ll;
    private Timer pathTimer, opmodeTimer;
    private UnifiedTelemetry tel;
    private ElapsedTime outtakeSequenceTime, intakeSequenceTime, resetEncoderDelay;
    /** Start Pose of our robot */
    private final Pose startPose = new Pose(7.35, 113.625, Math.toRadians(270));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(12, 128, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(24.5, 120, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(24.5, 129, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(26, 131, Math.toRadians(25));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(60, 100, Math.toRadians(270));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(60, 120, Math.toRadians(270));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
    private PathChain[] grabPickups, scorePickups;
    ElapsedTime elapsedTime;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());

        grabPickups = new PathChain[]{grabPickup1, grabPickup2, grabPickup3};
        scorePickups = new PathChain[]{scorePickup1, scorePickup2, scorePickup3};
    }
    public enum INTAKE_SEQUENCE{
        TRANSFER_WAIT, READY, GRAB;
        private static final INTAKE_SEQUENCE[] vals = values();
        public INTAKE_SEQUENCE next(){
            return vals[(this.ordinal() + 1) % vals.length];
        }
        public INTAKE_SEQUENCE prev(){
            return vals[(this.ordinal() - 1 + vals.length) % vals.length];
        }
    }

    public enum BUCKET_SEQUENCE{
        TRANSFER, GRAB_AND_LIFT, SCORE, RESET;
        private static final BUCKET_SEQUENCE[] vals = values();
        public BUCKET_SEQUENCE next(){
            return vals[(this.ordinal() + 1) % vals.length];
        }
        public BUCKET_SEQUENCE prev(){
            return vals[(this.ordinal() - 1 + vals.length) % vals.length];
        }
    }
    BUCKET_SEQUENCE bucketSequence = BUCKET_SEQUENCE.TRANSFER;
    INTAKE_SEQUENCE intakeSequence = INTAKE_SEQUENCE.TRANSFER_WAIT;
    enum AutoState {
        DRIVE_TO_PRELOAD_SCORE,
        SCORE_WAIT,
        SCORE_PRELOAD,
        INTAKE_WAIT,
        INTAKE_SAMPLE,
        VISION,
        VISION_DONE,
        PICKUP,
        TRANSFER_SAMPLE,
        OUTTAKE_GRAB_SAMPLE,
        TRANSFER_DONE,
        READY_TO_SCORE,
        SCORE,
        AFTER_SCORE,
        PARK
    }

    private MedianSmoother medianSmoother;
    private AutoState autoState = AutoState.DRIVE_TO_PRELOAD_SCORE;
    int sampleCounter = 0;
    public void autonomousPathUpdate() {
        switch (autoState) {
            case DRIVE_TO_PRELOAD_SCORE:
                follower.followPath(scorePreload, true);
                outtake.setClawState(Outtake.ClawStates.CLOSED);
                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.LIFT_HIGH_BASKET);
                outtake.setOuttakeState(Outtake.OuttakeState.SAMPLE_SCORE_WAIT);
                Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
                setPathState(AutoState.SCORE_WAIT);
                sampleCounter = 0;
                break;
            case SCORE_WAIT:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.3){
                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLESCORE);
                    setPathState(AutoState.SCORE_PRELOAD);
                }
                break;
            case SCORE_PRELOAD:
                if (pathTimer.getElapsedTimeSeconds() > 0.5){
                    outtake.setClawState(Outtake.ClawStates.OPEN);
                }
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(grabPickups[sampleCounter], true);
                    outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER_WAIT);
                }
                if (pathTimer.getElapsedTimeSeconds()> 1.6){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.TRANSFER);
                    setPathState(AutoState.INTAKE_WAIT);
                }
                break;
            case INTAKE_WAIT:
                if (!follower.isBusy()) {
                    medianSmoother.clear();
                    intake.ExtendTo(200, Intake_DiffyClaw.ExtensionUnits.ticks);
                    intake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                    intake.setIntakeState(Intake_DiffyClaw.IntakeState.VISION);
                    setPathState(AutoState.INTAKE_SAMPLE);
                    ll.turnOn();
                }
                break;
            case INTAKE_SAMPLE:
                if(intake.extensionReachedTarget()){
                    setPathState(AutoState.VISION);
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.1) {
                    intake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
                    setPathState(AutoState.PICKUP);
                }
                break;
            case VISION:
                if(ll.isResultValid()){
                    medianSmoother.add(ll.getHoriz(), ll.getVert(), ll.getAngle());
                }
                if(pathTimer.getElapsedTimeSeconds() > 1){
                    MedianSmoother.Sample detectedSample = medianSmoother.getMedian();
                    intake.ExtendTo(200+ll.getVert(), Intake_DiffyClaw.ExtensionUnits.ticks);
                    intake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                    follower.followPath(
                            follower.pathBuilder()
                                    .addPath(new BezierLine(follower.getPose(), new Pose(follower.getPose().getX(), follower.getPose().getY()-detectedSample.getX())))
                                    .setConstantHeadingInterpolation(follower.getPose().getHeading())
                                    .build(),
                            true
                    );
                    ll.turnOff();
                    setPathState(AutoState.VISION_DONE);
                }
                break;
            case VISION_DONE:
                if(!follower.isBusy() && intake.extensionReachedTarget()){
                    setPathState(AutoState.PICKUP);
                }
                break;
            case PICKUP:
                intake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    intake.setClawState(Intake_DiffyClaw.CLAW_STATE.CLOSED);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.4){
                    intake.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER_WAIT);
                    intake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                    follower.followPath(scorePickups[sampleCounter],true);
                    setPathState(AutoState.TRANSFER_SAMPLE);
                }
                break;
            case TRANSFER_SAMPLE:
                if (pathTimer.getElapsedTimeSeconds() > 0.8){
                    outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER_WAIT);
                    intake.setClawState(Intake_DiffyClaw.CLAW_STATE.LOOSE);
                }
                if(pathTimer.getElapsedTimeSeconds() > 1){
                    intake.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER);
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.1){
                    outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER);
                }
                if(pathTimer.getElapsedTimeSeconds() > 1.3){
                    outtake.setClawState(Outtake.ClawStates.CLOSED);
                }
                if(pathTimer.getElapsedTimeSeconds() > 1.5){
                    intake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                }
                if(pathTimer.getElapsedTimeSeconds() > 1.75){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.LIFT_HIGH_BASKET);
                }
                if(pathTimer.getElapsedTimeSeconds() > 1.85){
                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLE_SCORE_WAIT);
                }
                if (!follower.isBusy() && Math.abs(outtakeLift.getCurrentPosition() - OuttakeLiftSubsys.OuttakeLiftPositionsCONFIG.HIGH_BASKET_POS) <= 5){
                    setPathState(AutoState.READY_TO_SCORE);
                }
                break;
            case READY_TO_SCORE:
                outtake.setOuttakeState(Outtake.OuttakeState.SAMPLESCORE);
                if(pathTimer.getElapsedTimeSeconds() > 0.45){
                    outtake.setClawState(Outtake.ClawStates.OPEN);
                }
                if(pathTimer.getElapsedTimeSeconds() > 0.7){
                    setPathState(AutoState.SCORE);
                }
                break;
            case SCORE:
                sampleCounter++;
                if (sampleCounter < 3){
                    follower.followPath(grabPickups[sampleCounter], true);
                    outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER_WAIT);
                    setPathState(AutoState.AFTER_SCORE);
                } else {
                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLE_SCORE_WAIT);
                    follower.followPath(park);
                    setPathState(AutoState.PARK);
                }
                break;
            case AFTER_SCORE:
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.TRANSFER);
                    setPathState(AutoState.INTAKE_WAIT);
                }
                break;
            case PARK:
                Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
                if (pathTimer.getElapsedTimeSeconds() > 0.6){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.TRANSFER);
                }
                break;
            default:
                break;
        }
    }
    public void setPathState(AutoState newState) {
        autoState = newState;
        pathTimer.resetTimer();
    }
    private LynxModules lynxModules;
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = new Follower(hardwareMap, FConstants_samples.class, LConstants.class);
        follower.setStartingPose(startPose);

        SubsysCore.setGlobalParameters(hardwareMap, this);
        tel = new UnifiedTelemetry();
        tel.init(this.telemetry);

        lynxModules = new LynxModules();
        lynxModules.init();

        outtake = new Outtake();
        outtake.init();

        medianSmoother = new MedianSmoother(200);

        outtakeLift = new OuttakeLiftSubsys();
        outtakeLift.init();
        intake = new Intake_DiffyClaw();
        intake.init();
        ll = new IntakeLimelightSubsys();
        ll.init();
        elapsedTime = new ElapsedTime();
        intakeSequenceTime = new ElapsedTime();
        outtakeSequenceTime = new ElapsedTime();
        resetEncoderDelay = new ElapsedTime();
        buildPaths();
    }
    @Override
    public void start(){
        elapsedTime.startTime();
        intakeSequenceTime.startTime();
        outtakeSequenceTime.startTime();
        resetEncoderDelay.startTime();
    }

    private final ToggleButton teamColorButton = new ToggleButton(Storage.isRed);

    @Override
    public void init_loop(){
        teamColorButton.input(gamepad1.dpad_up);
        Storage.isRed = teamColorButton.getVal();

        ll.setAlliance(Storage.isRed ? IntakeLimelightSubsys.Alliance.RED : IntakeLimelightSubsys.Alliance.BLUE);
        ll.setSampleType(IntakeLimelightSubsys.SampleType.BOTH);
        outtake.setClawState(Outtake.ClawStates.CLOSED);
        outtake.setOuttakeState(Outtake.OuttakeState.Auto_Wait);
        outtake.loop();
        tel.addData("Team Color:", Storage.isRed ? "Red" : "Blue");
        tel.update();
    }

    @Override
    public void loop() {
        lynxModules.loop();

        intake.loop();
        intake.HoldExtension();
        ll.loop();
        outtakeLift.loop();
        outtakeLift.holdLift();
        outtake.loop();
        autonomousPathUpdate();
        follower.update();

        Storage.CurrentPose = follower.getPose();
        tel.addData("x", follower.getPose().getX());
        tel.addData("y", follower.getPose().getY());
        tel.addData("heading", follower.getPose().getHeading());
        tel.update();
    }
}