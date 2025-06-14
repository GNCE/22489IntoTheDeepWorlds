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

@Autonomous (name = "0 + 5 NV")
public class Auto_0_5_NV extends OpMode{
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
    private final Pose scorePose = new Pose(17.5, 132, Math.toRadians(330));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(24.5, 119, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(25, 128.75, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(27, 130.5, Math.toRadians(25));
    private final Pose visionPose = new Pose(62, 94, Math.toRadians(270));
    private final Pose visionControlP = new Pose(52, 122, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(60, 87.5, Math.toRadians(90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(60, 120, Math.toRadians(270));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, driveToVision, driveFromVision, parkFromVision;
    private PathChain[] grabPickups, scorePickups;
    ElapsedTime elapsedTime;
    public static double visionWaitTime = 0.5;
    private final double ZPAM = 0.5;
    private final double LPETC = 150;
    private final double LPETVC = 0.99;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload.setZeroPowerAccelerationMultiplier(1.3);
        scorePreload.setPathEndTValueConstraint(0.8);
        scorePreload.setPathEndTimeoutConstraint(LPETC);

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .setPathEndTValueConstraint(LPETVC)
                .setPathEndTimeoutConstraint(LPETC)
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(ZPAM)
                .setPathEndTValueConstraint(LPETVC)
                .setPathEndTimeoutConstraint(LPETC)
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(ZPAM)
                .setPathEndTValueConstraint(LPETVC)
                .setPathEndTimeoutConstraint(LPETC)
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(ZPAM)
                .setPathEndTValueConstraint(LPETVC)
                .setPathEndTimeoutConstraint(LPETC)
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(ZPAM)
                .setPathEndTValueConstraint(LPETVC)
                .setPathEndTimeoutConstraint(LPETC)
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(ZPAM)
                .setPathEndTValueConstraint(LPETVC)
                .setPathEndTimeoutConstraint(LPETC)
                .build();

        driveToVision = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(visionControlP),new Point(visionPose) ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), visionPose.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .setPathEndTValueConstraint(LPETVC)
                .setPathEndTimeoutConstraint(LPETC)
                .build();

        driveFromVision = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(visionPose), new Point(visionControlP),new Point(scorePose) ))
                .setLinearHeadingInterpolation(visionPose.getHeading(), scorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(2.7)
                .setPathEndTValueConstraint(LPETVC)
                .setPathEndTimeoutConstraint(LPETC)
                .build();
        parkFromVision = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(visionPose), new Point(visionControlP),new Point(parkPose) ))
                .setLinearHeadingInterpolation(visionPose.getHeading(), parkPose.getHeading())
                .setZeroPowerAccelerationMultiplier(2.7)
                .setPathEndTValueConstraint(LPETVC)
                .setPathEndTimeoutConstraint(LPETC)
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
        park.setZeroPowerAccelerationMultiplier(3.5);
        park.setPathEndTValueConstraint(LPETVC);
        park.setPathEndTimeoutConstraint(LPETC);

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
        AFTER_PRELOAD_SCORE,
        INTAKE_WAIT,
        INTAKE_SAMPLE,
        PICKUP,
        TRANSFER_SAMPLE,
        GRAB,
        READY_TO_SCORE,
        SCORE,
        AFTER_SCORE,
        VISION,
        VISION_MOVE,
        VISION_DONE,
        VISION_NOT_DETECTED,
        VISION_SETUP_AGAIN,
        VISION_PICKUP,
        BEFORE_SUB_DRIVE,
        SUB_DRIVE,
        SUB_SCORE,
        SUB_SCORE_WAIT,
        SUB_SCORE_DONE,
        PARK,
        KILL
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

                intake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.AUTO_POS);
                intake.setClawState(Intake_DiffyClaw.CLAW_STATE.SPIKE);
                intake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);

                Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
                setPathState(AutoState.SCORE_WAIT);
                sampleCounter = 0;
                break;
            case SCORE_WAIT:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.15){
                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLESCORE);
                    setPathState(AutoState.SCORE_PRELOAD);
                }
                break;
            case SCORE_PRELOAD:
                if (pathTimer.getElapsedTimeSeconds() > 0.3){
                    outtake.setClawState(Outtake.ClawStates.OPEN);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.6) {
                    follower.followPath(grabPickups[sampleCounter], true);
                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLE_SCORE_WAIT);
                    setPathState(AutoState.AFTER_PRELOAD_SCORE);
                }
                break;
            case AFTER_PRELOAD_SCORE:
                if(pathTimer.getElapsedTimeSeconds() > 0.4){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.TRANSFER);
                    outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER_WAIT);
                    setPathState(AutoState.INTAKE_WAIT);
                }
                break;
            case INTAKE_WAIT:
                if (!follower.isBusy()) {
                    if(sampleCounter == 2){
                        Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 30;
                    }
                    setPathState(AutoState.INTAKE_SAMPLE);
                }
                break;
            case INTAKE_SAMPLE:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) { //0.7
                    setPathState(AutoState.PICKUP);
                }
                break;
            case PICKUP:
                intake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
                if (pathTimer.getElapsedTimeSeconds() > 0.15){
                    intake.setClawState(Intake_DiffyClaw.CLAW_STATE.CLOSED);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.35){
                    intake.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER_WAIT);
                    intake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                    follower.followPath(scorePickups[sampleCounter],true);
                    setPathState(AutoState.TRANSFER_SAMPLE);
                }
                break;
            case TRANSFER_SAMPLE:
                if(intake.getCurrentPosition() < 50){
                    intake.setClawState(Intake_DiffyClaw.CLAW_STATE.LOOSE);
                }
                if(intake.getCurrentPosition() < 15){
                    setPathState(AutoState.GRAB);
                }
                break;
            case GRAB:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    intake.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    outtake.setClawState(Outtake.ClawStates.CLOSED);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    intake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.65) {
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.LIFT_HIGH_BASKET);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLE_SCORE_WAIT);

                    intake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.AUTO_POS);
                    intake.setClawState(Intake_DiffyClaw.CLAW_STATE.SPIKE);
                    intake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                }
                if (!follower.isBusy() && outtakeLift.getCurrentPosition() > 200 && !outtakeLift.isBusy()) {
                    setPathState(AutoState.READY_TO_SCORE);
                }
                break;
            case READY_TO_SCORE:
                outtake.setOuttakeState(Outtake.OuttakeState.SAMPLESCORE);
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    outtake.setClawState(Outtake.ClawStates.OPEN);
                }
                if(pathTimer.getElapsedTimeSeconds() > 0.6){
                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLE_SCORE_WAIT);
                    setPathState(AutoState.SCORE);
                }
                break;
            case SCORE:
                sampleCounter++;
                if (sampleCounter < 3){
                    follower.followPath(grabPickups[sampleCounter], true);
                    setPathState(AutoState.AFTER_SCORE);
                } else {
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLE_SCORE_WAIT);
                    intake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                    intake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                    intake.setIntakeState(Intake_DiffyClaw.IntakeState.VISION);

                    follower.followPath(driveToVision);
                    ll.turnOn();
                    sampleCounter = 0;
                    setPathState(AutoState.VISION);
                }
                break;
            case AFTER_SCORE:
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.TRANSFER);
                    outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER_WAIT);
                    setPathState(AutoState.INTAKE_WAIT);
                }
                break;
            case VISION:
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.TRANSFER);
                    outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER_WAIT);
                    outtake.setClawState(Outtake.ClawStates.OPEN);
                    intake.setIntakeState(Intake_DiffyClaw.IntakeState.VISION);
                }
                if (!follower.isBusy()){
                    medianSmoother.clear();
                    intake.setIntakeState(Intake_DiffyClaw.IntakeState.VISION);
                    setPathState(AutoState.VISION_MOVE);
                }
                break;
            case VISION_MOVE:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    if(ll.isResultValid()) {
                        medianSmoother.add(ll.getVert(), ll.getHoriz(), ll.getAngle());
                    }
                    if (pathTimer.getElapsedTimeSeconds() > visionWaitTime) { // Time it takes for claw to open
                        MedianSmoother.Sample detectedSample = medianSmoother.getMedian();
                        if (medianSmoother.getSize() > 0) {
                            follower.followPath(
                                    follower.pathBuilder()
                                            .addPath(new BezierLine(follower.getPose(), new Pose(follower.getPose().getX()- detectedSample.getY(), follower.getPose().getY())))
                                            .setConstantHeadingInterpolation(follower.getPose().getHeading())
                                            .setZeroPowerAccelerationMultiplier(1.2)
                                            .setPathEndTValueConstraint(0.75)
                                            .setPathEndTimeoutConstraint(100)
                                            .build(),
                                    true
                            );
                            intake.ExtendTo(detectedSample.getX(), Intake_DiffyClaw.ExtensionUnits.ticks);
                            intake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                            Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = detectedSample.getAngle() * 110 / 90;
                            intake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                            setPathState(AutoState.VISION_DONE);
                        } else {

                            setPathState(AutoState.VISION_NOT_DETECTED);
                        }
                    }
                }
                break;
            case VISION_NOT_DETECTED:
                medianSmoother.clear();
                intake.setIntakeState(Intake_DiffyClaw.IntakeState.VISION);
                intake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(new Point(follower.getPose()), new Point(follower.getPose().getX() + 3, follower.getPose().getY())))
                                .setConstantHeadingInterpolation(follower.getPose().getHeading())
                                .setZeroPowerAccelerationMultiplier(1.2)
                                .setPathEndTValueConstraint(0.99)
                                .build(),
                        true
                );
                setPathState(AutoState.VISION_SETUP_AGAIN);
                break;
            case VISION_SETUP_AGAIN:
                if(!follower.isBusy() && intake.getCurrentPosition() < 15){
                    setPathState(AutoState.VISION_MOVE);
                }
                break;
            case VISION_DONE:
                if(!follower.isBusy() && intake.extensionReachedTarget()){
                    setPathState(AutoState.VISION_PICKUP);
                }
                break;
            case VISION_PICKUP:
                intake.setUseColorSensor(true);
                if(pathTimer.getElapsedTimeSeconds() > 0.1){
                    intake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
                    if(pathTimer.getElapsedTimeSeconds() > 0.3){
                        intake.setClawState(Intake_DiffyClaw.CLAW_STATE.CLOSED);
                        if(pathTimer.getElapsedTimeSeconds() > 0.5){
                            intake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_RETRACT_HOLD);
                            if(pathTimer.getElapsedTimeSeconds() > 0.8){
                                if(intake.getCurrentSampleState(false) != Intake_DiffyClaw.SENSOR_READING.CORRECT){
                                    setPathState(AutoState.VISION_NOT_DETECTED);
                                } else {
                                    intake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                                    intake.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER_WAIT);
                                    outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER_WAIT);
                                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.TRANSFER);
                                    if(intake.getCurrentPosition() < 100){
                                        follower.followPath(driveFromVision, true);
                                        setPathState(AutoState.BEFORE_SUB_DRIVE);
                                    }
                                }
                            }
                        }
                    }
                }
                break;
            case BEFORE_SUB_DRIVE:
                if(intake.getCurrentPosition() < 15){
                    setPathState(AutoState.SUB_DRIVE);
                }
                break;
            case SUB_DRIVE:
                intake.setUseColorSensor(false);
                if(pathTimer.getElapsedTimeSeconds() > 0){
                    intake.setClawState(Intake_DiffyClaw.CLAW_STATE.LOOSE);
                    if (pathTimer.getElapsedTimeSeconds() > 0.3){
                        intake.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER);
                        if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                            outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER);
                            if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                                outtake.setClawState(Outtake.ClawStates.CLOSED);
                                if (pathTimer.getElapsedTimeSeconds() > 0.9) {
                                    intake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                                        outtake.setOuttakeState(Outtake.OuttakeState.SAMPLE_SCORE_WAIT);
                                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.LIFT_HIGH_BASKET);
                                        setPathState(AutoState.SUB_SCORE_WAIT);
                                    }
                                }
                            }
                        }
                    }
                }
                break;
            case SUB_SCORE_WAIT:
                ll.turnOff();
                if (!follower.isBusy() && !outtakeLift.isBusy()){
                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLESCORE);
                    setPathState(AutoState.SUB_SCORE);
                }
                break;
            case SUB_SCORE:
                if (pathTimer.getElapsedTimeSeconds() > 0.3){
                    outtake.setClawState(Outtake.ClawStates.OPEN);
                    setPathState(AutoState.SUB_SCORE_DONE);
                }
                break;
            case SUB_SCORE_DONE:
                sampleCounter++;
                if (sampleCounter >= 1){ //number is how many submersible samples you wanna score.
                    setPathState(AutoState.PARK);
                    follower.followPath(park);
                } else {
                    if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                        follower.followPath(driveToVision, true);
                        outtake.setOuttakeState(Outtake.OuttakeState.SAMPLE_SCORE_WAIT);
                    }
                    if (pathTimer.getElapsedTimeSeconds()> 0.7){
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.TRANSFER);
                        outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER_WAIT);
                        setPathState(AutoState.VISION);
                    }
                }
                break;
            case PARK:
                Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
                if (pathTimer.getElapsedTimeSeconds() > 0.6){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.TRANSFER);
                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLE_SCORE_WAIT);
                }
                if (!follower.isBusy()){
                    outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCORE);
                    setPathState(AutoState.KILL);
                }
                break;
            case KILL:
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    stop();
                    requestOpModeStop();
                    terminateOpModeNow();
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
        ll.setPipelineNumber(4);
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