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

@Autonomous (name = "0 + 6 NV W/Park but less consistent i think idk")
public class Auto_0_6_7_NV extends OpMode{
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
    private final Pose scorePose = new Pose(14, 130, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(24.5, 119, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(25, 129.6, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(27, 131, Math.toRadians(25));
    private final Pose visionPose = new Pose(62, 96.5, Math.toRadians(270));
    private final Pose visionControlP = new Pose(52, 122, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(60, 87.5, Math.toRadians(270));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(60, 120, Math.toRadians(270));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, driveToVision, driveFromVision, parkFromVision;
    private PathChain[] grabPickups, scorePickups;
    ElapsedTime elapsedTime;
    public static double visionWaitTime = 0.55;
    private final double ZPAM = 1.8;
    private final double LPETC = 10;
    private final double LPETVC = 0.8;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload.setZeroPowerAccelerationMultiplier(1.3);
        scorePreload.setPathEndTValueConstraint(0.8);
        scorePreload.setPathEndTimeoutConstraint(LPETC);

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */


        //DEAR THE GODS OF FTC PLEASE NO OVERSHOOT
//DEAR THE GODS OF FTC PLEASE NO OVERSHOOT
//DEAR THE GODS OF FTC PLEASE NO OVERSHOOT
//DEAR THE GODS OF FTC PLEASE NO OVERSHOOT
//DEAR THE GODS OF FTC PLEASE NO OVERSHOOT
//DEAR THE GODS OF FTC PLEASE NO OVERSHOOT
//DEAR THE GODS OF FTC PLEASE NO OVERSHOOT
//DEAR THE GODS OF FTC PLEASE NO OVERSHOOT
//DEAR THE GODS OF FTC PLEASE NO OVERSHOOT
//DEAR THE GODS OF FTC PLEASE NO OVERSHOOT
//DEAR THE GODS OF FTC PLEASE NO OVERSHOOT


        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(0.567)
                .setPathEndTValueConstraint(0.775)
                .setPathEndTimeoutConstraint(LPETC)
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(1.3)
                .setPathEndTValueConstraint(0.8)
                .setPathEndTimeoutConstraint(5)
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(0.420)
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
                .setZeroPowerAccelerationMultiplier(0.567)
                .setPathEndTValueConstraint(0.76)
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
                .setZeroPowerAccelerationMultiplier(0.67)
                .setPathEndTValueConstraint(0.82)
                .setPathEndTimeoutConstraint(5)
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
                .setZeroPowerAccelerationMultiplier(0.7)
                .setPathEndTValueConstraint(0.69)
                .setPathEndTimeoutConstraint(5)
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
        park.setZeroPowerAccelerationMultiplier(2);
        park.setPathEndTValueConstraint(0.76);
        park.setPathEndTimeoutConstraint(LPETC);

        grabPickups = new PathChain[]{grabPickup1, grabPickup2, grabPickup3};
        scorePickups = new PathChain[]{scorePickup1, scorePickup2, scorePickup3};
    }
    enum AutoState {
        DRIVE_TO_PRELOAD_SCORE,
        SCORE_WAIT,
        SCORE_PRELOAD,
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
        VISION_PICKUP,
        SUB_DRIVE,
        SUB_SCORE,
        SUB_SCORE_WAIT,
        SUB_SCORE_DONE,
        SUB_fix,
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
                Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
                setPathState(AutoState.SCORE_WAIT);
                sampleCounter = 0;
                break;
            case SCORE_WAIT:
                intake.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1){
                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLESCORE);
                    setPathState(AutoState.SCORE_PRELOAD);
                }
                break;
            case SCORE_PRELOAD:
                if (pathTimer.getElapsedTimeSeconds() > 0.25){
                    outtake.setClawState(Outtake.ClawStates.OPEN);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLE_SCORE_WAIT);
                }
                if (pathTimer.getElapsedTimeSeconds()> 0.6){
                    follower.followPath(grabPickups[sampleCounter], true);
                    setPathState(AutoState.INTAKE_WAIT);
                }
                break;
            case INTAKE_WAIT:
                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.TRANSFER);
                outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER);
                intake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.AUTO_POS);
                intake.setClawState(Intake_DiffyClaw.CLAW_STATE.SPIKE);
                intake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                if (!follower.isBusy()) {
                    if(sampleCounter == 2){
                        Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 40;
                    }
                    setPathState(AutoState.INTAKE_SAMPLE);
                }
                break;
            case INTAKE_SAMPLE:
                if (pathTimer.getElapsedTimeSeconds() > 0.25) { //0.7
                    intake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
                    setPathState(AutoState.PICKUP);
                }
                break;
            case PICKUP:
                if (pathTimer.getElapsedTimeSeconds() > 0.167){
                    intake.setClawState(Intake_DiffyClaw.CLAW_STATE.CLOSED);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.3){
                    intake.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.4){
                    intake.setClawState(Intake_DiffyClaw.CLAW_STATE.LOOSE);
                    intake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                    follower.followPath(scorePickups[sampleCounter],true);
                    setPathState(AutoState.TRANSFER_SAMPLE);
                }
                break;
            case TRANSFER_SAMPLE:
                if(intake.getCurrentPosition() < 10){
                    outtake.setClawState(Outtake.ClawStates.CLOSED);
                    setPathState(AutoState.GRAB);
                }
                break;
            case GRAB:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    intake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.35) {
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.LIFT_HIGH_BASKET);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.425) {
                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLE_SCORE_WAIT);
                }
                if (!follower.isBusy() && outtakeLift.getCurrentPosition() > 1700) {
                    setPathState(AutoState.READY_TO_SCORE);
                }
                break;
            case READY_TO_SCORE:
                outtake.setOuttakeState(Outtake.OuttakeState.SAMPLESCORE);
                if(pathTimer.getElapsedTimeSeconds() > 0.25){
                    outtake.setClawState(Outtake.ClawStates.OPEN);
                }
                if(pathTimer.getElapsedTimeSeconds() > 0.4){
                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLE_SCORE_WAIT);
                    setPathState(AutoState.SCORE);
                }
                break;
            case SCORE:
                sampleCounter++;
                medianSmoother.clear();
                if (sampleCounter < 3){
                    follower.followPath(grabPickups[sampleCounter], true);
                    setPathState(AutoState.AFTER_SCORE);
                } else {
                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLE_SCORE_WAIT);
                    follower.followPath(driveToVision, true);
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
                    intake.setIntakeState(Intake_DiffyClaw.IntakeState.VISION);
                    setPathState(AutoState.VISION_MOVE);
                }
                break;
            case VISION_MOVE:
                if (pathTimer.getElapsedTimeSeconds() > 0.35) {
                    if(ll.isResultValid()) {
                        medianSmoother.add(ll.getVert(), ll.getHoriz(), ll.getAngle());
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 0.35 + visionWaitTime) {
                        MedianSmoother.Sample detectedSample = medianSmoother.getMedian();
                        if (medianSmoother.getSize() > 0) {
                            follower.followPath(
                                    follower.pathBuilder()
                                            .addPath(new BezierLine(visionPose, new Pose(visionPose.getX()- detectedSample.getY(), visionPose.getY())))
                                            .setConstantHeadingInterpolation(visionPose.getHeading())
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
                        } else if (pathTimer.getElapsedTimeSeconds() > 6){
                            follower.followPath(parkFromVision, true);
                            setPathState(AutoState.PARK);
                        }
                    }
                }
                break;
            case VISION_DONE:
                if(!follower.isBusy() && intake.extensionReachedTarget()){
                    setPathState(AutoState.VISION_PICKUP);
                }
                break;
            case VISION_PICKUP:
                if(pathTimer.getElapsedTimeSeconds() > 0.1){
                    intake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
                    if(pathTimer.getElapsedTimeSeconds() > 0.3){
                        intake.setClawState(Intake_DiffyClaw.CLAW_STATE.CLOSED);
                        if(pathTimer.getElapsedTimeSeconds() > 0.5){
                            intake.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER_WAIT);
                            intake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                            intake.setClawState(Intake_DiffyClaw.CLAW_STATE.LOOSE);
                            outtake.setClawState(Outtake.ClawStates.OPEN);
                            outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER_WAIT);
                            outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.TRANSFER);
                            if(intake.getCurrentPosition() < 15){
                                follower.followPath(driveFromVision, true);
                                setPathState(AutoState.SUB_DRIVE);
                            }
                        }
                    }
                }
                break;
            case SUB_DRIVE:
                intake.setClawState(Intake_DiffyClaw.CLAW_STATE.LOOSE);
                intake.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER);
                outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER);
                outtake.setClawState(Outtake.ClawStates.OPEN);
                if(pathTimer.getElapsedTimeSeconds() > 0.45){
                    outtake.setClawState(Outtake.ClawStates.CLOSED);
                    if(pathTimer.getElapsedTimeSeconds() > 0.6){
                        intake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                        if(pathTimer.getElapsedTimeSeconds() > 0.7){
                            outtake.setOuttakeState(Outtake.OuttakeState.SAMPLE_SCORE_WAIT);
                            outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.LIFT_HIGH_BASKET);
                            setPathState(AutoState.SUB_SCORE_WAIT);
                        }
                    }
                }
                break;
            case SUB_SCORE_WAIT:
                if (!follower.isBusy() && (Math.abs(outtakeLift.getCurrentPosition() - OuttakeLiftSubsys.target) < 200)){
                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLESCORE);
                    setPathState(AutoState.SUB_SCORE);
                }
                break;
            case SUB_SCORE:
                if (pathTimer.getElapsedTimeSeconds() > 0.25){
                    outtake.setClawState(Outtake.ClawStates.OPEN);
                    setPathState(AutoState.SUB_SCORE_DONE);
                }
                break;
            case SUB_SCORE_DONE:
                sampleCounter++;
                if (sampleCounter >= 2){ //number is how many submersible samples you wanna score.
                    medianSmoother.clear();
                    follower.followPath(park, true);
                    setPathState(AutoState.PARK);
                } else {
                    medianSmoother.clear();
                    follower.followPath(driveToVision, true);
                    setPathState(AutoState.SUB_fix);
                }
                break;
            case SUB_fix:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLE_SCORE_WAIT);
                }
                if (pathTimer.getElapsedTimeSeconds()> 0.6){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.TRANSFER);
                    outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER_WAIT);
                    setPathState(AutoState.VISION);
                }
                break;
            case PARK:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLE_SCORE_WAIT);
                }
                Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
                if (pathTimer.getElapsedTimeSeconds() > 0.6){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.TRANSFER);
                    outtake.setOuttakeState(Outtake.OuttakeState.PARK);
                }
//                if (!follower.isBusy()){
//                    setPathState(AutoState.KILL);
//                }
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