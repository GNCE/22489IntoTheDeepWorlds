import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import subsystems.IntakeLimelightSubsys;
import subsystems.OuttakeLiftSubsys;

@Autonomous (name = "0+4 auton pls worky")
public class Auto_0_4 extends OpMode{
    private Follower follower;
    private Intake_DiffyClaw intake;
    private OuttakeLiftSubsys outtakeLift;
    private Outtake outtake;
    private IntakeLimelightSubsys ll;
    private Timer pathTimer, opmodeTimer;
    private ElapsedTime outtakeSequenceTime, intakeSequenceTime, resetEncoderDelay;
    /** Start Pose of our robot */
    private final Pose startPose = new Pose(7.35, 113.625, Math.toRadians(270));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(13, 130, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(24, 121, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(24, 131, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(24, 132, Math.toRadians(15));

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
        PICKUP,
        TRANSFER_SAMPLE,
        OUTTAKE_GRAB_SAMPLE,
        TRANSFER_DONE,
        READY_TO_SCORE,
        SCORE,
        PARK
    }
    private AutoState autoState = AutoState.DRIVE_TO_PRELOAD_SCORE;
    int sampleCounter = 0;
    public void autonomousPathUpdate() {
        switch (autoState) {
            case DRIVE_TO_PRELOAD_SCORE:
                follower.followPath(scorePreload, true);
                outtakeSequenceTime.reset();
                bucketSequence = BUCKET_SEQUENCE.GRAB_AND_LIFT;
                setPathState(AutoState.SCORE_PRELOAD);
                break;
            case SCORE_WAIT:
                if (!follower.isBusy()){
                    outtakeSequenceTime.reset();
                    setPathState(AutoState.SCORE_PRELOAD);
                }
                break;
            case SCORE_PRELOAD:
                bucketSequence = BUCKET_SEQUENCE.SCORE;
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(grabPickups[sampleCounter], true);
                }
                if (pathTimer.getElapsedTimeSeconds()> 2){
                    outtakeSequenceTime.reset();
                    setPathState(AutoState.INTAKE_SAMPLE);
                }
                break;
            case INTAKE_WAIT:
                bucketSequence = BUCKET_SEQUENCE.SCORE;
                if ( pathTimer.getElapsedTimeSeconds() > 1.5) {
                    intakeSequenceTime.reset();
                    setPathState(AutoState.INTAKE_SAMPLE);
                }
                break;
            case INTAKE_SAMPLE:
                bucketSequence = BUCKET_SEQUENCE.TRANSFER;
                if (!follower.isBusy()){
                    follower.breakFollowing();
                    follower.startTeleopDrive();
                    intakeSequenceTime.reset();
                    setPathState(AutoState.PICKUP);
                }
                break;
            case PICKUP:
                intakeSequence = INTAKE_SEQUENCE.READY;
                if (pathTimer.getElapsedTimeSeconds() > 3){
                    intakeSequenceTime.reset();
                    setPathState(AutoState.TRANSFER_SAMPLE);
                }
                break;
            case TRANSFER_SAMPLE:
                follower.breakFollowing();
                intakeSequence = INTAKE_SEQUENCE.GRAB;
                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                    intakeSequenceTime.reset();
                    setPathState(AutoState.OUTTAKE_GRAB_SAMPLE);
                }
                break;
            case OUTTAKE_GRAB_SAMPLE:
                intakeSequence = INTAKE_SEQUENCE.TRANSFER_WAIT;
                follower.followPath(scorePickups[sampleCounter],true);
                if (pathTimer.getElapsedTimeSeconds() > 1.5){
                    outtakeSequenceTime.reset();
                    setPathState(AutoState.READY_TO_SCORE);
                }
                break;
            case READY_TO_SCORE:
                bucketSequence = BUCKET_SEQUENCE.TRANSFER;
                if (pathTimer.getElapsedTimeSeconds() > 1.5){
                    outtakeSequenceTime.reset();
                    setPathState(AutoState.SCORE);
                }
                break;
            case SCORE:
                bucketSequence= BUCKET_SEQUENCE.GRAB_AND_LIFT;
                sampleCounter++;
                if (!follower.isBusy() && sampleCounter < 3){
                    outtakeSequenceTime.reset();
                    setPathState(AutoState.INTAKE_WAIT);
                } else {
                    setPathState(AutoState.PARK);
                }
                break;
            case PARK:
                break;
            default:
                break;
        }
    }
    public void setPathState(AutoState newState) {
        autoState = newState;
        pathTimer.resetTimer();
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        outtake = new Outtake(hardwareMap);

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
        outtake.outtakeLoop();
        telemetry.addData("Team Color:", Storage.isRed ? "Red" : "Blue");
        telemetry.update();
    }

    public static double mx =  -0.012, my =  -0.012;
    public static double targetX = 16, targetY = 0;
    @Override
    public void loop() {
        intake.loop();
        intake.HoldExtension();
        ll.loop();
        outtakeLift.loop();
        outtakeLift.holdLift();
        outtake.outtakeLoop();
        autonomousPathUpdate();
        if (intakeSequence != INTAKE_SEQUENCE.READY){
        follower.update();}
        switch (intakeSequence){
            case READY:
                intake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                intake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.FULL_EXTENSION);
                ll.turnOn();
                intake.setClawOpen(Intake_DiffyClaw.CLAW_STATE.OPEN);
                if (ll.isRunning() && ll.isResultValid()) {
                    follower.setTeleOpMovementVectors((targetX - ll.getTx()) * mx, (targetY -  ll.getTy()) * my, 0);
                    double angle = ll.getAngle(); // Output 0 is sample angle
                    if(Math.abs(angle) > 85){
                        if(Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED >= 0) angle = 85;
                        else angle = -85;
                    }
                    if(angle < -90) angle = -90;
                    else if(angle > 90) angle = 90;

                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = angle * 10.5/9;
                }
                break;
            case GRAB:
                intake
                        .setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
                ll.turnOff();
                intake
                        .ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.FULL_EXTENSION);
                if (intakeSequenceTime.time() > 0.2){
                    intake
                            .setClawOpen(Intake_DiffyClaw.CLAW_STATE.CLOSED);
                }
                if (intakeSequenceTime.time() > 0.4){
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
                    intake
                            .setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                }
                break;
            case TRANSFER_WAIT:
                ll.turnOff();
                    intake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                    if (intake.extensionReachedTarget()){
                        intake.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER_WAIT);
                    }
                break;
            }

        switch (bucketSequence){
            case TRANSFER:
                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.TRANSFER);
                outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER);
                outtake.setClawOpen(true);
                break;
            case GRAB_AND_LIFT:
                intake.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER);
                if (outtakeSequenceTime.time() > 0.23){
                    outtake.setClawOpen(false);
                }
                if (outtakeSequenceTime.time() > 0.74){
                    intake.setClawOpen(Intake_DiffyClaw.CLAW_STATE.OPEN);
                }
                if(outtakeSequenceTime.time() > 1){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.LIFT_BUCKET);
                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLESCORE);
                    resetEncoderDelay.reset();
                }
                break;
            case SCORE:
                outtake.setClawOpen(true);
                if (resetEncoderDelay.time() > 0.8){
                    outtake.setOuttakeState(Outtake.OuttakeState.RESET_ENCODER);
                }
                break;
            case RESET:
                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.RESET_ENCODER);
                break;
        }
        Storage.CurrentPose = follower.getPose();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}