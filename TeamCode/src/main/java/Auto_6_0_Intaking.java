import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants_5_0;
import pedroPathing.constants.FConstants_6_0;
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

@Config
@Autonomous (name = "6 + 0 Autonomous - Intaking")
public class Auto_6_0_Intaking extends OpMode {
    private Follower follower;
    private Intake_DiffyClaw intakeDiffyClaw;
    private OuttakeLiftSubsys outtakeLift;
    private Outtake outtake;
    private IntakeLimelightSubsys ll;
    private Timer pathTimer;
    private ElapsedTime loopTime;
    private LynxModules lynxModules;
    private MedianSmoother medianSmoother;
    private final double scoreX = 44.25;
    private final double scoreY = 65;

    private final double frontScoreX = 42.3;

    private final double scoreControlX = 23;
    private final double scoreYChange = 0;
    private final Pose startPose = new Pose(8.55, 63.5, Math.toRadians(0));
    private final Pose preloadScorePose = new Pose(frontScoreX, 69, Math.toRadians(0));
    private final Pose thirdSpikeMarkControl = new Pose(22, 55, Math.toRadians(0));
    private final Pose thirdSpikeMark = new Pose(24.25, 10, Math.toRadians(-18));
    private final Pose secondSpikeMark = new Pose(22.25, 12.2, Math.toRadians(0));
    private final Pose firstSpikeMark = new Pose(22.25, 22.2, Math.toRadians(0));

    private final Pose outtakeFirstPickupPose = new Pose(17.6, 23.45960264900662, Math.toRadians(0));
    private final Pose outtakePickupPose = new Pose(15, 35, Math.toRadians(180));
    private final Pose outtakePickupWaitPose = new Pose(22.69420, 35, Math.toRadians(180));

    private final Pose firstScorePose = new Pose(frontScoreX, 67, Math.toRadians(0));
    private final Pose firstScoreReturnPose = new Pose(32.805298013245036, 64.65695364238411, Math.toRadians(181));

    private final Pose secondScorePose = new Pose(scoreX, scoreY-scoreYChange*2, Math.toRadians(230));
    private final Pose thirdScorePose = new Pose(scoreX, scoreY-scoreYChange*3, Math.toRadians(230));
    private final Pose twohundeAngle = new Pose(scoreX, scoreY-scoreYChange*3, Math.toRadians(192.5));
    private final Pose fourthScorePose = new Pose(scoreX, scoreY-scoreYChange*4, Math.toRadians(230));
    private final Pose fifthScorePose = new Pose(scoreX, scoreY-scoreYChange*5, Math.toRadians(230));
    private final Pose parkPose = new Pose(26, 43, Math.toRadians(237.5));

    // TODO: ZPAM VARIABLES
    private final double zeroPowerAccelerationMultiplierForPICKUP_WAIT = 1.9;
    private final double zeroPowerAccelerationMultiplerForScore = 4.5;
    private final double pathEndTValueForScore = 0.93;
    private final double pathEndTimeoutForScore = 2.5;
    private PathChain scorePreloadPath, parkFromFifthPath;
    private PathChain spike3, spike2, spike1;
    private PathChain firstPickupPath, secondPickupPath, secondFinal, thirdPickupPath, fourthPickupPath, fifthPickupPath;
    private PathChain firstScorePath, secondScorePath, thirdScorePath, fourthScorePath, fifthScorePath;
    private PathChain[] pickupPaths, scorePaths;
    public void buildPaths(){
        scorePreloadPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadScorePose)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .setZeroPowerAccelerationMultiplier(2.2)
                .setPathEndTValueConstraint(0.95)
                .build();
        spike3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadScorePose), new Point(thirdSpikeMarkControl), new Point(thirdSpikeMark)))
                .setLinearHeadingInterpolation(preloadScorePose.getHeading(), thirdSpikeMark.getHeading())
                .setZeroPowerAccelerationMultiplier(1.8)
                .setPathEndTimeoutConstraint(75)
                .setPathEndTValueConstraint(0.995)
                .build();
        spike2 = follower.pathBuilder()
                .addPath(new BezierLine(thirdSpikeMark, secondSpikeMark))
                .setConstantHeadingInterpolation(secondSpikeMark.getHeading())
                .setZeroPowerAccelerationMultiplier(1.3)
                .setPathEndTimeoutConstraint(10)
                .setPathEndTValueConstraint(0.875)
                .build();
        spike1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSpikeMark), new Point(firstSpikeMark)))
                .setLinearHeadingInterpolation(secondSpikeMark.getHeading(), firstSpikeMark.getHeading())
                .setZeroPowerAccelerationMultiplier(1.3)
                .setPathEndTimeoutConstraint(10)
                .setPathEndTValueConstraint(0.875)
                .build();
        firstPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstSpikeMark), new Point(outtakeFirstPickupPose)))
                .setConstantHeadingInterpolation(firstSpikeMark.getHeading())
                .setZeroPowerAccelerationMultiplier(2.1)
                .setPathEndTValueConstraint(0.9)
                .setPathEndTimeoutConstraint(75)
                .build();
        firstScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakeFirstPickupPose), new Point(firstScorePose)))
                .setConstantHeadingInterpolation(outtakeFirstPickupPose.getHeading())
                .setZeroPowerAccelerationMultiplier(1.7)
                .setPathEndTimeoutConstraint(50)
                .setPathEndTValueConstraint(0.93)
                .build();
        secondPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstScorePose), new Point(outtakePickupWaitPose)))
                .setLinearHeadingInterpolation(firstScorePose.getHeading(), outtakePickupWaitPose.getHeading())
                .setPathEndTimeoutConstraint(10)
                .setPathEndTValueConstraint(0.69420) //0.82
                .setZeroPowerAccelerationMultiplier(1.69420)
                .build();
        secondFinal = follower.pathBuilder()
                .addPath(new BezierLine(new Point(outtakePickupWaitPose), new Point(outtakePickupPose)))
                .setConstantHeadingInterpolation(outtakePickupWaitPose.getHeading())
                .setPathEndTimeoutConstraint(5)
                .setPathEndTValueConstraint(0.98)
                .setZeroPowerAccelerationMultiplier(1.867676767)
                .build();
        secondScorePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(outtakePickupPose), new Point(secondScorePose)))
                .setLinearHeadingInterpolation(secondScorePose.getHeading(),twohundeAngle.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .setPathEndTValueConstraint(pathEndTValueForScore)
                .setPathEndTimeoutConstraint(pathEndTimeoutForScore)
                .build();
        thirdPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondScorePose), new Point(outtakePickupPose)))
                .setLinearHeadingInterpolation(secondScorePose.getHeading(), outtakePickupPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPICKUP_WAIT)
                .setPathEndTimeoutConstraint(25)
                .setPathEndTValueConstraint(0.95)
                .build();
        thirdScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupPose),new Point(thirdScorePose)))
                .setLinearHeadingInterpolation(secondScorePose.getHeading(),twohundeAngle.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .setPathEndTValueConstraint(pathEndTValueForScore)
                .setPathEndTimeoutConstraint(pathEndTimeoutForScore)
                .build();
        fourthPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdScorePose), new Point(outtakePickupPose)))
                .setLinearHeadingInterpolation(thirdScorePose.getHeading(), outtakePickupPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPICKUP_WAIT)
                .setPathEndTimeoutConstraint(25)
                .setPathEndTValueConstraint(0.95)
                .build();
        fourthScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupPose), new Point(fourthScorePose)))
                .setLinearHeadingInterpolation(secondScorePose.getHeading(),twohundeAngle.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .setPathEndTValueConstraint(pathEndTValueForScore)
                .setPathEndTimeoutConstraint(pathEndTimeoutForScore)
                .build();
        fifthPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fourthScorePose), new Point(outtakePickupPose)))
                .setLinearHeadingInterpolation(fourthScorePose.getHeading(), outtakePickupPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPICKUP_WAIT)
                .setPathEndTimeoutConstraint(25)
                .setPathEndTValueConstraint(0.95)
                .build();
        fifthScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupPose), new Point(fifthScorePose)))
                .setLinearHeadingInterpolation(secondScorePose.getHeading(),twohundeAngle.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .setPathEndTValueConstraint(pathEndTValueForScore)
                .setPathEndTimeoutConstraint(pathEndTimeoutForScore)
                .build();
        parkFromFifthPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fifthScorePose), new Point(parkPose)))
                .setLinearHeadingInterpolation(fifthScorePose.getHeading(), parkPose.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .setPathEndTValueConstraint(0.875)
                .build();
        pickupPaths = new PathChain[]{firstPickupPath, secondPickupPath, thirdPickupPath, fourthPickupPath, fifthPickupPath};
        scorePaths = new PathChain[]{firstScorePath, secondScorePath, thirdScorePath, fourthScorePath, fifthScorePath};
    }

    enum AutoState {
        DRIVE_TO_PRELOAD_SCORE,
        PRELOAD_AT_SUB,
        SCORE_PRELOAD,
        VISION_MOVE,
        VISION_MOVED,
        GO_DRIVE,
        DRIVE_TO_SPIKE_3,
        AT_SPIKE_3,
        TO_SPIKE_2,
        AT_SPIKE_2,
        PICKUP_SPIKE_2,
        TO_SPIKE_1,
        AT_SPIKE_1,
        PICKUP_SPIKE_1,

        BEFORE_PICKUP,
        PICKINGUPFIRST,
        GOTOFIRSTSCORE,
        FRONTSCOREFIRST,
        FRONTSCOREFIRSTA,
        WALL_PICKUP,
        WALL_DELAY,
        READY_FOR_PICKUP,
        PICKUP,
        READY_TO_SCORE,
        SCORE,
        PARK
    }
    private AutoState autoState = AutoState.DRIVE_TO_PRELOAD_SCORE;

    public void setPathState(AutoState newState){
        autoState = newState;
        pathTimer.resetTimer();
    }
    public static double visionWaitTime = 0.2;

    private int counter = 0;
    public void autonomousPathUpdate(){
        switch (autoState){
            case DRIVE_TO_PRELOAD_SCORE:
                outtake.setClawState(Outtake.ClawStates.CLOSED);
                outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTSCOREWAIT);
                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_SCORE);
                follower.followPath(scorePreloadPath,true);
                setPathState(AutoState.SCORE_PRELOAD);
                break;
            case SCORE_PRELOAD:
                if(pathTimer.getElapsedTimeSeconds() > 0.2){
                    intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.VISION);
                }
                if(!follower.isBusy()){
                    setPathState(AutoState.PRELOAD_AT_SUB);
                }
                break;
            case PRELOAD_AT_SUB:
                if(pathTimer.getElapsedTimeSeconds() > 0){
                    outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTSCOREDONE);
                    if(pathTimer.getElapsedTimeSeconds() > 0.1) {
                        if(ll.isResultValid()) {
                            medianSmoother.add(ll.getHoriz(), ll.getVert(), ll.getAngle());
                        }
                        if (pathTimer.getElapsedTimeSeconds() > 0.275) {
                            outtake.setClawState(Outtake.ClawStates.OPEN);
                            if (pathTimer.getElapsedTimeSeconds() > 0.575 + visionWaitTime) { // Time it takes for claw to open
                                MedianSmoother.Sample detectedSample = medianSmoother.getMedian();
                                if (medianSmoother.getSize() > 0) {
                                    follower.followPath(
                                            follower.pathBuilder()
                                                    .addPath(new BezierLine(preloadScorePose, new Pose(preloadScorePose.getX(), preloadScorePose.getY() - detectedSample.getX())))
                                                    .setConstantHeadingInterpolation(preloadScorePose.getHeading())
                                                    .setZeroPowerAccelerationMultiplier(1.8)
                                                    .setPathEndTValueConstraint(0.75)
                                                    .setPathEndTimeoutConstraint(100)                                                    .build(),
                                            true
                                    );
                                    intakeDiffyClaw.ExtendTo(detectedSample.getY(), Intake_DiffyClaw.ExtensionUnits.ticks);
                                    intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = detectedSample.getAngle() * 110 / 90;
                                    intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                                    setPathState(AutoState.VISION_MOVE);
                                } else {
                                    setPathState(AutoState.GO_DRIVE);
                                }
                            }
                        }
                    }
                }
                break;
            case VISION_MOVE:
                if(!follower.isBusy() && intakeDiffyClaw.extensionReachedTarget()){
                    setPathState(AutoState.VISION_MOVED);
                }
                break;
            case VISION_MOVED:
                if(pathTimer.getElapsedTimeSeconds() > 0.1){
                    intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
                    if(pathTimer.getElapsedTimeSeconds() > 0.3){
                        intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.CLOSED);
                        if(pathTimer.getElapsedTimeSeconds() > 0.5){
                            intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_RETRACT_HOLD);
                            intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.LOOSE);
                            intakeDiffyClaw.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                            if(intakeDiffyClaw.getCurrentPosition() < 50){
                                setPathState(AutoState.GO_DRIVE);
                            }
                        }
                    }
                }
                break;
            case GO_DRIVE:
                follower.followPath(spike3, true);
                setPathState(AutoState.DRIVE_TO_SPIKE_3);
                break;
            case DRIVE_TO_SPIKE_3:
                ll.turnOff();
                if (pathTimer.getElapsedTimeSeconds() > 0.15){
                    outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER_WAIT);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.TRANSFER);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.4){
                    intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER);
                }
                                if(pathTimer.getElapsedTimeSeconds() > 0.7){
                                    outtake.setClawState(Outtake.ClawStates.CLOSED);
                                    if (pathTimer.getElapsedTimeSeconds() >0.9){
                                    intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.SPIKE);
                                    if(pathTimer.getElapsedTimeSeconds() > 1.1){
                                        outtake.setOuttakeState(Outtake.OuttakeState.AUTO_SAMPLE_DEPOSIT);
                                        intakeDiffyClaw.setDontReset(true);
                                        intakeDiffyClaw.ExtendTo(380, Intake_DiffyClaw.ExtensionUnits.ticks);
                                        intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                                        Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = -30;
                                    }
                                }
                }
                if(!follower.isBusy()){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.AUTO_THROW);
                    outtake.setClawState(Outtake.ClawStates.OPEN);
                    setPathState(AutoState.AT_SPIKE_3);
                    // TODO: Vision
                }
                break;
            case AT_SPIKE_3:
                if(pathTimer.getElapsedTimeSeconds() > 0.4){
                    intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
                    if(pathTimer.getElapsedTimeSeconds() > 0.5){
                        intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.CLOSED);
                    }
                    if(pathTimer.getElapsedTimeSeconds() > 0.65){
                        intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.DEPOSIT);
                        intakeDiffyClaw.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                        follower.followPath(spike2, true);
                        setPathState(AutoState.TO_SPIKE_2);
                    }
                }
                break;
            case TO_SPIKE_2:
                if(intakeDiffyClaw.getCurrentPosition() < 270){
                    intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.LOOSE);
                }
                if(intakeDiffyClaw.getCurrentPosition() < 50){
                    intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                }
                if(!follower.isBusy() && intakeDiffyClaw.getCurrentPosition() < 10){
                    setPathState(AutoState.AT_SPIKE_2);
                }
                break;
            case AT_SPIKE_2:
                if(pathTimer.getElapsedTimeSeconds() > 0.2){
                    intakeDiffyClaw.ExtendTo(380, Intake_DiffyClaw.ExtensionUnits.ticks);
                    intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.SPIKE);
                    intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
                    if(intakeDiffyClaw.extensionReachedTarget()){
                        setPathState(AutoState.PICKUP_SPIKE_2);
                    }
                }
                break;
            case PICKUP_SPIKE_2:
                intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
                if(pathTimer.getElapsedTimeSeconds() > 0.15){
                    intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.CLOSED);
                    if(pathTimer.getElapsedTimeSeconds() > 0.3){
                        follower.followPath(spike1, true);
                        setPathState(AutoState.TO_SPIKE_1);
                        intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.DEPOSIT);
                        intakeDiffyClaw.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                    }
                }
                break;
            case TO_SPIKE_1:
                if(intakeDiffyClaw.getCurrentPosition() < 35){
                    intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.LOOSE);
                }
                if(!follower.isBusy() && intakeDiffyClaw.getCurrentPosition() < 8){
                    setPathState(AutoState.AT_SPIKE_1);
                }
                break;
            case AT_SPIKE_1:
                intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                if(pathTimer.getElapsedTimeSeconds() > 0.25){
                    intakeDiffyClaw.ExtendTo(380, Intake_DiffyClaw.ExtensionUnits.ticks);
                    intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                    intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.SPIKE);
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
                    if(intakeDiffyClaw.extensionReachedTarget()){
                        setPathState(AutoState.PICKUP_SPIKE_1);
                    }
                }
                break;
            case PICKUP_SPIKE_1:
                intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
                if(pathTimer.getElapsedTimeSeconds() > 0.2){
                    intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.CLOSED);
                    if(pathTimer.getElapsedTimeSeconds() > 0.35){
                        counter = 0;
                        follower.followPath(pickupPaths[counter], true);
                        setPathState(AutoState.BEFORE_PICKUP);
                        intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.DEPOSIT);
                        intakeDiffyClaw.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                        outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKPICKUP);
                    }
                }
                break;
            case BEFORE_PICKUP:
                if (intakeDiffyClaw.getCurrentPosition() < 270){
                    intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.LOOSE);
                }
                if(intakeDiffyClaw.getCurrentPosition() < 50){
                    intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                    setPathState(AutoState.PICKINGUPFIRST);
                }
                break;
            case PICKINGUPFIRST:
                if(pathTimer.getElapsedTimeSeconds() > 0.4){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.BACK_PICKUP);
                    if(outtakeLift.getCurrentPosition() < 10 && !follower.isBusy()){
                        outtake.setClawState(Outtake.ClawStates.CLOSED);
                        setPathState(AutoState.GOTOFIRSTSCORE);
                    }
                }
                break;
            case GOTOFIRSTSCORE:
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.AUTO_FRONT);
                    outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTSCOREWAIT);
                    follower.followPath(scorePaths[counter], true);
                    setPathState(AutoState.FRONTSCOREFIRST);
                }
                break;
            case FRONTSCOREFIRST:
                if(!follower.isBusy()){
                    setPathState(AutoState.FRONTSCOREFIRSTA);
                }
                break;
            case FRONTSCOREFIRSTA:
                if(pathTimer.getElapsedTimeSeconds() > 0){
                    outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTSCOREDONE);
                    if(pathTimer.getElapsedTimeSeconds() > 0.275){
                        outtake.setClawState(Outtake.ClawStates.OPEN);
                        if(pathTimer.getElapsedTimeSeconds() > 0.575){
                            setPathState(AutoState.READY_FOR_PICKUP);
                            follower.followPath(pickupPaths[++counter], true);
                        }
                    }
                }
                break;
            case READY_FOR_PICKUP:
                outtake.setClawState(Outtake.ClawStates.OPEN);
                if(pathTimer.getElapsedTimeSeconds() > 0.525){
                    outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
                }
                if(!follower.isBusy()){
                    follower.followPath(secondFinal,true);
                    setPathState(AutoState.WALL_DELAY);
                }
                break;
            case WALL_DELAY:
                if(!follower.isBusy()){
                    setPathState(AutoState.PICKUP);
                }
                break;
            case PICKUP:
                if(pathTimer.getElapsedTimeSeconds() > 0){
                    outtake.setClawState(Outtake.ClawStates.CLOSED);
                    if(pathTimer.getElapsedTimeSeconds() > 0.32){
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.BACK_SCORE);
                        outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCORE);
                        follower.followPath(scorePaths[counter], true);
                        setPathState(AutoState.READY_TO_SCORE);
                    }
                }
                break;
            case READY_TO_SCORE:
                if(!follower.isBusy()){
                    setPathState(AutoState.SCORE);
                }
                break;
            case SCORE:
                if(pathTimer.getElapsedTimeSeconds() > 0.04){
                    outtake.setClawState(Outtake.ClawStates.OPEN);
                    if (pathTimer.getElapsedTimeSeconds() > 0.26) {
                        counter++;
                        if (counter < 5) {
                            outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCOREOUT);
                            follower.followPath(pickupPaths[counter], true);
                            setPathState(AutoState.READY_FOR_PICKUP);
                        } else {
                            outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCOREOUT);
                            follower.followPath(parkFromFifthPath, false);
                            setPathState(AutoState.PARK);
                        }
                    }
                }
                break;
            case PARK:
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    intakeDiffyClaw.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.FULL_EXTENSION);
                    intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                }
                if (!follower.isBusy()){
                    stop();
                    requestOpModeStop();
                    terminateOpModeNow();
                }
                break;
            default:
                break;
        }
    }
    @Override
    public void init(){
        pathTimer = new Timer();
        loopTime = new ElapsedTime();
        loopTime.startTime();
        follower = new Follower(hardwareMap, FConstants_6_0.class, LConstants.class);
        follower.setStartingPose(startPose);

        tel = new UnifiedTelemetry();
        tel.init(this.telemetry);
        SubsysCore.setGlobalParameters(hardwareMap, this);

        lynxModules = new LynxModules();
        lynxModules.init();

        ll = new IntakeLimelightSubsys();
        ll.init();
        intakeDiffyClaw = new Intake_DiffyClaw();
        intakeDiffyClaw.init();
        outtake = new Outtake();
        outtake.init();
        outtake.setAlignedTo(0);
        outtakeLift = new OuttakeLiftSubsys();
        outtakeLift.init();
        medianSmoother = new MedianSmoother(300);
        buildPaths();
    }

    private final ToggleButton teamColorButton = new ToggleButton(Storage.isRed);
    private UnifiedTelemetry tel;
    @Override
    public void init_loop(){
        ll.turnOn();
        lynxModules.loop();
        teamColorButton.input(gamepad1.dpad_up);
        Storage.isRed = teamColorButton.getVal();
        ll.setPipelineNumber(4);
        ll.setAlliance(Storage.isRed ? IntakeLimelightSubsys.Alliance.RED : IntakeLimelightSubsys.Alliance.BLUE);
        ll.setSampleType(IntakeLimelightSubsys.SampleType.ALLIANCE);

        outtake.setOuttakeState(Outtake.OuttakeState.Auto_Wait);
        outtake.setClawState(Outtake.ClawStates.CLOSED);
        outtake.loop();

        ll.loop();
        tel.addData("Team Color:", Storage.isRed ? "Red" : "Blue");
        tel.update();
    }
    @Override
    public void loop(){
        lynxModules.loop();

        follower.update();
        autonomousPathUpdate();
        ll.loop();
        outtake.loop();
        outtake.setAlignedTo(10);
        outtakeLift.holdLift();
        outtakeLift.loop();
        intakeDiffyClaw.auto = true;
        intakeDiffyClaw.loop();
        intakeDiffyClaw.HoldExtension();


        Storage.CurrentPose = follower.getPose();
        tel.addData("AutoState", autoState.name());
        tel.addData("TValue", follower.getCurrentTValue());
        tel.addData("VeloX", follower.getVelocity().getMagnitude());
        tel.addData("XPOS", follower.getPose().getX());
        tel.addData("YPOS", follower.getPose().getY());
        tel.addData("translational error", follower.getTranslationalError().getMagnitude());
        tel.addData("LoopTime", loopTime);
        loopTime.reset();
        tel.update();
    }
}