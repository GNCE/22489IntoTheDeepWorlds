import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants_5_0;
import pedroPathing.constants.LConstants;
import subsystems.Intake_DiffyClaw;
import subsystems.LynxModules;
import subsystems.Outtake;
import subsystems.OuttakeLiftSubsys;
import subsystems.SubsysCore;
import subsystems.UnifiedTelemetry;


@Autonomous (name = "6 + 0 Autonomous - Pushing")
public class Auto_6_0_Intaking extends OpMode {
    private Follower follower;
    private Intake_DiffyClaw intakeDiffyClaw;
    private OuttakeLiftSubsys outtakeLift;
    private Outtake outtake;
    private Timer pathTimer;
    private LynxModules lynxModules;
    private final double scoreX = 40;
    private final double scoreY = 69;

    private final double frontScoreX = 37.5;

    private final double scoreControlX = 23;
    private final double scoreYChange = 1.3;
    private final Pose startPose = new Pose(8.55, 63.5, Math.toRadians(0));
    private final Pose preloadScorePose = new Pose(frontScoreX, scoreY+scoreYChange, Math.toRadians(0));
    private final Pose thirdSpikeMark = new Pose(24.170860927152315, 12.0158940397351, Math.toRadians(-20));
    private final Pose secondSpikeMark = new Pose(23.170860927152315, 12.0158940397351, Math.toRadians(0));
    private final Pose firstSpikeMark = new Pose(23.170860927152315, 23.45960264900662, Math.toRadians(0));

    private final Pose outtakeFirstPickupPose = new Pose(18, 23.45960264900662, Math.toRadians(0));
    private final Pose outtakePickupPose = new Pose(14.4, 31.4, Math.toRadians(180));

    private final Pose firstScorePose = new Pose(frontScoreX, scoreY-scoreYChange, Math.toRadians(0));
    private final Pose firstScoreControl = new Pose(scoreControlX, scoreY-scoreYChange, Math.toRadians(0));
    private final Pose secondScorePose = new Pose(scoreX, scoreY-scoreYChange*2, Math.toRadians(180));
    private final Pose secondScoreControl = new Pose(scoreControlX, scoreY - scoreYChange*2, Math.toRadians(180));
    private final Pose thirdScorePose = new Pose(scoreX, scoreY-scoreYChange*3, Math.toRadians(180));
    private final Pose thirdScoreControl = new Pose(scoreControlX, scoreY - scoreYChange*3, Math.toRadians(180));
    private final Pose fourthScorePose = new Pose(scoreX, scoreY-scoreYChange*4, Math.toRadians(180));
    private final Pose fourthScoreControl = new Pose(scoreControlX, scoreY - scoreYChange*4, Math.toRadians(180));
    private final Pose fifthScorePose = new Pose(scoreX, scoreY-scoreYChange*5, Math.toRadians(180));
    private final Pose fifthScoreControl = new Pose(scoreControlX, scoreY - scoreYChange*5, Math.toRadians(180));
    private final Pose parkPose = new Pose(14, 30, Math.toRadians(359));

    // TODO: ZPAM VARIABLES
    private final double zeroPowerAccelerationMultiplierForPICKUP_WAIT = 2.6;
    private final double zeroPowerAccelerationMultiplerForScore = 4;

    private PathChain scorePreloadPath, parkFromFifthPath;
    private PathChain spike3, spike2, spike1;
    private PathChain firstPickupPath, secondPickupPath, thirdPickupPath, fourthPickupPath, fifthPickupPath;
    private PathChain firstScorePath, secondScorePath, thirdScorePath, fourthScorePath, fifthScorePath;
    private PathChain[] pickupPaths, scorePaths;
    public void buildPaths(){
        scorePreloadPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadScorePose)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .setZeroPowerAccelerationMultiplier(1.6)
                .setPathEndTValueConstraint(0.95)
                .build();
        spike3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preloadScorePose), new Point(thirdSpikeMark)))
                .setLinearHeadingInterpolation(preloadScorePose.getHeading(), thirdSpikeMark.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .setPathEndTValueConstraint(0.99)
                .build();
        spike2 = follower.pathBuilder()
                .addPath(new BezierLine(thirdSpikeMark, secondSpikeMark))
                .setConstantHeadingInterpolation(secondSpikeMark.getHeading())
                .setZeroPowerAccelerationMultiplier(5)
                .build();
        spike1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSpikeMark), new Point(firstSpikeMark)))
                .setLinearHeadingInterpolation(secondSpikeMark.getHeading(), firstSpikeMark.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();
        firstPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstSpikeMark), new Point(outtakeFirstPickupPose)))
                .setConstantHeadingInterpolation(firstSpikeMark.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();
        firstScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakeFirstPickupPose), new Point(firstScoreControl), new Point(firstScorePose)))
                .setConstantHeadingInterpolation(outtakeFirstPickupPose.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
        secondPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstScorePose), new Point(outtakePickupPose)))
                .setConstantHeadingInterpolation(outtakePickupPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPICKUP_WAIT)
                .setPathEndTimeoutConstraint(10)
                .setPathEndTValueConstraint(0.825)
                .build();
        secondScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupPose),new Point(secondScoreControl), new Point(secondScorePose)))
                .setConstantHeadingInterpolation(outtakePickupPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .build();
        thirdPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondScorePose), new Point(outtakePickupPose)))
                .setConstantHeadingInterpolation(secondScorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPICKUP_WAIT)
                .setPathEndTimeoutConstraint(10)
                .setPathEndTValueConstraint(0.825)
                .build();
        thirdScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupPose), new Point(thirdScoreControl),new Point(thirdScorePose)))
                .setConstantHeadingInterpolation(outtakePickupPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .build();
        fourthPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdScorePose), new Point(outtakePickupPose)))
                .setConstantHeadingInterpolation(thirdScorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPICKUP_WAIT)
                .setPathEndTimeoutConstraint(10)
                .setPathEndTValueConstraint(0.825)
                .build();
        fourthScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupPose), new Point(fourthScoreControl), new Point(fourthScorePose)))
                .setConstantHeadingInterpolation(outtakePickupPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .build();
        fifthPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fourthScorePose), new Point(outtakePickupPose)))
                .setConstantHeadingInterpolation(fourthScorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPICKUP_WAIT)
                .setPathEndTimeoutConstraint(10)
                .setPathEndTValueConstraint(0.825)
                .build();
        fifthScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupPose), new Point(fifthScoreControl), new Point(fifthScorePose)))
                .setConstantHeadingInterpolation(outtakePickupPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .build();
        parkFromFifthPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fifthScorePose), new Point(parkPose)))
                .setLinearHeadingInterpolation(fifthScorePose.getHeading(), parkPose.getHeading())
                .build();
        pickupPaths = new PathChain[]{firstPickupPath, secondPickupPath, thirdPickupPath, fourthPickupPath, fifthPickupPath};
        scorePaths = new PathChain[]{firstScorePath, secondScorePath, thirdScorePath, fourthScorePath, fifthScorePath};
    }

    enum AutoState {
        DRIVE_TO_PRELOAD_SCORE,
        PRELOAD_AT_SUB,
        SCORE_PRELOAD,
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
        READY_FOR_PICKUP,
        WALL_PICKUP,
        WALL_DELAY,
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

    private int counter = 0;
    public void autonomousPathUpdate(){
        switch (autoState){
            case DRIVE_TO_PRELOAD_SCORE:
                outtake.setClawState(Outtake.ClawStates.CLOSED);
                outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTSCORE);
                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_SCORE_WAIT);
                follower.followPath(scorePreloadPath,true);
                setPathState(AutoState.SCORE_PRELOAD);
                break;
            case SCORE_PRELOAD:
                if(!follower.isBusy()){
                    setPathState(AutoState.PRELOAD_AT_SUB);
                }
                break;
            case PRELOAD_AT_SUB:
                if(pathTimer.getElapsedTimeSeconds() > 0.1){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_SCORE_DONE);
                    if(pathTimer.getElapsedTimeSeconds() > 0.5){
                        outtake.setClawState(Outtake.ClawStates.OPEN);
                        if(pathTimer.getElapsedTimeSeconds() > 0.75){ // Time it takes for claw to open
                            follower.followPath(spike3, true);
                            setPathState(AutoState.DRIVE_TO_SPIKE_3);
                        }
                    }
                }
                break;
            case DRIVE_TO_SPIKE_3:
                if(follower.getCurrentTValue()  > 0.3){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.BACK_PICKUP_WAIT);
                }
                if(follower.getCurrentTValue() > 0.4){
                    intakeDiffyClaw.ExtendTo(380, Intake_DiffyClaw.ExtensionUnits.ticks);
                    intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = -30;
                }
                if(!follower.isBusy()){
                    setPathState(AutoState.AT_SPIKE_3);
                    // TODO: Vision
                }
                break;
            case AT_SPIKE_3:
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
                    if(pathTimer.getElapsedTimeSeconds() > 0.6){
                        intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.CLOSED);
                    }
                    if(pathTimer.getElapsedTimeSeconds() > 0.9){
                        intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.DEPOSIT);
                        intakeDiffyClaw.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                        follower.followPath(spike2, true);
                        setPathState(AutoState.TO_SPIKE_2);
                    }
                }
                break;
            case TO_SPIKE_2:
                double k = follower.getPose().getHeading();
                if(k > Math.PI) k = 2*Math.PI - k;
                if(Math.abs(k) < Math.toRadians(5) && intakeDiffyClaw.getCurrentPosition() < 10){
                    setPathState(AutoState.AT_SPIKE_2);
                }
                break;
            case AT_SPIKE_2:
                intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                if(pathTimer.getElapsedTimeSeconds() > 0.5){
                    intakeDiffyClaw.ExtendTo(380, Intake_DiffyClaw.ExtensionUnits.ticks);
                    intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
                    if(intakeDiffyClaw.extensionReachedTarget()){
                        setPathState(AutoState.PICKUP_SPIKE_2);
                    }
                }
                break;
            case PICKUP_SPIKE_2:
                intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.CLOSED);
                    if(pathTimer.getElapsedTimeSeconds() > 0.5){
                        follower.followPath(spike1, true);
                        setPathState(AutoState.TO_SPIKE_1);
                        intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.DEPOSIT);
                        intakeDiffyClaw.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                        if(pathTimer.getElapsedTimeSeconds() > 0.6){
                            intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.LOOSE);
                        }
                        if(intakeDiffyClaw.getCurrentPosition() < 10){
                            intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                        }
                    }
                }
                break;
            case TO_SPIKE_1:
                if(!follower.isBusy() && intakeDiffyClaw.getCurrentPosition() < 10){
                    setPathState(AutoState.AT_SPIKE_1);
                }
                break;
            case AT_SPIKE_1:
                intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                if(pathTimer.getElapsedTimeSeconds() > 0.5){
                    intakeDiffyClaw.ExtendTo(380, Intake_DiffyClaw.ExtensionUnits.ticks);
                    intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
                    if(intakeDiffyClaw.extensionReachedTarget()){
                        setPathState(AutoState.PICKUP_SPIKE_1);
                    }
                }
                break;
            case PICKUP_SPIKE_1:
                intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.CLOSED);
                    if(pathTimer.getElapsedTimeSeconds() > 0.5){
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
                if(intakeDiffyClaw.getCurrentPosition() < 50){
                    intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.LOOSE);
                }
                if(intakeDiffyClaw.getCurrentPosition() < 10){
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
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_SCORE_WAIT);
                    outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTSCORE);
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
                if(pathTimer.getElapsedTimeSeconds() > 0.4){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_SCORE_DONE);
                    if(pathTimer.getElapsedTimeSeconds() > 1){
                        outtake.setClawState(Outtake.ClawStates.OPEN);
                        if(pathTimer.getElapsedTimeSeconds() > 1.25){
                            setPathState(AutoState.READY_FOR_PICKUP);
                            outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
                            outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
                            follower.followPath(pickupPaths[++counter]);
                        }
                    }
                }
                break;
            case READY_FOR_PICKUP:
                outtake.setClawState(Outtake.ClawStates.OPEN);
                if(pathTimer.getElapsedTimeSeconds() > 0.2){
                    outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
                }
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
                if (pathTimer.getElapsedTimeSeconds() > 0.6){
                    outtake.setOuttakeState(Outtake.OuttakeState.RESET_ENCODER);
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.RESET_ENCODER);
                }
                break;
            default:
                break;
        }
    }
    @Override
    public void init(){

        pathTimer = new Timer();
        follower = new Follower(hardwareMap, FConstants_5_0.class, LConstants.class);
        follower.setStartingPose(startPose);

        tel = new UnifiedTelemetry();
        tel.init(this.telemetry);
        SubsysCore.setGlobalParameters(hardwareMap, this);


        lynxModules = new LynxModules();
        lynxModules.init();

        intakeDiffyClaw = new Intake_DiffyClaw();
        intakeDiffyClaw.init();
        outtake = new Outtake();
        outtake.init();
        outtakeLift = new OuttakeLiftSubsys();
        outtakeLift.init();
        buildPaths();
    }

    private final ToggleButton teamColorButton = new ToggleButton(Storage.isRed);
    private UnifiedTelemetry tel;
    @Override
    public void init_loop(){
        lynxModules.loop();
        teamColorButton.input(gamepad1.dpad_up);
        Storage.isRed = teamColorButton.getVal();
        outtake.setOuttakeState(Outtake.OuttakeState.Auto_Wait);
        outtake.setClawState(Outtake.ClawStates.CLOSED);
        outtake.loop();
        tel.addData("Team Color:", Storage.isRed ? "Red" : "Blue");
        tel.update();
    }
    @Override
    public void loop(){
        lynxModules.loop();

        follower.update();
        autonomousPathUpdate();
        outtake.loop();
        outtakeLift.holdLift();
        outtakeLift.loop();
        intakeDiffyClaw.loop();
        intakeDiffyClaw.HoldExtension();


        Storage.CurrentPose = follower.getPose();
        tel.addData("AutoState", autoState.name());
        tel.addData("TValue", follower.getCurrentTValue());
        tel.addData("VeloX", follower.getVelocity().getMagnitude());
        tel.addData("XPOS", follower.getPose().getX());
        tel.addData("YPOS", follower.getPose().getY());
        tel.addData("translational error", follower.getTranslationalError().getMagnitude());
        tel.update();
    }
}