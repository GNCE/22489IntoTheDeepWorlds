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
public class Auto_6_0_Pushing extends OpMode {
    private Follower follower;
    private Intake_DiffyClaw intakeDiffyClaw;
    private OuttakeLiftSubsys outtakeLift;
    private Outtake outtake;
    private Timer pathTimer;
    private LynxModules lynxModules;
    private final double scoreX = 37;
    private final double scoreY = 73;

    private final double frontScoreX = 36;

    private final Pose startPose = new Pose(8.55, 63.5, Math.toRadians(0));
    private final Pose preloadScorePose = new Pose(frontScoreX, scoreY, Math.toRadians(0));
    private final Pose thirdSpikeMark = new Pose(21.170860927152315, 12.0158940397351, Math.toRadians(-27));
    private final Pose secondSpikeMark = new Pose(21.170860927152315, 12.0158940397351, Math.toRadians(0));
    private final Pose firstSpikeMark = new Pose(21.170860927152315, 23.45960264900662, Math.toRadians(0));


    private final Pose[][] samplePushPoses = {
            {
                    preloadScorePose,
                    new Pose(22.50596026490066, 23, Math.toRadians(180)),
                    new Pose(56.45562913907285, 38.71788079470198, Math.toRadians(180)),
                    new Pose(57.40927152317881, 25.36688741721855, Math.toRadians(180))
            },
            {
                    new Pose(56.83708609271523, 21.45960264900662, Math.toRadians(180)),
                    new Pose(18.69139072847682, 24.22251655629138, Math.toRadians(180))
            },
            {
                    new Pose(18.69139072847682, 24.22251655629138, Math.toRadians(180)),
                    new Pose(51.68741721854305, 25.17615894039735, Math.toRadians(180)),
                    new Pose(60.46092715231788, 14.495364238410597, Math.toRadians(180)),
            },
            {
                    new Pose(60.46092715231788, 14.495364238410597, Math.toRadians(180)),
                    new Pose(18.69139072847682, 14.495364238410597, Math.toRadians(180)),
            },
            {
                    new Pose(18.69139072847682, 14.495364238410597, Math.toRadians(180)),
                    new Pose(49.78013245033112, 14.87682119205298, Math.toRadians(180)),
                    new Pose(60.46092715231788, 7.5, Math.toRadians(180)),
            },
            {
                    new Pose(60.46092715231788, 7.5, Math.toRadians(180)),
                    new Pose(9.5, 7.5, Math.toRadians(180))
            }
    };


    private final double scoreControlX = 23;
    private final double scoreYChange = 1.15;
    private final Pose outtakeFirstPickupPose = new Pose(9.5, 7.5, Math.toRadians(180));
    private final Pose outtakePickupWaitPose = new Pose(15, 33.5, Math.toRadians(180));
    private final Pose outtakePickupPose = new Pose(9.1, 33.5, Math.toRadians(180));

    private final Pose firstScorePose = new Pose(scoreX, scoreY-scoreYChange, Math.toRadians(180));
    private final Pose firstScoreControl = new Pose(scoreControlX, scoreY-scoreYChange, Math.toRadians(180));
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
                .setZeroPowerAccelerationMultiplier(3)
                .build();
        spike3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preloadScorePose), new Point(thirdSpikeMark)))
                .setLinearHeadingInterpolation(preloadScorePose.getHeading(), thirdSpikeMark.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();
        spike2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdSpikeMark), new Point(secondSpikeMark)))
                .setLinearHeadingInterpolation(thirdSpikeMark.getHeading(), secondSpikeMark.getHeading())
                .setZeroPowerAccelerationMultiplier(5)
                .build();
        spike1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSpikeMark), new Point(firstSpikeMark)))
                .setLinearHeadingInterpolation(secondSpikeMark.getHeading(), firstSpikeMark.getHeading())
                .setZeroPowerAccelerationMultiplier(5)
                .build();
        firstPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstSpikeMark), new Point(outtakeFirstPickupPose)))
                .setConstantHeadingInterpolation(firstSpikeMark.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();
        firstScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakeFirstPickupPose), new Point(firstScoreControl), new Point(firstScorePose)))
                .setConstantHeadingInterpolation(outtakePickupWaitPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .build();
        secondPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstScorePose), new Point(outtakePickupPose)))
                .setConstantHeadingInterpolation(firstScorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPICKUP_WAIT)
                .setPathEndTimeoutConstraint(10)
                .setPathEndTValueConstraint(0.825)
                .build();
        secondScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupPose),new Point(secondScoreControl), new Point(secondScorePose)))
                .setConstantHeadingInterpolation(outtakePickupWaitPose.getHeading())
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
                .setConstantHeadingInterpolation(outtakePickupWaitPose.getHeading())
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
                .setConstantHeadingInterpolation(outtakePickupWaitPose.getHeading())
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
                .setConstantHeadingInterpolation(outtakePickupWaitPose.getHeading())
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

        BEFORE_PUSHING,
        READY_FOR_PUSHING,
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
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_SCORE_DONE);
                    if(!outtakeLift.isBusy()){
                        outtake.setClawState(Outtake.ClawStates.OPEN);
                        setPathState(AutoState.PRELOAD_AT_SUB);
                    }
                }
                break;
            case PRELOAD_AT_SUB:
                if(pathTimer.getElapsedTimeSeconds() > 0.23){ // Time it takes for claw to open
                    // TODO: do vision things
                    follower.followPath(spike3, true);
                    setPathState(AutoState.DRIVE_TO_SPIKE_3);
                }
                break;
            case DRIVE_TO_SPIKE_3:
                if(follower.getCurrentTValue() > 0.4){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.BACK_PICKUP_WAIT);
                }
                if(follower.getCurrentTValue() > 0.5){
                    intakeDiffyClaw.ExtendTo(250, Intake_DiffyClaw.ExtensionUnits.ticks);
                    intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                }
                if(!follower.isBusy()){
                    setPathState(AutoState.AT_SPIKE_3);
                    // TODO: Vision
                    intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);

                }
                break;
            case AT_SPIKE_3:
                intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.CLOSED);
                }
                if(pathTimer.getElapsedTimeSeconds() > 0.5){
                    intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.DEPOSIT);
                    intakeDiffyClaw.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                    follower.followPath(spike2, true);
                    setPathState(AutoState.TO_SPIKE_2);
                }
                break;
            case TO_SPIKE_2:
                if(!follower.isBusy() && intakeDiffyClaw.getCurrentPosition() < 10){
                    setPathState(AutoState.AT_SPIKE_2);
                }
                break;
            case AT_SPIKE_2:
                intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    intakeDiffyClaw.ExtendTo(300, Intake_DiffyClaw.ExtensionUnits.ticks);
                    intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
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
                        intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.DEPOSIT);
                        intakeDiffyClaw.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                        if(intakeDiffyClaw.getCurrentPosition() < 10){
                            intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                        }
                    }
                }
                break;
            case READY_FOR_PICKUP:
                outtake.setClawState(Outtake.ClawStates.OPEN);
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
                        if (counter < 4) {
                            outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCOREOUT);
                            follower.followPath(pickupPaths[counter], true);
                            setPathState(AutoState.WALL_PICKUP);
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