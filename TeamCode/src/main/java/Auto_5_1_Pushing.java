import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.FConstants_5_0;
import pedroPathing.constants.LConstants;
import subsystems.Intake_DiffyClaw;
import subsystems.LynxModules;
import subsystems.Outtake;
import subsystems.OuttakeLiftSubsys;
import subsystems.SubsysCore;
import subsystems.UnifiedTelemetry;


@Autonomous (name = "5 + 1 Autonomous - Pushing")
public class Auto_5_1_Pushing extends OpMode {
    private Follower follower;
    private Intake_DiffyClaw intakeDiffyClaw;
    private OuttakeLiftSubsys outtakeLift;
    private Outtake outtake;
    private Timer pathTimer;
    private LynxModules lynxModules;
    private final double scoreX = 37;
    private final double scoreY = 71.5;

    private final Pose startPose = new Pose(6.55, 63.5, Math.toRadians(180));
    private final Pose preloadScoreControl = new Pose(20, scoreY, Math.toRadians(180));
    private final Pose preloadScorePose = new Pose(scoreX, scoreY, Math.toRadians(180));

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
                    new Pose(60.46092715231788, 16.495364238410597, Math.toRadians(180)),
            },
            {
                    new Pose(60.46092715231788, 16.495364238410597, Math.toRadians(180)),
                    new Pose(18.69139072847682, 14.495364238410597, Math.toRadians(180)),
            },
            {
                    new Pose(18.69139072847682, 14.495364238410597, Math.toRadians(180)),
                    new Pose(49.78013245033112, 14.87682119205298, Math.toRadians(180)),
                    new Pose(60.46092715231788, 7.5, Math.toRadians(180)),
            },
            {
                    new Pose(60.46092715231788, 7.5, Math.toRadians(180)),
                    new Pose(9.1, 7.5, Math.toRadians(180))
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
    private final Pose basketScorePose = new Pose(7.25, 127, Math.toRadians(270));
    private final Pose basketMIDPose = new Pose(12, 40, Math.toRadians(270));
    private final Pose parkPose = new Pose(10, 30, Math.toRadians(270));
    private final double pushPathEndTimeout = 5;
    private final double pickupWaitTimeout = 50;

    // TODO: ZPAM VARIABLES
    private final double zeroPowerAccelerationMultiplierForPICKUP_WAIT = 2.6;
    private final double zeroPowerAccelerationMultiplierForPickupWall= 2;
    private final double zeroPowerAccelerationMultiplierForPickupFirst = 3.5;
    private final double zeroPowerAccelerationMultiplierForPush = 3.3;
    private final double zeroPowerAccelerationMultiplierForPushWait = 2.3;
    private final double zeroPowerAccelerationMultiplerForScore = 4;

    private PathChain scorePreloadPath, parkFromFifthPath;
    private PathChain goToPush1, goToPush2, goToPush3;
    private PathChain pushSample1, pushSample2, pushSample3, pushSample;
    private PathChain firstPickupPath, secondPickupPath, thirdPickupPath, fourthPickupPath, fifthPickupPath;
    private PathChain firstScorePath, secondScorePath, thirdScorePath, fourthScorePath, fifthScorePath;
    private PathChain basketPickup, basketScore;
    private PathChain wallPickup;
    private PathChain[] pickupPaths, scorePaths, basketPaths;
    private PathChain[] pushPaths;
    public void buildPaths(){
        scorePreloadPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(preloadScoreControl), new Point(preloadScorePose)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .setZeroPowerAccelerationMultiplier(2.5)
                .build();
        goToPush1 = follower.pathBuilder()
                .addPath(new BezierCurve(samplePushPoses[0]))
                .setConstantHeadingInterpolation(samplePushPoses[0][0].getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPushWait)
                .setPathEndTimeoutConstraint(pushPathEndTimeout)
                .setPathEndTValueConstraint(0.995)
                .build();
        pushSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(samplePushPoses[1]))
                .setConstantHeadingInterpolation(samplePushPoses[1][0].getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPush)
                .setPathEndTimeoutConstraint(pushPathEndTimeout)
                .setPathEndTValueConstraint(0.9)
                .build();
        goToPush2 = follower.pathBuilder()
                .addPath(new BezierCurve(samplePushPoses[2]))
                .setConstantHeadingInterpolation(samplePushPoses[2][0].getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPushWait)
                .setPathEndTimeoutConstraint(pushPathEndTimeout)
                .setPathEndTValueConstraint(0.995)
                .build();
        pushSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(samplePushPoses[3]))
                .setConstantHeadingInterpolation(samplePushPoses[3][0].getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPush)
                .setPathEndTimeoutConstraint(pushPathEndTimeout)
                .setPathEndTValueConstraint(0.9)
                .build();
        goToPush3 = follower.pathBuilder()
                .addPath(new BezierCurve(samplePushPoses[4]))
                .setConstantHeadingInterpolation(samplePushPoses[4][0].getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPushWait)
                .setPathEndTimeoutConstraint(pushPathEndTimeout)
                .setPathEndTValueConstraint(0.995)
                .build();
        pushSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(samplePushPoses[5]))
                .setConstantHeadingInterpolation(samplePushPoses[5][0].getHeading())
                .setZeroPowerAccelerationMultiplier(2.5)
                .setPathEndTimeoutConstraint(10)
                .setPathEndTValueConstraint(0.93)
                .build();
        firstPickupPath = follower.pathBuilder() //Unused, combined into 3rd push
                .addPath(new BezierCurve(new Point(samplePushPoses[5][samplePushPoses[5].length-1]), new Point(outtakeFirstPickupPose)))
                .setConstantHeadingInterpolation(samplePushPoses[5][samplePushPoses[5].length-1].getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPickupFirst)
                .setPathEndTimeoutConstraint(25)
                .build();
        wallPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(outtakePickupWaitPose),new Point(outtakePickupPose)))
                .setConstantHeadingInterpolation(outtakePickupWaitPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPickupWall)
                .setPathEndTimeoutConstraint(25)
                .setPathEndTValueConstraint(0.98)
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
        basketPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fifthScorePose), new Point(outtakePickupPose)))
                .setConstantHeadingInterpolation(outtakePickupWaitPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPICKUP_WAIT)
                .setPathEndTimeoutConstraint(10)
                .setPathEndTValueConstraint(0.825)
                .build();
        basketScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(outtakePickupPose), new Point(basketMIDPose)))
                .setLinearHeadingInterpolation(outtakePickupPose.getHeading(),basketScorePose.getHeading())
                .addPath(new BezierLine(new Point(basketMIDPose), new Point(basketScorePose)))
                .setConstantHeadingInterpolation(basketScorePose.getHeading())
                .setPathEndTimeoutConstraint(50)
                .setPathEndTValueConstraint(0.99)
                .setZeroPowerAccelerationMultiplier(2)
                .build();
        parkFromFifthPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(basketScorePose), new Point(parkPose)))
                .setLinearHeadingInterpolation(basketScorePose.getHeading(), parkPose.getHeading())
                .setZeroPowerAccelerationMultiplier(5.5)
                .build();
        pushPaths = new PathChain[]{goToPush1, pushSample1, goToPush2, pushSample2, goToPush3, pushSample3};
        pickupPaths = new PathChain[]{firstPickupPath, secondPickupPath, thirdPickupPath, fourthPickupPath, fifthPickupPath};
        scorePaths = new PathChain[]{firstScorePath, secondScorePath, thirdScorePath, fourthScorePath, fifthScorePath};
    }

    enum AutoState {
        DRIVE_TO_PRELOAD_SCORE,
        SCORE_PRELOAD,
        DRIVE_TO_PUSHING,
        BEFORE_PUSHING,
        READY_FOR_PUSHING,
        READY_FOR_PICKUP,
        WALL_PICKUP,
        WALL_DELAY,
        PICKUP,
        READY_TO_SCORE,
        SCORE,
        BASKET_PICKUP,
        BASKET_DELAY,
        REAL_BASKET_PICKUP,
        BASKET_READY,
        BASKET_SCORE,
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
                outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCORE);
                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.BACK_SCORE);
                if(pathTimer.getElapsedTimeSeconds() > 0){
                    follower.followPath(scorePreloadPath,true);
                    setPathState(AutoState.SCORE_PRELOAD);
                }
                break;
            case SCORE_PRELOAD:
                if(!follower.isBusy()){
                    outtake.setClawState(Outtake.ClawStates.OPEN);
                    outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCOREOUT);
                    setPathState(AutoState.DRIVE_TO_PUSHING);
                }
                break;
            case DRIVE_TO_PUSHING:
                if(pathTimer.getElapsedTimeSeconds() > 0){
                    counter = 0;
                    follower.followPath(pushPaths[0], true);
                    setPathState(AutoState.BEFORE_PUSHING);
                }
                break;
            case BEFORE_PUSHING:
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
                    outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
                    setPathState(AutoState.READY_FOR_PUSHING);
                }
                break;
            case READY_FOR_PUSHING:
                if(!follower.isBusy()){
                    counter++;
                    if(counter >= pushPaths.length){
                        setPathState(AutoState.READY_FOR_PICKUP);
                        counter=0;
                    } else {
                        setPathState(AutoState.READY_FOR_PUSHING);
                        outtake.setClawState(Outtake.ClawStates.OPEN);
                        follower.followPath(pushPaths[counter], true);
                    }
                }
                break;
            case WALL_PICKUP:
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
                    outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
                }
                if (!follower.isBusy()){
                    setPathState(AutoState.WALL_DELAY);
                }
                break;
            case WALL_DELAY: //no more seperate wall pickup!
                setPathState(AutoState.READY_FOR_PICKUP);
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
                            follower.followPath(basketPickup, false);
                            setPathState(AutoState.BASKET_PICKUP);
                        }
                    }
                }
                break;
            case BASKET_PICKUP:
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
                    outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
                }
                if (!follower.isBusy()){
                    setPathState(AutoState.BASKET_DELAY);
                }
                break;
            case BASKET_DELAY:
                outtake.setClawState(Outtake.ClawStates.OPEN);
                if(!follower.isBusy()){
                    setPathState(AutoState.REAL_BASKET_PICKUP);
                }
                break;
            case REAL_BASKET_PICKUP:
                if(pathTimer.getElapsedTimeSeconds() > 0){
                    outtake.setClawState(Outtake.ClawStates.CLOSED);
                    if(pathTimer.getElapsedTimeSeconds() > 0.32){
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.LIFT_BUCKET);
                        outtake.setOuttakeState(Outtake.OuttakeState.SAMPLESCORE);
                        follower.followPath(basketScore, true);
                        setPathState(AutoState.BASKET_READY);
                    }
                }
                break;
            case BASKET_READY:
                if(!follower.isBusy()){
                    setPathState(AutoState.BASKET_SCORE);
                }
                break;
            case BASKET_SCORE:
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    outtake.setClawState(Outtake.ClawStates.OPEN);
                    if (pathTimer.getElapsedTimeSeconds() > 1.1) {
                        {
                            outtake.setOuttakeState(Outtake.OuttakeState.SAMPLE_SCORE_WAIT);
                            follower.followPath(parkFromFifthPath, false);
                            setPathState(AutoState.PARK);
                        }
                    }
                }
                break;
            case PARK:
                if (pathTimer.getElapsedTimeSeconds() > 0.4){
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