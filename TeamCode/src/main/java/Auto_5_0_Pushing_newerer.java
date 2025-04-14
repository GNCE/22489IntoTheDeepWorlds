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
import pedroPathing.constants.LConstants;
import subsystems.OuttakeLiftSubsys;
import subsystems.SubsysCore;
import subsystems.UnifiedTelemetry;


@Autonomous (name = "Auto_5_0_pushing_newerer")
public class Auto_5_0_Pushing_newerer extends OpMode {
    private Follower follower;
    private Intake_DiffyClaw intakeDiffyClaw;
    private OuttakeLiftSubsys outtakeLift;
    private Outtake outtake;
    private Timer pathTimer;
    private final double scoreX = 36.15;
    private final double scoreY = 77;

    private final Pose startPose = new Pose(6.55, 65.5, Math.toRadians(180));
    private final Pose preloadScoreControl = new Pose(20, scoreY, Math.toRadians(180));
    private final Pose preloadScorePose = new Pose(scoreX, scoreY, Math.toRadians(180));

    private final Pose[][] samplePushPoses = {
            {
                    preloadScorePose,
                    new Pose(27.66, 38.5, Math.toRadians(180)),
                    new Pose(25.4, 19.8, Math.toRadians(180)),
                    new Pose(58.94, 54.17, Math.toRadians(180)),
                    new Pose(65.80132450331126, 29.75364238410596, Math.toRadians(180)),
                    new Pose(68.09006622516556, 15.067549668874179, Math.toRadians(180)),
                    new Pose(51.30596026490066, 24.985430463576165, Math.toRadians(180)),
                    new Pose(18, 23.5, Math.toRadians(180)),
            },
            {
                    new Pose(18, 23.5, Math.toRadians(180)),
                    new Pose(69.61589403973511, 26.939072847682112, Math.toRadians(180)),
                    new Pose(65.0384105960265, 17.784105960264895, Math.toRadians(180)),
                    new Pose(60.46092715231788, 10.536423841059603, Math.toRadians(180)),
                    new Pose(18, 16.0158940397351, Math.toRadians(180)),
            },
            {
                    new Pose(18, 16.0158940397351, Math.toRadians(180)),
                    new Pose(66.37350993377483, 16.784105960264895, Math.toRadians(180)),
                    new Pose(67.3271523178808, 13.160264900662249, Math.toRadians(180)),
                    new Pose(66.37350993377483, 5.531125827814575, Math.toRadians(180)),
                    new Pose(32.805298013245036, 10.299337748344369, Math.toRadians(180)),
                    new Pose(18, 9, Math.toRadians(180)),
            }
    };


    private final double scoreControlX = 23;
    private final double scoreYChange = 1.1;
    private final Pose outtakeFirstPickupPose = new Pose(10.1, 9.5, Math.toRadians(180));
    private final Pose outtakePickupWaitPose = new Pose(14.5, 32, Math.toRadians(180));
    private final Pose outtakePickupPose = new Pose(12.1, 32, Math.toRadians(180));

    private final Pose firstOuttakePickupControl1 = new Pose(scoreX-10, scoreY - scoreYChange, Math.toRadians(180));
    private final Pose firstOuttakePickupControl2 = new Pose(scoreX-15, scoreY - scoreYChange, Math.toRadians(180));

    private final Pose secondOuttakePickupControl1 = new Pose(scoreX-10, scoreY - scoreYChange*2, Math.toRadians(180));
    private final Pose secondOuttakePickupControl2 = new Pose(scoreX-15, scoreY - scoreYChange*2, Math.toRadians(180));

    private final Pose thirdOuttakePickupControl1 = new Pose(scoreX-10, scoreY - scoreYChange*3, Math.toRadians(180));
    private final Pose thirdOuttakePickupControl2 = new Pose(scoreX-15, scoreY - scoreYChange*3, Math.toRadians(180));

    private final Pose fourthOuttakePickupControl1 = new Pose(scoreX-10, scoreY - scoreYChange*4, Math.toRadians(180));
    private final Pose fourthOuttakePickupControl2 = new Pose(scoreX-15, scoreY - scoreYChange*4, Math.toRadians(180));


    private final Pose outtakePickupControl3 = new Pose(30, 32, Math.toRadians(180));
    private final Pose outtakePickupControl4 = new Pose(20, 32, Math.toRadians(180));
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
    private final Pose parkPose = new Pose(18, 30, Math.toRadians(230));
    private final double pushPathEndTimeout = 300;
    private final double pickupWaitTimeout = 100;

    // TODO: ZPAM VARIABLES
    private final double zeroPowerAccelerationMultiplierForPICKUP_WAIT = 3;
    private final double zeroPowerAccelerationMultiplierForPickupWall= 0.4;
    private final double zeroPowerAccelerationMultiplierForPickupFirst = 0.5;
    private final double zeroPowerAccelerationMultiplierForPush = 2;
    private final double zeroPowerAccelerationMultiplerForScore = 3.5;

    private PathChain scorePreloadPath, parkFromFifthPath;
    private PathChain pushSample1, pushSample2, pushSample3;
    private PathChain firstPickupPath, secondPickupPath, thirdPickupPath, fourthPickupPath, fifthPickupPath;
    private PathChain firstScorePath, secondScorePath, thirdScorePath, fourthScorePath, fifthScorePath;
    private PathChain wallPickup;
    private PathChain[] pushWaitPaths, pickupPaths, scorePaths;
    public void buildPaths(){
        scorePreloadPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(preloadScoreControl), new Point(preloadScorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadScorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(1.3)
                .build();
        pushSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(samplePushPoses[0]))
                .setLinearHeadingInterpolation(samplePushPoses[0][0].getHeading(), samplePushPoses[0][samplePushPoses[0].length-1].getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPush)
                .setPathEndTimeoutConstraint(pushPathEndTimeout)
                .build();
        pushSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(samplePushPoses[1]))
                .setLinearHeadingInterpolation(samplePushPoses[1][0].getHeading(), samplePushPoses[1][samplePushPoses[1].length-1].getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPush)
                .setPathEndTimeoutConstraint(pushPathEndTimeout)
                .build();
        pushSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(samplePushPoses[2]))
                .setLinearHeadingInterpolation(samplePushPoses[2][0].getHeading(), samplePushPoses[2][samplePushPoses[2].length-1].getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPush)
                .setPathEndTimeoutConstraint(pushPathEndTimeout)
                .build();
        firstPickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(samplePushPoses[2][samplePushPoses[2].length-1]), new Point(outtakeFirstPickupPose)))
                .setLinearHeadingInterpolation(samplePushPoses[2][samplePushPoses[2].length-1].getHeading(), outtakeFirstPickupPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPickupFirst)
                .setPathEndTimeoutConstraint(25)
                .build();
        wallPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(outtakePickupWaitPose),new Point(outtakePickupPose)))
                .setLinearHeadingInterpolation(outtakePickupWaitPose.getHeading(),outtakePickupPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPickupWall)
                .setPathEndTimeoutConstraint(25)
                .build();
        firstScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupWaitPose), new Point(firstScoreControl), new Point(firstScorePose)))
                .setLinearHeadingInterpolation(outtakePickupWaitPose.getHeading(),firstScorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .build();
        secondPickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(firstScorePose), new Point(firstOuttakePickupControl1), new Point(firstOuttakePickupControl2), new Point(outtakePickupControl3), new Point(outtakePickupControl4), new Point(outtakePickupWaitPose)))
                .setLinearHeadingInterpolation(firstScorePose.getHeading(), outtakePickupWaitPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPICKUP_WAIT)
                .setPathEndTimeoutConstraint(pickupWaitTimeout)
                .build();
        secondScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupWaitPose),new Point(secondScoreControl), new Point(secondScorePose)))
                .setLinearHeadingInterpolation(outtakePickupWaitPose.getHeading(), secondScorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .build();
        thirdPickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(secondScorePose), new Point(secondOuttakePickupControl1), new Point(secondOuttakePickupControl2), new Point(outtakePickupControl3), new Point(outtakePickupControl4), new Point(outtakePickupWaitPose)))
                .setLinearHeadingInterpolation(secondScorePose.getHeading(), outtakePickupWaitPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPICKUP_WAIT)
                .setPathEndTimeoutConstraint(pickupWaitTimeout)
                .build();
        thirdScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupWaitPose), new Point(thirdScoreControl),new Point(thirdScorePose)))
                .setLinearHeadingInterpolation(outtakePickupWaitPose.getHeading(), thirdScorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .build();
        fourthPickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(thirdScorePose), new Point(thirdOuttakePickupControl1), new Point(thirdOuttakePickupControl2),new Point(outtakePickupControl3), new Point(outtakePickupControl4), new Point(outtakePickupWaitPose)))
                .setLinearHeadingInterpolation(thirdScorePose.getHeading(), outtakePickupWaitPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPICKUP_WAIT)
                .setPathEndTimeoutConstraint(pickupWaitTimeout)
                .build();
        fourthScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupWaitPose), new Point(fourthScoreControl), new Point(fourthScorePose)))
                .setLinearHeadingInterpolation(outtakePickupWaitPose.getHeading(), fourthScorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .build();
        fifthPickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(fourthScorePose), new Point(fourthOuttakePickupControl1), new Point(fourthOuttakePickupControl2),new Point(outtakePickupControl3), new Point(outtakePickupControl4), new Point(outtakePickupWaitPose)))
                .setLinearHeadingInterpolation(fourthScorePose.getHeading(), outtakePickupWaitPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPICKUP_WAIT)
                .setPathEndTimeoutConstraint(pickupWaitTimeout)
                .build();
        fifthScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupWaitPose), new Point(fifthScoreControl), new Point(fifthScorePose)))
                .setLinearHeadingInterpolation(outtakePickupWaitPose.getHeading(), fifthScorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .build();
        parkFromFifthPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fifthScorePose), new Point(parkPose)))
                .setLinearHeadingInterpolation(fifthScorePose.getHeading(), parkPose.getHeading())
                .build();

        pushWaitPaths = new PathChain[]{pushSample1, pushSample2, pushSample3};
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
                outtake.setClawOpen(false);
                outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCORE);
                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.BACK_SCORE);
                if(pathTimer.getElapsedTimeSeconds() > 0){
                    follower.followPath(scorePreloadPath,true);
                    setPathState(AutoState.SCORE_PRELOAD);
                }
                break;
            case SCORE_PRELOAD:
                if(!follower.isBusy()){
                    outtake.setClawOpen(true);
                    outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCOREOUT);
                    setPathState(AutoState.DRIVE_TO_PUSHING);
                }
                break;
            case DRIVE_TO_PUSHING:
                if(pathTimer.getElapsedTimeSeconds() > 0){
                    follower.followPath(pushWaitPaths[counter], true);
                    counter = 0;
                    setPathState(AutoState.BEFORE_PUSHING);
                }
                break;
            case BEFORE_PUSHING:
                if(pathTimer.getElapsedTimeSeconds() > 0.4){
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
                    outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
                    setPathState(AutoState.READY_FOR_PUSHING);
                }
                break;
            case READY_FOR_PUSHING:
                if(!follower.isBusy()){
                    counter++;
                    if(counter < 3){
                        follower.followPath(pushWaitPaths[counter], false);
                        setPathState(AutoState.READY_FOR_PUSHING);
                    } else {
                        counter = 0;
                        setPathState(AutoState.READY_FOR_PICKUP); // Skips WALL_PICKUP when first pickup
                        outtake.setClawOpen(true);
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
                        follower.followPath(pickupPaths[counter], false);
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
            case WALL_DELAY:
                if (pathTimer.getElapsedTimeSeconds() > 0.0) {
                    follower.followPath(wallPickup, 0.5, true);
                    setPathState(AutoState.READY_FOR_PICKUP);
                }
                break;
            case READY_FOR_PICKUP:
                outtake.setClawOpen(true);
                if(!follower.isBusy()){
                    setPathState(AutoState.PICKUP);
                }
                break;
            case PICKUP:
                if(pathTimer.getElapsedTimeSeconds() > 0){
                    outtake.setClawOpen(false);
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
                    outtake.setClawOpen(true);
                    if (pathTimer.getElapsedTimeSeconds() > 0.26) {
                        counter++;
                        if (counter < 4) {
                            follower.followPath(pickupPaths[counter], false);
                            setPathState(AutoState.WALL_PICKUP);
                        } else {
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
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        tel = new UnifiedTelemetry();
        tel.init(this.telemetry);
        SubsysCore.setGlobalParameters(hardwareMap, this);

        intakeDiffyClaw = new Intake_DiffyClaw();
        intakeDiffyClaw.init();
        outtake = new Outtake(hardwareMap);
        outtakeLift = new OuttakeLiftSubsys();
        outtakeLift.init();
        buildPaths();
    }

    private final ToggleButton teamColorButton = new ToggleButton(Storage.isRed);
    private UnifiedTelemetry tel;
    @Override
    public void init_loop(){
        teamColorButton.input(gamepad1.dpad_up);
        Storage.isRed = teamColorButton.getVal();
        outtake.setOuttakeState(Outtake.OuttakeState.Auto_Wait);
        outtake.outtakeLoop();
        outtake.setClawOpen(false);
        tel.addData("Team Color:", Storage.isRed ? "Red" : "Blue");
        tel.update();
    }
    @Override
    public void loop(){
        follower.update();
        autonomousPathUpdate();
        outtake.outtakeLoop();
        outtakeLift.holdLift();
        outtakeLift.loop();
        intakeDiffyClaw.loop();


        Storage.CurrentPose = follower.getPose();
        telemetry.update();
    }
}