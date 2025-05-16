import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import subsystems.Intake_DiffyClaw;
import subsystems.OuttakeLiftSubsys;
import subsystems.SubsysCore;
import subsystems.UnifiedTelemetry;


@Disabled
@Autonomous (name = "4+0 Pushing Specimen Auto")
public class Auto_4_0_Pushing extends OpMode {
    private Follower follower;
    private Intake_DiffyClaw intakeDiffyClaw;
    private OuttakeLiftSubsys outtakeLift;
    private Outtake outtake;
    private Timer pathTimer;
    private final double scoreX = 36;
    private final double scoreY = 78;

    private final Pose startPose = new Pose(7.1, 65.5, Math.toRadians(180));
    private final Pose preloadScoreControl = new Pose(20, scoreY, Math.toRadians(180));
    private final Pose preloadScorePose = new Pose(scoreX, scoreY, Math.toRadians(180));

    private final double startPushingSampleX = 52.5;
    private final Pose firstSamplePos = new Pose(startPushingSampleX, 25, Math.toRadians(180));
    private final Pose firstSampleControl1 = new Pose(0, 60, Math.toRadians(180));
    private final Pose firstSampleControl2 = new Pose(29, 8, Math.toRadians(180));
    private final Pose firstSampleControl3 = new Pose(66, 56, Math.toRadians(180));
    private final Pose firstSampleEnd = new Pose(28, 30, Math.toRadians(180));
    private final Pose secondSamplePos = new Pose(startPushingSampleX, 15, Math.toRadians(180));
    private final Pose secondSampleControl = new Pose(74, 29, Math.toRadians(180));
    private final Pose secondSampleEnd = new Pose(25, 18, Math.toRadians(180));
    private final Pose outtakeFirstPickupPose = new Pose(10.5, 18, Math.toRadians(180));
    private final Pose outtakePickupWaitPose = new Pose(16, 30, Math.toRadians(180));
    private final Pose outtakePickupPose = new Pose(10.5, 30, Math.toRadians(180));
    private final Pose outtakePickupControlFirst = new Pose(44, 32, Math.toRadians(180));
    private final Pose outtakePickupControl1 = new Pose(15, 68, Math.toRadians(180));
    private final Pose outtakePickupControl2 = new Pose(37, 19, Math.toRadians(180));

    private final double scoreControlX = 12;
    private final Pose firstScorePose = new Pose(scoreX, scoreY-3, Math.toRadians(180));
    private final Pose firstScoreControl = new Pose(scoreControlX, scoreY - 3, Math.toRadians(180));
    private final Pose secondScorePose = new Pose(scoreX, scoreY-6, Math.toRadians(180));
    private final Pose secondScoreControl = new Pose(scoreControlX, scoreY - 6, Math.toRadians(180));
    private final Pose thirdScorePose = new Pose(scoreX, scoreY-9, Math.toRadians(180));
    private final Pose thirdScoreControl = new Pose(scoreControlX, scoreY - 9, Math.toRadians(180));
    private final Pose fourthScorePose = new Pose(scoreX, scoreY-12, Math.toRadians(180));
    private final Pose fourthScoreControl = new Pose(scoreControlX, scoreY - 12, Math.toRadians(180));
    private final Pose parkPose = new Pose(18, 30, Math.toRadians(230));
    private final double pushPathEndTimeout = 5;
    private final double pickupWaitTimeout = 10;
    private final double zeroPowerAccelerationMultiplierForPickup = 1, zeroPowerAccelerationMultiplierForPush = 5, zeroPowerAccelerationMultiplerForScore = 3.5;

    private PathChain scorePreloadPath, parkFromFourthPath;
    private PathChain goToFirstPush, pushFirstSample, goToSecondPush, pushSecondSample, firstPickupPath, secondPickupPath, thirdPickupPath, fourthPickupPath, firstScorePath, secondScorePath,thirdScorePath, fourthScorePath;
    private PathChain wallPickup;
    private PathChain[] pushWaitPaths, pushDonePaths, pickupPaths, wallPickupPaths, scorePaths;
    public void buildPaths(){
        scorePreloadPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(preloadScoreControl), new Point(preloadScorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadScorePose.getHeading())
                .build();
        goToFirstPush = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadScorePose),new Point(firstSampleControl1), new Point(firstSampleControl2), new Point(firstSampleControl3), new Point(firstSamplePos)))
                .setLinearHeadingInterpolation(preloadScorePose.getHeading(), firstSamplePos.getHeading())
                .setPathEndTimeoutConstraint(pushPathEndTimeout)
                .build();
        pushFirstSample = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(firstSamplePos), new Point(firstSampleEnd)))
                .setLinearHeadingInterpolation(firstSamplePos.getHeading(), firstSampleEnd.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPush)
                .setPathEndTimeoutConstraint(pushPathEndTimeout)
                .build();
        goToSecondPush = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(firstSampleEnd),new Point(secondSampleControl), new Point(secondSamplePos)))
                .setLinearHeadingInterpolation(firstSampleEnd.getHeading(), secondSamplePos.getHeading())
                .setPathEndTimeoutConstraint(pushPathEndTimeout)
                .build();
        pushSecondSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSamplePos), new Point(secondSampleEnd)))
                .setLinearHeadingInterpolation(secondSamplePos.getHeading(), secondSampleEnd.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPush)
                .setPathEndTimeoutConstraint(pushPathEndTimeout)
                .build();
        firstPickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(secondSampleEnd), new Point(outtakeFirstPickupPose)))
                .setLinearHeadingInterpolation(secondSampleEnd.getHeading(), outtakePickupWaitPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPickup)
                .build();
        wallPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(outtakePickupWaitPose),new Point(outtakePickupPose)))
                .setLinearHeadingInterpolation(outtakePickupWaitPose.getHeading(),outtakePickupPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPickup)
                .build();
        firstScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupWaitPose), new Point(firstScoreControl), new Point(firstScorePose)))
                .setLinearHeadingInterpolation(outtakePickupWaitPose.getHeading(),firstScorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .build();
        secondPickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(firstScorePose), new Point(outtakePickupControl1), new Point(outtakePickupControl2), new Point(outtakePickupWaitPose)))
                .setLinearHeadingInterpolation(firstScorePose.getHeading(), outtakePickupWaitPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPickup)
                .setPathEndTimeoutConstraint(pickupWaitTimeout)
                .build();
        secondScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupWaitPose),new Point(secondScoreControl), new Point(secondScorePose)))
                .setLinearHeadingInterpolation(outtakePickupWaitPose.getHeading(), secondScorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .build();
        thirdPickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(secondScorePose), new Point(outtakePickupControl1), new Point(outtakePickupControl2), new Point(outtakePickupWaitPose)))
                .setLinearHeadingInterpolation(secondScorePose.getHeading(), outtakePickupWaitPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPickup)
                .setPathEndTimeoutConstraint(pickupWaitTimeout)
                .build();
        thirdScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupWaitPose), new Point(thirdScoreControl),new Point(thirdScorePose)))
                .setLinearHeadingInterpolation(outtakePickupWaitPose.getHeading(), thirdScorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .build();
        fourthPickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(thirdScorePose), new Point(outtakePickupControl1), new Point(outtakePickupControl2), new Point(outtakePickupWaitPose)))
                .setLinearHeadingInterpolation(thirdScorePose.getHeading(), outtakePickupWaitPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPickup)
                .setPathEndTimeoutConstraint(pickupWaitTimeout)
                .build();
        fourthScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupWaitPose), new Point(fourthScoreControl), new Point(fourthScorePose)))
                .setLinearHeadingInterpolation(outtakePickupWaitPose.getHeading(), fourthScorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .build();
        parkFromFourthPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fourthScorePose), new Point(parkPose)))
                .setLinearHeadingInterpolation(fourthScorePose.getHeading(), parkPose.getHeading())
                .build();

        pushWaitPaths = new PathChain[]{goToFirstPush, goToSecondPush};
        pushDonePaths = new PathChain[]{pushFirstSample, pushSecondSample};
        pickupPaths = new PathChain[]{firstPickupPath, secondPickupPath, thirdPickupPath, fourthPickupPath};
        scorePaths = new PathChain[]{firstScorePath, secondScorePath, thirdScorePath, fourthScorePath};
    }

    enum AutoState {
        DRIVE_TO_PRELOAD_SCORE,
        SCORE_PRELOAD,
        DRIVE_TO_PUSHING,
        BEFORE_PUSHING,
        READY_FOR_PUSHING,
        PUSHING,
        READY_FOR_PICKUP,
        WALL_PICKUP,
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
                    follower.followPath(pushWaitPaths[counter], false);
                    counter = 0;
                    setPathState(AutoState.BEFORE_PUSHING);
                }
                break;
            case BEFORE_PUSHING:
                if(!follower.isBusy()){
                    outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
                    setPathState(AutoState.READY_FOR_PUSHING);
                }
                break;
            case READY_FOR_PUSHING:
                if(!follower.isBusy()){
                    follower.followPath(pushDonePaths[counter], false);
                    setPathState(AutoState.PUSHING);
                }
                break;
            case PUSHING:
                if(!follower.isBusy()){
                    counter++;
                    if(counter < 2){
                        follower.followPath(pushWaitPaths[counter]);
                        outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
                        setPathState(AutoState.READY_FOR_PUSHING);
                    } else {
                        counter = 0;
                        outtake.setClawOpen(true);
                        outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
                        follower.followPath(pickupPaths[counter], true);
                        setPathState(AutoState.READY_FOR_PICKUP); // Skips WALL_PICKUP when first pickup
                    }
                }
                break;
            case WALL_PICKUP:
                if (pathTimer.getElapsedTimeSeconds() > 0.35){
                    outtake.setClawOpen(true);
                    outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
                }
                if (!follower.isBusy()){
                    follower.followPath(wallPickup);
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
                    if(pathTimer.getElapsedTimeSeconds() > 0.1){
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
                if(pathTimer.getElapsedTimeSeconds() > 0.1){
                    outtake.setClawOpen(true);
                    if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                        counter++;
                        if (counter < 3) {
                            follower.followPath(pickupPaths[counter], false);
                            outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCOREOUT);
                            setPathState(AutoState.WALL_PICKUP);
                        } else {
                            follower.followPath(parkFromFourthPath, true);
                            outtake.setOuttakeState(Outtake.OuttakeState.RESET_ENCODER);
                            outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.RESET_ENCODER);
                            setPathState(AutoState.PARK);
                        }
                    }
                }
                break;
            case PARK:
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
