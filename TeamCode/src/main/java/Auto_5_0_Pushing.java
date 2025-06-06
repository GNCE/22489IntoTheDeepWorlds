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
import subsystems.Outtake;
import subsystems.OuttakeLiftSubsys;
import subsystems.SubsysCore;
import subsystems.UnifiedTelemetry;
import utils.Storage;

@Disabled
@Deprecated

@Autonomous (name = "5+0 Pushing Specimen Auto")
public class Auto_5_0_Pushing extends OpMode {
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

    private final double startPushingSampleX = 58;
    private final Pose firstSamplePos = new Pose(startPushingSampleX, 25, Math.toRadians(180));
    private final Pose firstSampleControl1 = new Pose(0, 60, Math.toRadians(180));
    private final Pose firstSampleControl2 = new Pose(29, 8, Math.toRadians(180));
    private final Pose firstSampleControl3 = new Pose(66, 56, Math.toRadians(180));
    private final Pose firstSampleEnd = new Pose(18, 25, Math.toRadians(180));
    private final Pose secondSamplePos = new Pose(startPushingSampleX, 14, Math.toRadians(180));
    private final Pose secondSampleControl = new Pose(74, 29, Math.toRadians(180));
    private final Pose secondSampleEnd = new Pose(18, 14, Math.toRadians(180));
    private final Pose thirdSamplePos = new Pose(startPushingSampleX, 8, Math.toRadians(180));
    private final Pose thirdSampleControl = new Pose(63, 22, Math.toRadians(180));
    private final Pose thirdSampleEnd = new Pose(9, 8, Math.toRadians(180));

    private final Pose outtakePickupPose = new Pose(9, 30, Math.toRadians(180));
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
    private final Pose fifthScorePose = new Pose(scoreX, scoreY-14, Math.toRadians(180));
    private final Pose fifthScoreControl = new Pose(scoreControlX, scoreY - 14, Math.toRadians(180));
    private final Pose parkPose = new Pose(12, 14, Math.toRadians(230));

    private final double pushPathEndTimeout = 50;
    private final double zeroPowerAccelerationMultiplierForPickup = 1.6, zeroPowerAccelerationMultiplierForPush = 5, zeroPowerAccelerationMultiplerForScore = 3.5;

    private PathChain scorePreloadPath, parkFromFourthPath;
    private PathChain goToFirstPush, pushFirstSample, goToSecondPush, pushSecondSample, firstPickupPath, secondPickupPath, thirdPickupPath, fourthPickupPath, firstScorePath, secondScorePath,thirdScorePath, fourthScorePath;
private PathChain goToThirdPush, pushThirdSample, fifthPickupPath, fifthScorePath;
    private PathChain[] pushWaitPaths, pushDonePaths, pickupPaths, scorePaths;
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
        goToThirdPush = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(secondSampleEnd), new Point(thirdSampleControl), new Point(thirdSamplePos)))
                .setLinearHeadingInterpolation(secondSampleEnd.getHeading(), thirdSamplePos.getHeading())
                .setPathEndTimeoutConstraint(pushPathEndTimeout)
                .build();
        pushThirdSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdSamplePos),new Point(thirdSampleEnd)))
                .setLinearHeadingInterpolation(thirdSamplePos.getHeading(),thirdSampleEnd.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPickup)
                .build();
        firstScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupPose), new Point(firstScoreControl), new Point(firstScorePose)))
                .setLinearHeadingInterpolation(outtakePickupPose.getHeading(),firstScorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .build();
        secondPickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(firstScorePose), new Point(outtakePickupControl1), new Point(outtakePickupControl2), new Point(outtakePickupPose)))
                .setLinearHeadingInterpolation(firstScorePose.getHeading(), outtakePickupPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPickup)
                .build();
        secondScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupPose),new Point(secondScoreControl), new Point(secondScorePose)))
                .setLinearHeadingInterpolation(outtakePickupPose.getHeading(), secondScorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .build();
        thirdPickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(secondScorePose), new Point(outtakePickupControl1), new Point(outtakePickupControl2), new Point(outtakePickupPose)))
                .setLinearHeadingInterpolation(secondScorePose.getHeading(), outtakePickupPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPickup)
                .build();
        thirdScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupPose), new Point(thirdScoreControl),new Point(thirdScorePose)))
                .setLinearHeadingInterpolation(outtakePickupPose.getHeading(), thirdScorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .build();
        fourthPickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(thirdScorePose), new Point(outtakePickupControl1), new Point(outtakePickupControl2), new Point(outtakePickupPose)))
                .setLinearHeadingInterpolation(thirdScorePose.getHeading(), outtakePickupPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPickup)
                .build();
        fourthScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupPose), new Point(fourthScoreControl), new Point(fourthScorePose)))
                .setLinearHeadingInterpolation(outtakePickupPose.getHeading(), fourthScorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .build();
        fifthPickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(fourthScorePose),new Point(outtakePickupControl1),new Point(outtakePickupPose)))
                .setLinearHeadingInterpolation(fourthScorePose.getHeading(),outtakePickupPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
                .build();
        fifthScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupPose), new Point(fifthScoreControl),new Point(fifthScorePose)))
                .setLinearHeadingInterpolation(outtakePickupPose.getHeading(),fifthScorePose.getHeading())
                .build();
        parkFromFourthPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fifthScorePose), new Point(parkPose)))
                .setLinearHeadingInterpolation(fifthScorePose.getHeading(), parkPose.getHeading())
                .build();

        pushWaitPaths = new PathChain[]{goToFirstPush, goToSecondPush, goToThirdPush};
        pushDonePaths = new PathChain[]{pushFirstSample, pushSecondSample, pushThirdSample};
        pickupPaths = new PathChain[]{firstPickupPath, secondPickupPath, thirdPickupPath, fourthPickupPath, fifthPickupPath};
        scorePaths = new PathChain[]{firstScorePath, secondScorePath, thirdScorePath, fourthScorePath, fifthScorePath};
    }

    enum AutoState {
        DRIVE_TO_PRELOAD_SCORE,
        SCORE_PRELOAD,
        DRIVE_TO_PUSHING,
        BEFORE_PUSHING,
        READY_FOR_PUSHING,
        PUSHING,
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

    private int counter = 0;
    public void autonomousPathUpdate(){
        switch (autoState){
            case DRIVE_TO_PRELOAD_SCORE:
                outtake.setClawState(Outtake.ClawStates.CLOSED);
                outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCORE);
                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.BACK_SCORE);
                if(pathTimer.getElapsedTimeSeconds() > 3){
                    follower.followPath(scorePreloadPath,true);
                    setPathState(AutoState.SCORE_PRELOAD);
                }
                break;
            case SCORE_PRELOAD:
                if(!follower.isBusy()){
                    outtake.setClawState(Outtake.ClawStates.OPEN);
                    setPathState(AutoState.DRIVE_TO_PUSHING);
                }
                break;
            case DRIVE_TO_PUSHING:
                if(pathTimer.getElapsedTimeSeconds() > 0.2){
                    follower.followPath(pushWaitPaths[counter], true);
                    counter = 0;
                    setPathState(AutoState.BEFORE_PUSHING);
                }
                break;
            case BEFORE_PUSHING:
                if(pathTimer.getElapsedTimeSeconds() > 0.4){
                    outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
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
                    if(counter < 3){
                        follower.followPath(pushWaitPaths[counter]);
                        setPathState(AutoState.READY_FOR_PUSHING);
                    } else {
                        counter = 0;
                        setPathState(AutoState.READY_FOR_PICKUP);
                        outtake.setClawState(Outtake.ClawStates.OPEN);
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
                        //follower.followPath(pickupPaths[counter], true);
                    }
                }
                break;
            case READY_FOR_PICKUP:
                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
                outtake.setClawState(Outtake.ClawStates.OPEN);
                if(!follower.isBusy()){
                    setPathState(AutoState.PICKUP);
                }
                break;
            case PICKUP:
                if(pathTimer.getElapsedTimeSeconds() > 0.15){
                    outtake.setClawState(Outtake.ClawStates.CLOSED);
                    if(pathTimer.getElapsedTimeSeconds() > 0.5){
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
                    outtake.setClawState(Outtake.ClawStates.OPEN);
                    if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                        counter++;
                        if (counter < 4) {
                            follower.followPath(pickupPaths[counter], true);
                            outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
                            outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
                            setPathState(AutoState.READY_FOR_PICKUP);
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
        SubsysCore.setGlobalParameters(hardwareMap, this);
        intakeDiffyClaw = new Intake_DiffyClaw();
        intakeDiffyClaw.init();
        outtake = new Outtake(); outtake.init();
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
        outtake.loop();
        tel.addData("Team Color:", Storage.isRed ? "Red" : "Blue");
        tel.update();
    }
    @Override
    public void loop(){
        follower.update();
        autonomousPathUpdate();
        outtake.loop();
        outtakeLift.holdLift();
        outtakeLift.loop();
        intakeDiffyClaw.loop();


        Storage.CurrentPose = follower.getPose();
        telemetry.update();
    }
}
