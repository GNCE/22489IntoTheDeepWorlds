import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous (name = "Specimen Auto")
public class EC_Auto_4_0_PUSHING extends OpMode {
    private Follower follower;
    private Intake intake;
    private OuttakeLift outtakeLift;
    private Outtake outtake;
    private Misc misc;
    private Timer pathTimer;
    private final double scoreX = 41;
    private final double scoreY = 69;


    private final Pose startPose = new Pose(7.5, 66, Math.toRadians(180));
    private final Pose preloadScorePose = new Pose(scoreX, scoreY, Math.toRadians(180));
    private final Pose firstSamplePos = new Pose(57, 34, Math.toRadians(90));
    private final Pose firstSampleControl = new Pose(57, 34, Math.toRadians(90));
    private final Pose firstSampleEnd = new Pose(18, 24, Math.toRadians(90));
    private final Pose firstSampleEndControl = new Pose(57, 34, Math.toRadians(90));
    private final Pose secondSamplePos = new Pose(61, 14, Math.toRadians(90));
    private final Pose secondSampleControl = new Pose(57, 34, Math.toRadians(90));
    private final Pose secondSampleEnd = new Pose(18, 13, Math.toRadians(90));

    private final Pose outtakePickupPose = new Pose(8, 31, Math.toRadians(180));
    private final Pose scoreControl = new Pose(10, 69, Math.toRadians(0));
    private final Pose scoreToPickupControl = new Pose(42, 55, Math.toRadians(0));
    private final Pose firstScorePose = new Pose(scoreX, scoreY-3, Math.toRadians(0));
    private final Pose secondScorePose = new Pose(scoreX, scoreY-6, Math.toRadians(0));
    private final Pose thirdScorePose = new Pose(scoreX, scoreY-9, Math.toRadians(0));
    private final Pose fourthScorePose = new Pose(scoreX, scoreY-12, Math.toRadians(0));
    private final Pose parkPose = new Pose(20, 45, Math.toRadians(240));

    private PathChain scorePreloadPath, parkFromFourthPath;
    private PathChain goToFirstPush, pushFirstSample, goToSecondPush, pushSecondSample, firstPickupPath, secondPickupPath, thirdPickupPath, fourthPickupPath, firstScorePath, secondScorePath,thirdScorePath, fourthScorePath;

    private PathChain[] pushPaths, pickupPaths, scorePaths;
    public void buildPaths(){
        scorePreloadPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadScorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadScorePose.getHeading())
                .build();
        goToFirstPush = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadScorePose),new Point(firstSampleControl), new Point(firstSamplePos)))
                .setLinearHeadingInterpolation(preloadScorePose.getHeading(), firstSamplePos.getHeading())
                .build();
        pushFirstSample = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(firstSamplePos),new Point(firstSampleEndControl), new Point(firstSampleEnd)))
                .setLinearHeadingInterpolation(firstSamplePos.getHeading(), firstSampleEnd.getHeading())
                .build();
        goToSecondPush = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(firstSampleEnd),new Point(secondSampleControl), new Point(secondSamplePos)))
                .setLinearHeadingInterpolation(firstSampleEnd.getHeading(), secondSamplePos.getHeading())
                .build();
        pushSecondSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSamplePos), new Point(secondSampleEnd)))
                .setLinearHeadingInterpolation(secondSamplePos.getHeading(), secondSampleEnd.getHeading())
                .build();
        firstPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSampleEnd), new Point(outtakePickupPose)))
                .setLinearHeadingInterpolation(secondSampleEnd.getHeading(), outtakePickupPose.getHeading())
                .build();
        firstScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupPose), new Point(scoreControl), new Point(firstScorePose)))
                .setLinearHeadingInterpolation(outtakePickupPose.getHeading(),firstScorePose.getHeading())
                .build();
        secondPickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(firstScorePose), new Point(scoreToPickupControl),new Point(outtakePickupPose)))
                .setLinearHeadingInterpolation(firstScorePose.getHeading(), outtakePickupPose.getHeading())
                .build();
        secondScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupPose),new Point(scoreControl), new Point(secondScorePose)))
                .setLinearHeadingInterpolation(outtakePickupPose.getHeading(), secondScorePose.getHeading())
                .build();
        thirdPickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(secondScorePose), new Point(scoreToPickupControl),new Point(outtakePickupPose)))
                .setLinearHeadingInterpolation(secondScorePose.getHeading(), outtakePickupPose.getHeading())
                .build();
        thirdScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupPose),new Point(scoreControl), new Point(thirdScorePose)))
                .setLinearHeadingInterpolation(outtakePickupPose.getHeading(), thirdScorePose.getHeading())
                .build();
        fourthPickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(thirdScorePose),new Point(scoreToPickupControl), new Point(outtakePickupPose)))
                .setLinearHeadingInterpolation(thirdScorePose.getHeading(), outtakePickupPose.getHeading())
                .build();
        fourthScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupPose), new Point(scoreControl), new Point(fourthScorePose)))
                .setLinearHeadingInterpolation(outtakePickupPose.getHeading(), fourthScorePose.getHeading())
                .build();
        parkFromFourthPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fourthScorePose), new Point(parkPose)))
                .setLinearHeadingInterpolation(fourthScorePose.getHeading(), parkPose.getHeading())
                .build();

        pushPaths = new PathChain[]{goToFirstPush, pushFirstSample, goToSecondPush, pushSecondSample};
        pickupPaths = new PathChain[]{firstPickupPath, secondPickupPath, thirdPickupPath,fourthPickupPath};
        scorePaths = new PathChain[]{firstScorePath, secondScorePath, thirdScorePath, fourthScorePath};
    }

    enum AutoState {
        DRIVE_TO_PRELOAD_SCORE,
        SCORE_PRELOAD,
        READY_FOR_PUSHING,
        PUSHING,
        READY_FOR_PICKUP,
        PICKUP,
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
                follower.followPath(scorePreloadPath,true);
                outtake.setClawOpen(false);
                outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCORE);
                outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.BACK_SCORE);
                setPathState(AutoState.SCORE_PRELOAD);
                break;
            case SCORE_PRELOAD:
                if(!follower.isBusy()){
                    outtake.setClawOpen(true);
                        if(pathTimer.getElapsedTimeSeconds() > 0.3){
                            follower.followPath(pushPaths[0], true);
                            counter = 0;
                            setPathState(AutoState.READY_FOR_PUSHING);
                        }

                }
                break;
            case READY_FOR_PUSHING:
                if(!follower.isBusy()){
                    setPathState(AutoState.PUSHING);
                }
                break;
            case PUSHING:
                counter++;
                if(counter < 4){
                    follower.followPath(pushPaths[counter], true);
                    setPathState(AutoState.READY_FOR_PUSHING);
                } else {
                    setPathState(AutoState.READY_FOR_PICKUP);
                    counter = 0;
                    outtake.setClawOpen(true);
                    outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.FRONT_PICKUP);
                    follower.followPath(pickupPaths[counter], true);
                }
                break;
            case READY_FOR_PICKUP:
                outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.FRONT_PICKUP);
                outtake.setClawOpen(true);
                if(!outtakeLift.isBusy()){
                            setPathState(AutoState.PICKUP);

                    }

                break;
            case PICKUP:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    outtake.setClawOpen(false);
                    if(pathTimer.getElapsedTimeSeconds() > 0.3){
                        outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.BACK_SCORE);
                        outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCORE);
                        follower.followPath(scorePaths[counter], true);
                        setPathState(AutoState.SCORE);
                    }
                }
                break;
            case SCORE:
                if(!follower.isBusy()){
                        outtake.setClawOpen(true);
                        if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                            counter++;
                            if (counter < 4) {
                                follower.followPath(pickupPaths[counter], true);
                                setPathState(AutoState.READY_FOR_PICKUP);
                            } else {
                                follower.followPath(parkFromFourthPath, true);
                                intake.initiate();
                                outtake.setOuttakeState(Outtake.OuttakeState.RESET_ENCODER);
                                outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.RESET_ENCODER);
                                setPathState(AutoState.PARK);
                            }
                        }

                }
                break;
            case PARK:
                intake.setExtensionTarget(intake.FULL_EXTENSION);
                intake.intakeLoop();
                break;
            default:
                break;
        }
    }
    @Override
    public void init(){
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        intake = new Intake(hardwareMap, this);
        outtake = new Outtake(hardwareMap);
        outtakeLift = new OuttakeLift(hardwareMap,this);
        buildPaths();
    }
    @Override
    public void start(){
        intake.initiate();
    }

    private final ToggleButton teamColorButton = new ToggleButton(Storage.isRed);
    @Override
    public void init_loop(){
        teamColorButton.input(gamepad1.dpad_up);
        Storage.isRed = teamColorButton.getVal();

        telemetry.addData("Team Color:", Storage.isRed ? "Red" : "Blue");
        telemetry.update();
    }
    @Override
    public void loop(){
        follower.update();
        autonomousPathUpdate();
        outtakeLift.HoldLift();

        Storage.CurrentPose = follower.getPose();
        telemetry.update();
    }
}
