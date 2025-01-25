import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
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
public class EC_Auto_Specimen extends OpMode {
    private Follower follower;
    private Intake intake;
    private OuttakeLift outtakeLift;
    private Outtake outtake;
    private Misc misc;
    private Timer pathTimer;
    private final double scoreX = 41;
    private final double scoreY = 73;
    private final double intakeX = 25;

    private final Pose startPose = new Pose(9.1, 54.125, Math.toRadians(0));
    private final Pose preloadScorePose = new Pose(scoreX, scoreY, Math.toRadians(0));
    private final Pose firstIntakePose = new Pose(intakeX, 23, Math.toRadians(0));
    private final Pose secondIntakePose = new Pose(intakeX, 13, Math.toRadians(0));
    private final Pose thirdIntakePose = new Pose(intakeX, 11, Math.toRadians(342));
    private final Pose outtakePickupPose = new Pose(20, 35, Math.toRadians(0));
    private final Pose firstScorePose = new Pose(scoreX, scoreY-3, Math.toRadians(0));
    private final Pose secondScorePose = new Pose(scoreX, scoreY-6, Math.toRadians(0));
    private final Pose thirdScorePose = new Pose(scoreX, scoreY-9, Math.toRadians(0));
    private final Pose fourthScorePose = new Pose(scoreX, scoreY-12, Math.toRadians(0));
    private final Pose parkPose = new Pose(20, 45, Math.toRadians(60));

    private PathChain scorePreloadPath, parkFromFourthPath;
    private PathChain firstIntakePath, secondIntakePath, thirdIntakePath, firstPickupPath, secondPickupPath, thirdPickupPath, fourthPickupPath, firstScorePath, secondScorePath, thirdScorePath, fourthScorePath;

    private PathChain[] intakePaths, pickupPaths, scorePaths;
    public void buildPaths(){
        scorePreloadPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadScorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadScorePose.getHeading())
                .build();
        firstIntakePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preloadScorePose), new Point(firstIntakePose)))
                .setLinearHeadingInterpolation(preloadScorePose.getHeading(), firstIntakePose.getHeading())
                .build();
        secondIntakePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstIntakePose), new Point(secondIntakePose)))
                .setLinearHeadingInterpolation(firstIntakePose.getHeading(), secondIntakePose.getHeading())
                .build();
        thirdIntakePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondIntakePose), new Point(thirdIntakePose)))
                .setLinearHeadingInterpolation(secondIntakePose.getHeading(), thirdIntakePose.getHeading())
                .build();
        firstPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdIntakePose), new Point(outtakePickupPose)))
                .setLinearHeadingInterpolation(thirdIntakePose.getHeading(), outtakePickupPose.getHeading())
                .build();
        firstScorePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(outtakePickupPose), new Point(firstScorePose)))
                .setLinearHeadingInterpolation(outtakePickupPose.getHeading(), firstScorePose.getHeading())
                .build();
        secondPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstScorePose), new Point(outtakePickupPose)))
                .setLinearHeadingInterpolation(firstScorePose.getHeading(), outtakePickupPose.getHeading())
                .build();
        secondScorePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(outtakePickupPose), new Point(secondScorePose)))
                .setLinearHeadingInterpolation(outtakePickupPose.getHeading(), secondScorePose.getHeading())
                .build();
        thirdPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondScorePose), new Point(outtakePickupPose)))
                .setLinearHeadingInterpolation(secondScorePose.getHeading(), outtakePickupPose.getHeading())
                .build();
        thirdScorePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(outtakePickupPose), new Point(thirdScorePose)))
                .setLinearHeadingInterpolation(outtakePickupPose.getHeading(), thirdScorePose.getHeading())
                .build();
        fourthPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdScorePose), new Point(outtakePickupPose)))
                .setLinearHeadingInterpolation(thirdScorePose.getHeading(), outtakePickupPose.getHeading())
                .build();
        fourthScorePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(outtakePickupPose), new Point(fourthScorePose)))
                .setLinearHeadingInterpolation(outtakePickupPose.getHeading(), fourthScorePose.getHeading())
                .build();
        parkFromFourthPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fourthScorePose), new Point(parkPose)))
                .setLinearHeadingInterpolation(fourthScorePose.getHeading(), parkPose.getHeading())
                .build();

        intakePaths = new PathChain[]{firstIntakePath, secondIntakePath, thirdIntakePath};
        pickupPaths = new PathChain[]{firstPickupPath, secondPickupPath, thirdPickupPath, fourthPickupPath};
        scorePaths = new PathChain[]{firstScorePath, secondScorePath, thirdScorePath, fourthScorePath};
    }

    enum AutoState {
        DRIVE_TO_PRELOAD_SCORE,
        SCORE_PRELOAD,
        READY_FOR_INTAKE,
        INTAKE,
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
                outtake.setClaw(false);
                outtake.pivotToFront();
                outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.FRONT_SCORE_WAIT);
                setPathState(AutoState.SCORE_PRELOAD);
                break;
            case SCORE_PRELOAD:
                if(!follower.isBusy()){
                    outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.FRONT_SCORE_DONE);
                    if(!outtakeLift.isBusy()){
                        outtake.setClaw(true);
                        if(!outtake.isClawBusy()){
                            outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.BACK_PICKUP);
                            follower.followPath(firstIntakePath, true);
                            counter = 0;
                            setPathState(AutoState.READY_FOR_INTAKE);
                        }
                    }
                }
                break;
            case READY_FOR_INTAKE:
                if(!outtakeLift.isBusy()){
                    outtake.setClaw(false);
                    if(!outtake.isClawBusy()){
                        outtake.pivotToPickupBack();
                    }
                }
                if(!follower.isBusy()){
                    intake.startIntake();
                    intake.ManualExtend();
                    setPathState(AutoState.INTAKE);
                }
                break;
            case INTAKE:
                if(intake.isCorrectColor() || pathTimer.getElapsedTimeSeconds() > 4){
                    intake.ManualRetract();
                    if(intake.extendo.getCurrentPosition() < 15){
                        intake.deposit();
                        counter++;
                        if(counter < 3){
                            follower.followPath(intakePaths[counter], true);
                            setPathState(AutoState.READY_FOR_INTAKE);
                        } else {
                            setPathState(AutoState.READY_FOR_PICKUP);
                            counter = 0;
                            outtake.setClaw(true);
                            follower.followPath(pickupPaths[counter], true);
                        }
                    }
                }
                break;
            case READY_FOR_PICKUP:
                outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.BACK_PICKUP);
                if(!outtakeLift.isBusy()){
                    outtake.setClaw(false);
                    if(!outtake.isClawBusy()){
                        outtake.pivotToPickupBack();
                        if(!outtake.isArmBusy()){
                            outtake.setClaw(true);
                            setPathState(AutoState.PICKUP);
                        }
                    }
                }
                break;
            case PICKUP:
                if(!follower.isBusy()){
                    outtake.setClaw(false);
                    if(!outtake.isClawBusy()){
                        outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.FRONT_SCORE_WAIT);
                        outtake.pivotToFront();
                        follower.followPath(scorePaths[counter], true);
                        setPathState(AutoState.SCORE);
                    }
                }
                break;
            case SCORE:
                if(!follower.isBusy()){
                    outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.FRONT_SCORE_DONE);
                    if(!outtakeLift.isBusy()){
                        outtake.setClaw(true);
                        if(!outtake.isClawBusy()){
                            counter++;
                            if(counter < 4){
                                follower.followPath(pickupPaths[counter], true);
                                setPathState(AutoState.READY_FOR_PICKUP);
                            } else {
                                follower.followPath(parkFromFourthPath, true);
                                setPathState(AutoState.PARK);
                            }
                        }
                    }
                }
                break;
            case PARK:
                if(pathTimer.getElapsedTimeSeconds() > 5){
                    intake.ManualExtend();
                }
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
        outtakeLift.HoldLift();
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
        intake.extendoLoop();
        intake.intakeLoop();
        outtake.loop();
        outtakeLift.HoldLift();

        Storage.CurrentPose = follower.getPose();

        telemetry.update();
    }
}
