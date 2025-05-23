//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.util.Constants;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//import subsystems.OuttakeLiftSubsys;
//
//
//@Disabled
//@Autonomous (name = "Specimen Auto")
//public class Auto_5_0_Intaking extends OpMode {
//    private Follower follower;
//    private subsystems.Intake_DiffyClaw intakeDiffyClaw;
//    private OuttakeLiftSubsys outtakeLift;
//    private subsystems.Outtake outtake;
//    private Timer pathTimer;
//    private final double scoreX = 38.5;
//    private final double scoreY = 78;
//
//    private final Pose startPose = new Pose(7.1, 65.5, Math.toRadians(180));
//    private final Pose preloadScoreControl = new Pose(20, scoreY, Math.toRadians(180));
//    private final Pose preloadScorePose = new Pose(scoreX, scoreY, Math.toRadians(180));
//
//    //intake poses
//    private final Pose preloadToFirstSampleControl = new Pose(28, 16, Math.toRadians(0));
//    private final Pose intakeFirstSamplePose = new Pose(24, 24, Math.toRadians(0));
//    private final Pose intakeSecondSamplePose = new Pose(24, 12, Math.toRadians(0));
//    private final Pose intakeThirdSamplePose = new Pose(28, 16, Math.toRadians(-40));
//    private final Pose depositThirdSamplePose = new Pose(26, 32, Math.toRadians(-145));
//    int alignx=0, aligny=0;
//    private Pose AutoAlignSamplePose = new Pose(alignx, aligny, Math.toRadians(0));
//
//
//
//
//    //scoring poses
//    private final Pose outtakePickupPose = new Pose(8.65, 30, Math.toRadians(180));
//    private final Pose outtakePickupControlFirst = new Pose(44, 32, Math.toRadians(180));
//    private final Pose outtakePickupControl1 = new Pose(15, 68, Math.toRadians(180));
//    private final Pose outtakePickupControl2 = new Pose(37, 19, Math.toRadians(180));
//
//    private final double scoreControlX = 12;
//    private final Pose firstScorePose = new Pose(scoreX, scoreY-3, Math.toRadians(180));
//    private final Pose firstScoreControl = new Pose(scoreControlX, scoreY - 3, Math.toRadians(180));
//    private final Pose secondScorePose = new Pose(scoreX, scoreY-6, Math.toRadians(180));
//    private final Pose secondScoreControl = new Pose(scoreControlX, scoreY - 6, Math.toRadians(180));
//    private final Pose thirdScorePose = new Pose(scoreX, scoreY-9, Math.toRadians(180));
//    private final Pose thirdScoreControl = new Pose(scoreControlX, scoreY - 9, Math.toRadians(180));
//    private final Pose fourthScorePose = new Pose(scoreX, scoreY-12, Math.toRadians(180));
//    private final Pose fourthScoreControl = new Pose(scoreControlX, scoreY - 12, Math.toRadians(180));
//    private final Pose parkPose = new Pose(20, 50, Math.toRadians(230));
//
//    private final double zeroPowerAccelerationMultiplierForPickup = 2.75 , zeroPowerAccelerationMultiplierForPush = 5, zeroPowerAccelerationMultiplerForScore = 4.3;
//
//    private PathChain scorePreloadPath, parkFromFourthPath;
//    private PathChain goToFirstPush, intakeFirstSample, intakeSecondSample, intakeThirdSample,depositThirdSample, firstPickupPath, secondPickupPath, thirdPickupPath, fourthPickupPath, firstScorePath, secondScorePath,thirdScorePath, fourthScorePath;
//
//    private PathChain[] intakePaths, pickupPaths, scorePaths;
//    public void buildPaths(){
//        scorePreloadPath = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(startPose), new Point(preloadScoreControl), new Point(preloadScorePose)))
//                .setLinearHeadingInterpolation(startPose.getHeading(), preloadScorePose.getHeading())
//                .build();
//        intakeFirstSample = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(preloadScorePose),new Point(preloadToFirstSampleControl), new Point(intakeFirstSamplePose)))
//                .setLinearHeadingInterpolation(preloadScorePose.getHeading(), intakeFirstSamplePose.getHeading())
//                .build();
//        intakeSecondSample = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(intakeFirstSamplePose), new Point(intakeSecondSamplePose)))
//                .setLinearHeadingInterpolation(intakeFirstSamplePose.getHeading(), intakeSecondSamplePose.getHeading())
//                .build();
//        intakeThirdSample = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(intakeSecondSamplePose), new Point(intakeThirdSamplePose)))
//                .setLinearHeadingInterpolation(intakeSecondSamplePose.getHeading(), intakeThirdSamplePose.getHeading())
//                .build();
//        depositThirdSample = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(intakeThirdSamplePose), new Point(depositThirdSamplePose)))
//                .setLinearHeadingInterpolation(intakeThirdSamplePose.getHeading(),depositThirdSamplePose.getHeading())
//                .build();
//        firstPickupPath = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(depositThirdSamplePose), new Point(outtakePickupPose)))
//                .setLinearHeadingInterpolation(depositThirdSamplePose.getHeading(), outtakePickupPose.getHeading())
//                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPickup)
//                .build();
//        firstScorePath = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(outtakePickupPose), new Point(firstScoreControl), new Point(firstScorePose)))
//                .setLinearHeadingInterpolation(outtakePickupPose.getHeading(),firstScorePose.getHeading())
//                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
//                .build();
//        secondPickupPath = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(firstScorePose), new Point(outtakePickupControl1), new Point(outtakePickupControl2), new Point(outtakePickupPose)))
//                .setLinearHeadingInterpolation(firstScorePose.getHeading(), outtakePickupPose.getHeading())
//                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPickup)
//                .build();
//        secondScorePath = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(outtakePickupPose),new Point(secondScoreControl), new Point(secondScorePose)))
//                .setLinearHeadingInterpolation(outtakePickupPose.getHeading(), secondScorePose.getHeading())
//                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
//                .build();
//        thirdPickupPath = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(secondScorePose), new Point(outtakePickupControl1), new Point(outtakePickupControl2), new Point(outtakePickupPose)))
//                .setLinearHeadingInterpolation(secondScorePose.getHeading(), outtakePickupPose.getHeading())
//                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPickup)
//                .build();
//        thirdScorePath = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(outtakePickupPose), new Point(thirdScoreControl),new Point(thirdScorePose)))
//                .setLinearHeadingInterpolation(outtakePickupPose.getHeading(), thirdScorePose.getHeading())
//                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
//                .build();
//        fourthPickupPath = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(thirdScorePose), new Point(outtakePickupControl1), new Point(outtakePickupControl2), new Point(outtakePickupPose)))
//                .setLinearHeadingInterpolation(thirdScorePose.getHeading(), outtakePickupPose.getHeading())
//                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPickup)
//                .build();
//        fourthScorePath = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(outtakePickupPose), new Point(fourthScoreControl), new Point(fourthScorePose)))
//                .setLinearHeadingInterpolation(outtakePickupPose.getHeading(), fourthScorePose.getHeading())
//                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
//                .build();
//        parkFromFourthPath = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(fourthScorePose), new Point(parkPose)))
//                .setLinearHeadingInterpolation(fourthScorePose.getHeading(), parkPose.getHeading())
//                .build();
//
//        intakePaths = new PathChain[]{intakeFirstSample, intakeSecondSample,intakeThirdSample,depositThirdSample};
//        pickupPaths = new PathChain[]{firstPickupPath, secondPickupPath, thirdPickupPath, fourthPickupPath};
//        scorePaths = new PathChain[]{firstScorePath, secondScorePath, thirdScorePath, fourthScorePath};
//    }
//
//    enum AutoState {
//        DRIVE_TO_PRELOAD_SCORE,
//        SCORE_PRELOAD,
//        DRIVE_TO_INTAKE,
//        INTAKE_READY,
//        INTAKING,
//        DEPOSITING,
//        READY_FOR_PICKUP,
//        PICKUP,
//        READY_TO_SCORE,
//        SCORE,
//        PARK
//    }
//    private AutoState autoState = AutoState.DRIVE_TO_PRELOAD_SCORE;
//
//    public void setPathState(AutoState newState){
//        autoState = newState;
//        pathTimer.resetTimer();
//    }
//
//    private int counter = 0;
//    public void autonomousPathUpdate(){
//        switch (autoState){
//           g case DRIVE_TO_PRELOAD_SCORE:
//                outtake.setClawState(subsystems.Outtake.ClawStates.CLOSED);
//                outtake.setOuttakeState(subsystems.Outtake.OuttakeState.SPECBACKSCORE);
//                outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.BACK_SCORE);
//                if(pathTimer.getElapsedTimeSeconds() > 1.7){
//                    follower.followPath(scorePreloadPath,true);
//                    setPathState(AutoState.SCORE_PRELOAD);
//                }
//                break;
//            case SCORE_PRELOAD:
//                if(!follower.isBusy()){
//                    outtake.setClawState(subsystems.Outtake.ClawStates.OPEN);
//                    setPathState(AutoState.DRIVE_TO_INTAKE);
//                }
//                break;
//            case DRIVE_TO_INTAKE:
//                if(pathTimer.getElapsedTimeSeconds() > 0.1){
//                    follower.followPath(intakePaths[counter], true);
//                    counter = 0;
//                    outtake.setOuttakeState(subsystems.Outtake.OuttakeState.RESET_ENCODER);
//                    outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.AVOID_INTAKE);
//                    setPathState(AutoState.INTAKE_READY);
//                }
//                break;
//            case INTAKE_READY:
//                if(!follower.isBusy()){
//                    intakeDiffyClaw.ExtendTo(subsystems.Intake_DiffyClaw.IntakePositions.AUTO_INTAKE_POSE);
//                    intakeDiffyClaw.setIntakeState(subsystems.Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
//                    intakeDiffyClaw.setClawState(subsystems.Outtake.ClawStates.OPEN);
//                    intakeDiffyClaw.HoldExtension();
//                    setPathState(AutoState.INTAKING);
//                }
//                break;
//            case INTAKING:
//                if (pathTimer.getElapsedTimeSeconds()>0.4){
//                    intakeDiffyClaw.setIntakeState(subsystems.Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
//                }
//                if (pathTimer.getElapsedTimeSeconds()>0.6){
//                    intakeDiffyClaw.setClawState(subsystems.Outtake.ClawStates.CLOSED);
//                }
//                if (pathTimer.getElapsedTimeSeconds()>0.8){
//                    intakeDiffyClaw.setIntakeState(subsystems.Intake_DiffyClaw.IntakeState.DEPOSIT);
//                    setPathState(AutoState.DEPOSITING);
//                }
//
//                break;
//            case DEPOSITING:
//                if(!follower.isBusy()){
//                    intakeDiffyClaw.setClawState(subsystems.Outtake.ClawStates.OPEN);
//                    counter++;
//                    if(counter < 3){
//                        follower.followPath(intakePaths[counter]);
//                        setPathState(AutoState.INTAKE_READY);
//                    } else {
//                        counter = 0;
//                        setPathState(AutoState.READY_FOR_PICKUP);
//                        outtake.setClawState(subsystems.Outtake.ClawStates.OPEN);
//                        outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.FRONT_PICKUP);
//                        follower.followPath(pickupPaths[counter], true);
//                    }
//                }
//                break;
//            case READY_FOR_PICKUP:
//                outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.FRONT_PICKUP);
//                outtake.setClawState(subsystems.Outtake.ClawStates.OPEN);
//                if(!follower.isBusy()){
//                    setPathState(AutoState.PICKUP);
//                }
//                break;
//            case PICKUP:
//                if(pathTimer.getElapsedTimeSeconds() > 0.15){
//                    outtake.setClawState(subsystems.Outtake.ClawStates.CLOSED);
//                    if(pathTimer.getElapsedTimeSeconds() > 0.5){
//                        outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.BACK_SCORE);
//                        outtake.setOuttakeState(subsystems.Outtake.OuttakeState.SPECBACKSCORE);
//                        follower.followPath(scorePaths[counter], true);
//                        setPathState(AutoState.READY_TO_SCORE);
//                    }
//                }
//                break;
//            case READY_TO_SCORE:
//                if(!follower.isBusy()){
//                    setPathState(AutoState.SCORE);
//                }
//                break;
//            case SCORE:
//                if(pathTimer.getElapsedTimeSeconds() > 0.05){
//                    outtake.setClawState(subsystems.Outtake.ClawStates.OPEN);
//                    if (pathTimer.getElapsedTimeSeconds() > 0.2) {
//                        counter++;
//                        if (counter < 3) {
//                            follower.followPath(pickupPaths[counter], true);
//                            outtake.setOuttakeState(subsystems.Outtake.OuttakeState.SPECFRONTPICKUP);
//                            outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.FRONT_PICKUP);
//                            setPathState(AutoState.READY_FOR_PICKUP);
//                        } else {
//                            follower.followPath(parkFromFourthPath, true);
//                            outtake.setOuttakeState(subsystems.Outtake.OuttakeState.RESET_ENCODER);
//                            outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.RESET_ENCODER);
//                            setPathState(AutoState.PARK);
//                        }
//                    }
//                }
//                break;
//            case PARK:
//                break;
//            default:
//                break;
//        }
//    }
//    @Override
//    public void init(){
//        pathTimer = new Timer();
//        Constants.setConstants(FConstants.class, LConstants.class);
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//
//        intakeDiffyClaw = new subsystems.Intake_DiffyClaw();
//        outtake = new subsystems.Outtake(hardwareMap);
//        outtakeLift = new OuttakeLift(hardwareMap,this);
//        buildPaths();
//    }
//
//    private final ToggleButton teamColorButton = new ToggleButton(Storage.isRed);
//    @Override
//    public void init_loop(){
//        teamColorButton.input(gamepad1.dpad_up);
//        Storage.isRed = teamColorButton.getVal();
//
//        telemetry.addData("Team Color:", Storage.isRed ? "Red" : "Blue");
//        telemetry.update();
//    }
//    @Override
//    public void loop(){
//        follower.update();
//        autonomousPathUpdate();
//        outtake.outtakeLoop();
//        outtakeLift.holdLift();
//
//
//        Storage.CurrentPose = follower.getPose();
//        telemetry.update();
//    }
//}
