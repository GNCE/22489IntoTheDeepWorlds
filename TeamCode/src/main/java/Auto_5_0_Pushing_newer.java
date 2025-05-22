//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import java.sql.Array;
//import java.util.ArrayList;
//import java.util.Arrays;
//import java.util.List;
//import java.util.stream.Collectors;
//
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//import subsystems.OuttakeLiftSubsys;
//import subsystems.SubsysCore;
//import subsystems.UnifiedTelemetry;
//
//
//@Autonomous (name = "Auto_5_0_pushing_newer")
//public class Auto_5_0_Pushing_newer extends OpMode {
//    private Follower follower;
//    private subsystems.Intake_DiffyClaw intakeDiffyClaw;
//    private OuttakeLiftSubsys outtakeLift;
//    private Outtake outtake;
//    private Timer pathTimer;
//    private final double scoreX = 39.3;
//    private final double scoreY = 78;
//
//    private final Pose startPose = new Pose(7.1, 65.5, Math.toRadians(180));
//    private final Pose preloadScoreControl = new Pose(20, scoreY, Math.toRadians(180));
//    private final Pose preloadScorePose = new Pose(scoreX, scoreY, Math.toRadians(180));
//
//
//
//
//    private final Pose[][] samplePushPoses = {
//            {
//                    preloadScorePose,
//                    new Pose(27.66, 38.5, Math.toRadians(180)),
//                    new Pose(25.4, 19.8, Math.toRadians(180)),
//                    new Pose(58.94, 54.17, Math.toRadians(180)),
//                    new Pose(65.80132450331126, 29.75364238410596, Math.toRadians(180)),
//                    new Pose(68.09006622516556, 15.067549668874179, Math.toRadians(180)),
//                    new Pose(51.30596026490066, 24.985430463576165, Math.toRadians(180)),
//                    new Pose(18.5, 22.5, Math.toRadians(180)),
//            },
//            {
//                    new Pose(18.5, 22.5, Math.toRadians(180)),
//                    new Pose(69.61589403973511, 25.939072847682112, Math.toRadians(180)),
//                    new Pose(65.0384105960265, 16.784105960264895, Math.toRadians(180)),
//                    new Pose(60.46092715231788, 9.536423841059603, Math.toRadians(180)),
//                    new Pose(18.5, 12.0158940397351, Math.toRadians(180)),
//            },
//            {
//                    new Pose(18.5, 12.0158940397351, Math.toRadians(180)),
//                    new Pose(72.28609271523179, 14.87682119205298, Math.toRadians(180)),
//                    new Pose(64.08476821192053, 10.108609271523186, Math.toRadians(180)),
//                    new Pose(61.98675496688742, 4.386754966887411, Math.toRadians(180)),
//                    new Pose(31.47019867549669, 7.629139072847689, Math.toRadians(180)),
//                    new Pose(10.5, 6.675496688741724, Math.toRadians(180)),
//            }
//    };
//
//    private final Pose outtakePickupWaitPose = new Pose(14.5, 32, Math.toRadians(180));
//    private final Pose outtakePickupPose = new Pose(10.5, 32, Math.toRadians(180));
//    private final Pose outtakePickupControl1 = new Pose(20, 32, Math.toRadians(180));
//    private final Pose outtakePickupControl2 = new Pose(25, 32, Math.toRadians(180));
//
//    private final double scoreYChange = 1.7;
//    private final List<Pose[]> scorePoses = new ArrayList<>();
//    private final Pose parkPose = new Pose(18, 30, Math.toRadians(230));
//    private final double pushPathEndTimeout = 0;
//    private final double pickupWaitTimeout = 0;
//    private final double zeroPowerAccelerationMultiplierForPickupLastTwo = 0.43, zeroPowerAccelerationMultiplierForPickupFirst = 0.39, zeroPowerAccelerationMultiplierForPush = 4.5, zeroPowerAccelerationMultiplierForPushWAIT = 1 ,zeroPowerAccelerationMultiplerForScore = 4;
//
//    private PathChain scorePreloadPath, parkFromFifthPath;
//    private PathChain goToFirstPush, pushFirstSample, goToSecondPush, pushSecondSample, goToThirdPush, pushThirdSample, firstPickupPath, secondPickupPath, thirdPickupPath, fourthPickupPath, fifthPickupPath, firstScorePath, secondScorePath,thirdScorePath, fourthScorePath, fifthScorePath;
//    private PathChain wallPickup;
//    private List<PathChain> pushPaths, scorePaths, pickupPaths;
//    public void buildPaths(){
//        for(int i=1; i<=4; i++){
//            double newY = scoreY - scoreYChange*i;
//            scorePoses.add(
//                    new Pose[]{
//                            outtakePickupPose,
//                            new Pose(scoreX - 20, newY, Math.toRadians(180)),
//                            new Pose(scoreX - 10, newY, Math.toRadians(180)),
//                            new Pose(scoreX, newY, Math.toRadians(180))
//                    }
//            );
//        }
//
//        scorePreloadPath = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(startPose), new Point(preloadScoreControl), new Point(preloadScorePose)))
//                .setLinearHeadingInterpolation(startPose.getHeading(), preloadScorePose.getHeading())
//                .setZeroPowerAccelerationMultiplier(1.3)
//                .build();
//        pushPaths = Arrays.stream(samplePushPoses).map(
//                e ->
//                follower.pathBuilder()
//                        .addPath(new BezierCurve(e))
//                        .setLinearHeadingInterpolation(e[0].getHeading(), e[e.length - 1].getHeading())
//                        .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplierForPushWAIT)
//                        .setPathEndTimeoutConstraint(pushPathEndTimeout)
//                        .build()
//        ).collect(Collectors.toList());
//        scorePaths = scorePoses.stream().map(
//                e ->
//                        follower.pathBuilder()
//                                .addPath(new BezierCurve(e))
//                                .setLinearHeadingInterpolation(e[0].getHeading(), e[e.length-1].getHeading())
//                                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplerForScore)
//                                .setPathEndTimeoutConstraint(pushPathEndTimeout)
//                                .build()
//        ).collect(Collectors.toList());
//    }
//
//    enum AutoState {
//        DRIVE_TO_PRELOAD_SCORE,
//        SCORE_PRELOAD,
//        DRIVE_TO_PUSHING,
//        BEFORE_PUSHING,
//        READY_FOR_PUSHING,
//        PUSHING,
//        READY_FOR_PICKUP,
//        WALL_PICKUP,
//        WALL_DELAY,
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
//            case DRIVE_TO_PRELOAD_SCORE:
//                outtake.setClawState(Outtake.ClawStates.CLOSED);
//                outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCORE);
//                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.BACK_SCORE);
//                if(pathTimer.getElapsedTimeSeconds() > 0){
//                    follower.followPath(scorePreloadPath,true);
//                    setPathState(AutoState.SCORE_PRELOAD);
//                }
//                break;
//            case SCORE_PRELOAD:
//                if(!follower.isBusy()){
//                    outtake.setClawState(Outtake.ClawStates.OPEN);
//                    setPathState(AutoState.DRIVE_TO_PUSHING);
//                }
//                break;
//            case DRIVE_TO_PUSHING:
//                if(pathTimer.getElapsedTimeSeconds() > 0){
//                    follower.followPath(pushWaitPaths[counter], false);
//                    counter = 0;
//                    setPathState(AutoState.BEFORE_PUSHING);
//                }
//                break;
//            case BEFORE_PUSHING:
//                if(pathTimer.getElapsedTimeSeconds() > 0){
//                    setPathState(AutoState.READY_FOR_PUSHING);
//                }
//                break;
//            case READY_FOR_PUSHING:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4){
//                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
//                    outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
//                }
//                if(!follower.isBusy()){
//                    follower.followPath(pushDonePaths[counter], false);
//                    setPathState(AutoState.PUSHING);
//                }
//                break;
//            case PUSHING:
//                if(!follower.isBusy()){
//                    counter++;
//                    if(counter < 3){
//                        follower.followPath(pushWaitPaths[counter]);
//                        setPathState(AutoState.READY_FOR_PUSHING);
//                    } else {
//                        counter = 0;
//                        setPathState(AutoState.READY_FOR_PICKUP); // Skips WALL_PICKUP when first pickup
//                        outtake.setClawState(Outtake.ClawStates.OPEN);
//                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
//                        follower.followPath(pickupPaths[counter], true);
//                    }
//                }
//                break;
//            case WALL_PICKUP:
//                if (pathTimer.getElapsedTimeSeconds() > 0.55){
//                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
//                    outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
//                }
//                if (!follower.isBusy()){
//                    setPathState(AutoState.WALL_DELAY);
//                }
//                break;
//            case WALL_DELAY:
//                if (pathTimer.getElapsedTimeSeconds() > 0.0) {
//                    follower.followPath(wallPickup, true);
//                    setPathState(AutoState.READY_FOR_PICKUP);
//                }
//                break;
//            case READY_FOR_PICKUP:
//                outtake.setClawState(Outtake.ClawStates.OPEN);
//                if(!follower.isBusy()){
//                    setPathState(AutoState.PICKUP);
//                }
//                break;
//            case PICKUP:
//                if(pathTimer.getElapsedTimeSeconds() > 0){
//                    outtake.setClawState(Outtake.ClawStates.CLOSED);
//                    if(pathTimer.getElapsedTimeSeconds() > 0.28){
//                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.BACK_SCORE);
//                        outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCORE);
//                        follower.followPath(scorePaths[counter], false);
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
//                if(pathTimer.getElapsedTimeSeconds() > 0.04){
//                    outtake.setClawState(Outtake.ClawStates.OPEN);
//                    if (pathTimer.getElapsedTimeSeconds() > 0.26) {
//                        counter++;
//                        if (counter < 4) {
//                            follower.followPath(pickupPaths[counter], false);
//                            setPathState(AutoState.WALL_PICKUP);
//                        } else {
//                            follower.followPath(parkFromFifthPath, false);
//                            setPathState(AutoState.PARK);
//                        }
//                    }
//                }
//                break;
//            case PARK:
//                if (pathTimer.getElapsedTimeSeconds() > 0.6){
//                    outtake.setOuttakeState(Outtake.OuttakeState.RESET_ENCODER);
//                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.RESET_ENCODER);
//                }
//                break;
//            default:
//                break;
//        }
//    }
//    @Override
//    public void init(){
//        pathTimer = new Timer();
//        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
//        follower.setStartingPose(startPose);
//
//        tel = new UnifiedTelemetry();
//        tel.init(this.telemetry);
//        SubsysCore.setGlobalParameters(hardwareMap, this);
//
//        intakeDiffyClaw = new subsystems.Intake_DiffyClaw();
//        intakeDiffyClaw.init();
//        outtake = new Outtake(hardwareMap);
//        outtakeLift = new OuttakeLiftSubsys();
//        outtakeLift.init();
//        buildPaths();
//    }
//
//    private final ToggleButton teamColorButton = new ToggleButton(Storage.isRed);
//    private UnifiedTelemetry tel;
//    @Override
//    public void init_loop(){
//        teamColorButton.input(gamepad1.dpad_up);
//        Storage.isRed = teamColorButton.getVal();
//        outtake.setOuttakeState(Outtake.OuttakeState.Auto_Wait);
//        outtake.outtakeLoop();
//        outtake.setClawState(Outtake.ClawStates.CLOSED);
//        tel.addData("Team Color:", Storage.isRed ? "Red" : "Blue");
//        tel.update();
//    }
//    @Override
//    public void loop(){
//        follower.update();
//        autonomousPathUpdate();
//        outtake.outtakeLoop();
//        outtakeLift.holdLift();
//        outtakeLift.loop();
//        intakeDiffyClaw.loop();
//
//
//        Storage.CurrentPose = follower.getPose();
//        telemetry.update();
//    }
//}