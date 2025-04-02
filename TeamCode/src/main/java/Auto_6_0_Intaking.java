//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//import subsystems.OuttakeLiftSubsys;
//import subsystems.SubsysCore;
//import subsystems.UnifiedTelemetry;
//import java.util.ArrayList;
//import java.util.Comparator;
//import java.util.List;
//
//
//@Autonomous (name = "6+0 Intaking")
//public class Auto_6_0_Intaking extends OpMode {
//    private Follower follower;
//    private Intake_DiffyClaw intakeDiffyClaw;
//    private OuttakeLiftSubsys outtakeLift;
//    private Outtake outtake;
//    private Timer pathTimer;
//    private final double scoreX = 39;
//
//    // TODO: Find correct starting pose
//    // Static
//    private final Pose startPose = new Pose(6.495, 65.45, Math.toRadians(0));
//    private final Pose pickupPose = new Pose(10.5, 35, Math.toRadians(180));
//    private final Pose midPose = new Pose()
//    private final Pose sample3Pose = new Pose(20, 12.5, Math.toRadians(-28));
//    private final Pose sample2Pose = new Pose(20, 12.5, Math.toRadians(0));
//    private final Pose sample1Pose = new Pose(20, 23, Math.toRadians(0));
//    private final Pose firstPickupPose = new Pose(12, 23, Math.toRadians(0));
//
//    private PathChain samp3to2, samp2to1, samp1toFirstPickup;
//
//    /**
//     * @param lowerBound Lower bound Y of specimen scoring
//     * @param upperBound Upper bound Y of specimen scoring
//     * @param preload Preload Y location
//     * @param diff Amount of Y gap between each specimen
//     * @param num Number of non-preload specimens
//     * @return Y location of the specimens in reverse order
//     */
//    public static List<Double> findClosestNumbers(double lowerBound, double upperBound, double preload, double diff, double num) {
//        List<Double> numbers = new ArrayList<>();
//        for(double i = lowerBound; i <= preload-diff && numbers.size() < num; i+=diff) numbers.add(i);
//        for(double i = preload+diff; i <= upperBound && numbers.size() < num; i+=diff) numbers.add(i);
//        numbers.sort(Comparator.reverseOrder());
//        return numbers;
//    }
//
//    public void buildStaticPaths(){
//        samp3to2 = follower.pathBuilder()
//                .addBezierCurve(new Point(sample3Pose), new Point(sample2Pose))
//                .build();
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
//                outtake.setClawOpen(false);
//                outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCORE);
//                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.BACK_SCORE);
//                if(pathTimer.getElapsedTimeSeconds() > 0){
//                    follower.followPath(scorePreloadPath,false);
//                    setPathState(AutoState.SCORE_PRELOAD);
//                }
//                break;
//            case SCORE_PRELOAD:
//                if(!follower.isBusy()){
//                    outtake.setClawOpen(true);
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
//                    outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
//                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
//                    setPathState(AutoState.READY_FOR_PUSHING);
//                }
//                break;
//            case READY_FOR_PUSHING:
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
//                        outtake.setClawOpen(true);
//                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
//                        follower.followPath(pickupPaths[counter], false);
//                    }
//                }
//                break;
//            case WALL_PICKUP:
//                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
//                if (!follower.isBusy()){
//                    follower.followPath(wallPickup, true);
//                    setPathState(AutoState.READY_FOR_PICKUP);
//                }
//                break;
//            case READY_FOR_PICKUP:
//                outtake.setClawOpen(true);
//                if(!follower.isBusy()){
//                    setPathState(AutoState.PICKUP);
//                }
//                break;
//            case PICKUP:
//                if(pathTimer.getElapsedTimeSeconds() > 0){
//                    outtake.setClawOpen(false);
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
//                    outtake.setClawOpen(true);
//                    if (pathTimer.getElapsedTimeSeconds() > 0.26) {
//                        counter++;
//                        if (counter < 4) {
//                            follower.followPath(pickupPaths[counter], false);
//                            outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
//                            outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
//                            setPathState(AutoState.WALL_PICKUP);
//                        } else {
//                            follower.followPath(parkFromFifthPath, false);
//                            outtake.setOuttakeState(Outtake.OuttakeState.RESET_ENCODER);
//                            outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.RESET_ENCODER);
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
//        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
//        follower.setStartingPose(startPose);
//
//        tel = new UnifiedTelemetry();
//        tel.init(this.telemetry);
//        SubsysCore.setGlobalParameters(hardwareMap, this);
//
//        intakeDiffyClaw = new Intake_DiffyClaw();
//        intakeDiffyClaw.init();
//        outtake = new Outtake(hardwareMap);
//        outtakeLift = new OuttakeLiftSubsys();
//        outtakeLift.init();
//        buildStaticPaths();
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
//        outtake.setClawOpen(false);
//        tel.addData("Team Color:", Storage.isRed ? "Red" : "Blue");
//        tel.addData("Preload Score Y", preloadScoreY);
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