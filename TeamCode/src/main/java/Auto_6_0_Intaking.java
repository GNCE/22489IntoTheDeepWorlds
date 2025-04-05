import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import subsystems.IntakeLimelightSubsys;
import subsystems.OuttakeLiftSubsys;
import subsystems.SubsysCore;
import subsystems.UnifiedTelemetry;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;


@Autonomous (name = "6+0 Intaking")
public class Auto_6_0_Intaking extends OpMode {
    private Follower follower;
    private Intake_DiffyClaw intakeDiffyClaw;
    private IntakeLimelightSubsys ll;
    private OuttakeLiftSubsys outtakeLift;
    private Outtake outtake;
    private Timer pathTimer;
    private ElapsedTime timeSpentInSub;
    private final double backScoreX = 39;
    private final double frontScoreX = 38;
    private double firstPickupX = 48, firstPickupY = 72;
    private double secondPickupX = 48, secondPickupY = 72;

    private double visionCounter = 0;



    // TODO: Find correct starting pose
    // Static
    private final Pose startPose = new Pose(8.145, 65.45, Math.toRadians(0));
    private final Pose pickupPose = new Pose(10.5, 35, Math.toRadians(180));
    private final Pose sample3Pose = new Pose(20, 12.5, Math.toRadians(-28));
    private final Pose sample2Pose = new Pose(20, 12.5, Math.toRadians(0));
    private final Pose sample1Pose = new Pose(20, 23, Math.toRadians(0));
    private final Pose firstPickupPose = new Pose(12, 23, Math.toRadians(0));

    private PathChain samp3to2, samp2to1, samp1toFirstPickup;

    /**
     * @param lowerBound Lower bound Y of specimen scoring
     * @param upperBound Upper bound Y of specimen scoring
     * @param preload Preload Y location
     * @param diff Amount of Y gap between each specimen
     * @param num Number of non-preload specimens
     * @return Y location of the specimens in reverse order
     */
    public static List<Double> findClosestNumbers(double lowerBound, double upperBound, double preload, double diff, double num) {
        List<Double> numbers = new ArrayList<>();
        for(double i = lowerBound; i <= preload-diff && numbers.size() < num; i+=diff) numbers.add(i);
        for(double i = preload+diff; i <= upperBound && numbers.size() < num; i+=diff) numbers.add(i);
        numbers.sort(Comparator.reverseOrder());
        return numbers;
    }

    public void buildStaticPaths(){
        samp3to2 = follower.pathBuilder()
                .addBezierCurve(new Point(sample3Pose), new Point(sample2Pose))
                .setLinearHeadingInterpolation(sample3Pose.getHeading(), sample2Pose.getHeading())
                .build();
        samp2to1 = follower.pathBuilder()
                .addBezierCurve(new Point(sample2Pose), new Point(sample1Pose))
                .setLinearHeadingInterpolation(sample2Pose.getHeading(), sample1Pose.getHeading())
                .build();
        samp1toFirstPickup = follower.pathBuilder()
                .addBezierLine(new Point(sample1Pose), new Point(firstPickupPose))
                .setLinearHeadingInterpolation(sample1Pose.getHeading(), firstPickupPose.getHeading())
                .build();
    }

    private AutoState autoState = AutoState.DRIVE_TO_PRELOAD_SCORE;

    public void setPathState(AutoState newState){
        autoState = newState;
        pathTimer.resetTimer();
    }

    private int counter = 0;

    enum AutoState {
        DRIVE_TO_PRELOAD_SCORE, READY_FOR_PRELOAD, PRELOAD_DRIVE_DONE, PRELOAD_DETECTED_SAMPLE, PRELOAD_DONE_ALIGNING, DETECT_EXTEND, DETECT_RETRACT, PRELOAD_FAILED_ALIGNING, DRIVE_TO_SPIKE_MARK_3, AT_SPIKE_MARK_3
    }
    int attemptedNext = 0;
    public void autonomousPathUpdate(){
        switch (autoState){
            case DRIVE_TO_PRELOAD_SCORE:
                outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTSCORE);
                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_SCORE_WAIT);
                follower.followPath(follower.pathBuilder()
                        .addBezierCurve(new Point(startPose), new Point(frontScoreX, firstPickupY))
                        .setConstantHeadingInterpolation(startPose.getHeading())
                        .setZeroPowerAccelerationMultiplier(1.5)
                        .build(),true);
                ll.turnOn();
                intakeDiffyClaw.ExtendTo(firstPickupX - 48, Intake_DiffyClaw.ExtensionUnits.inches);
                setPathState(AutoState.READY_FOR_PRELOAD);
                break;
            case READY_FOR_PRELOAD:
                if(outtakeLift.getCurrentPosition() > 300){
                    intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                    intakeDiffyClaw.setClawOpen(true);
                    ll.turnOn();
                    intakeDiffyClaw.useVision();
                    setPathState(AutoState.PRELOAD_DRIVE_DONE);
                }
                break;
            case PRELOAD_DRIVE_DONE:
                if(!follower.isBusy() && intakeDiffyClaw.extensionReachedTarget()){
                    if(ll.isResultValid()){
                        setPathState(AutoState.PRELOAD_DETECTED_SAMPLE);
                    } else {
                        setPathState(AutoState.DETECT_EXTEND);
                        counter = 0;
                    }
                }
                break;
            case DETECT_EXTEND: // TODO: Wrap in !follower.isbusy?
                intakeDiffyClaw.stopVision();
                intakeDiffyClaw.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.FULL_EXTENSION);
                intakeDiffyClaw.setPowerScale(0.2);
                if(ll.isResultValid()){
                    attemptedNext++;
                    setPathState(AutoState.PRELOAD_DETECTED_SAMPLE);
                }
                if(intakeDiffyClaw.getCurrentPosition() > 300&& !ll.isResultValid()){
                    follower.followPath(
                            follower.pathBuilder()
                                    .addBezierLine(new Point(follower.getPose()), new Point(frontScoreX, follower.getPose().getY()+0.5))
                                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), 0)
                                    .build(),
                            true
                    );
                    setPathState(AutoState.DETECT_RETRACT);
                }
                break;
            case DETECT_RETRACT:
                intakeDiffyClaw.stopVision();
                intakeDiffyClaw.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                intakeDiffyClaw.setPowerScale(0.5);
                if(ll.isResultValid()){
                    attemptedNext++;
                    setPathState(AutoState.PRELOAD_DETECTED_SAMPLE);
                }
                if(intakeDiffyClaw.getCurrentPosition() < 30 && !ll.isResultValid()){
                    counter++; // TODO: Counter UNUSED. ADD TIME OR COUNTER LIMIT
                    follower.followPath(
                            follower.pathBuilder()
                                    .addBezierLine(new Point(follower.getPose()), new Point(frontScoreX, follower.getPose().getY()+5))
                                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), 0)
                                    .build(),
                            true
                    );
                    setPathState(AutoState.DETECT_EXTEND);
                }
                break;
            case PRELOAD_DETECTED_SAMPLE: // CALL WHEN LL DETECT
                if(ll.isResultValid()){
                    visionCounter++;
                    intakeDiffyClaw.setPowerScale(1);
                    intakeDiffyClaw.useVision();
                    double angle = ll.getAngle(); // Output 0 is sample angle
                    if(Math.abs(angle) > 85){
                        if(Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED >= 0) angle = 85;
                        else angle = -85;
                    }
                    if(angle < -90) angle = -90;
                    else if(angle > 90) angle = 90;

                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = angle * 10.5/9;

                    follower.holdPoint(new Pose(frontScoreX, follower.getPose().getY() - ll.getTy()*0.0012, 0));
                    if(Math.abs(ll.getTx() -16) < 1.5 && Math.abs(ll.getTy()) < 1.5 || pathTimer.getElapsedTimeSeconds() > 5){
                        setPathState(AutoState.PRELOAD_DONE_ALIGNING);
                    }
                } else if (pathTimer.getElapsedTimeSeconds() > 5){
                    setPathState(AutoState.PRELOAD_DONE_ALIGNING);
                }
                break;
            case PRELOAD_DONE_ALIGNING:
                firstPickupY = follower.getPose().getY();
                follower.holdPoint(new Pose(frontScoreX, firstPickupY, 0));
                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_SCORE_DONE);
                intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
                ll.turnOff();
                if (pathTimer.getElapsedTimeSeconds() > 0.7){
                    intakeDiffyClaw.setClawOpen(false);
                    outtake.setClawOpen(true);
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.6){
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
                    intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER_WAIT);
                }
                if(pathTimer.getElapsedTimeSeconds() > 3){
                    setPathState(AutoState.DRIVE_TO_SPIKE_MARK_3);
                }
                break;
            case PRELOAD_FAILED_ALIGNING: // TODO: CURRENTLY UNUSED I THINK
                firstPickupY = follower.getPose().getY();
                follower.holdPoint(new Pose(frontScoreX, firstPickupY, 0));
                intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                intakeDiffyClaw.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_SCORE_DONE);
                if(pathTimer.getElapsedTime() > 0.2){
                    outtake.setClawOpen(true);
                }
                if(pathTimer.getElapsedTime() > 0.3){
                    setPathState(AutoState.DRIVE_TO_SPIKE_MARK_3);
                }
                break;
            case DRIVE_TO_SPIKE_MARK_3:
                follower.followPath(
                        follower.pathBuilder()
                                .addBezierCurve(new Point(follower.getPose()), new Point(sample3Pose))
                                .setLinearHeadingInterpolation(follower.getPose().getHeading(), sample3Pose.getHeading())
                                .addParametricCallback(0, ()-> {
                                    ll.turnOff();
                                })
                                .addParametricCallback(0.5, () -> {
                                    intakeDiffyClaw.ExtendTo(330, Intake_DiffyClaw.ExtensionUnits.ticks);
                                })
                                .build(),
                        false
                );
                setPathState(AutoState.AT_SPIKE_MARK_3);
                break;
            case AT_SPIKE_MARK_3:
                if (!follower.isBusy()){
                    if(ll.isResultValid()){
                        // If valid, align for a little bit. If not, just hope for the best and pickup right away.
                    } else {

                    }
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

        ll = new IntakeLimelightSubsys();
        ll.init();
        ll.setPipelineNumber(Storage.isRed ? 6 : 5);
        intakeDiffyClaw = new Intake_DiffyClaw();
        intakeDiffyClaw.init();
        outtake = new Outtake(hardwareMap);
        outtakeLift = new OuttakeLiftSubsys();
        outtakeLift.init();
        buildStaticPaths();
    }

    private final ToggleButton teamColorButton = new ToggleButton(Storage.isRed);
    private UnifiedTelemetry tel;
    @Override
    public void init_loop(){
        teamColorButton.input(gamepad1.dpad_up);
        Storage.isRed = teamColorButton.getVal();
        outtake.setOuttakeState(Outtake.OuttakeState.Auto_Wait);
        outtake.setClawOpen(false);

        double translatedX = gamepad1.touchpad_finger_1_y * 12 + 60; // Range is 48 to 72
        double translatedY = (-gamepad1.touchpad_finger_1_x  * 12.2/2) + 72; // Range is 55.6 to 88.4

        if(gamepad1.x){
            firstPickupX = translatedX;
            firstPickupY = translatedY;
        }


        outtake.outtakeLoop();

        tel.addData("Team Color:", Storage.isRed ? "Red" : "Blue");

        tel.addLine("Pre-defined Positions:");
        tel.addData("Current Touchpad", "(%.3f, %.3f)", translatedY, translatedX);
        tel.addData("1st Pickup", "(%.3f, %.3f)", firstPickupY, firstPickupX);
        tel.update();
    }


    @Override
    public void loop(){
        follower.update();
        autonomousPathUpdate();
        ll.loop();
        outtake.outtakeLoop();
        outtakeLift.holdLift();
        outtakeLift.loop();
        intakeDiffyClaw.loop();
        intakeDiffyClaw.HoldExtension();


        Storage.CurrentPose = follower.getPose();
        tel.addData("Auto State", autoState.name());
        tel.addData("Vision Counter", visionCounter);
        tel.addData("Attempted Alignment", attemptedNext);
        tel.update();
    }
}