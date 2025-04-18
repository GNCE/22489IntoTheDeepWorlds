import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import subsystems.IntakeLimelightSubsys;
import subsystems.OuttakeLiftSubsys;
import subsystems.SubsysCore;
import subsystems.UnifiedTelemetry;


@Disabled
@Autonomous (name = "INTAKING TEST AUTO")
public class Auto_Horizontal_Slide_Correction_Intake_Test extends OpMode {
    private Follower follower;
    private Intake_DiffyClaw intakeDiffyClaw;
    private IntakeLimelightSubsys ll;
    private OuttakeLiftSubsys outtakeLift;
    private Outtake outtake;
    private Timer pathTimer;
    private ElapsedTime timeSpentInSub;
    private final double backScoreX = 39;
    private final double frontScoreX = 37;
    private double firstPickupX = 48, firstPickupY = 72;
    private double secondPickupX = 48, secondPickupY = 72;

    // TODO: Find correct starting pose
    // Static
    private final Pose startPose = new Pose(6.495, 65.45, Math.toRadians(0));
    private final Pose pickupPose = new Pose(10.5, 35, Math.toRadians(180));

    private AutoState autoState = AutoState.DRIVE_TO_PRELOAD_SCORE;

    public void setPathState(AutoState newState){
        autoState = newState;
        pathTimer.resetTimer();
    }

    private int counter = 0;

    enum AutoState {
        DRIVE_TO_PRELOAD_SCORE, READY_FOR_PRELOAD, PRELOAD_DRIVE_DONE, PRELOAD_DETECTED_SAMPLE, PRELOAD_DONE_ALIGNING, DETECT_EXTEND, DETECT_RETRACT, PRELOAD_FAILED_ALIGNING, DRIVE_TO_SPIKE_MARK_3
    }
    public void autonomousPathUpdate(){
        switch (autoState){
            case DRIVE_TO_PRELOAD_SCORE:
                outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTSCORE);
                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_SCORE_WAIT);
                if(scorePreloadPath != null){
                    follower.followPath(scorePreloadPath,false);
                    ll.turnOn();
                    intakeDiffyClaw.ExtendTo(firstPickupX - 48, Intake_DiffyClaw.ExtensionUnits.inches);
                    setPathState(AutoState.READY_FOR_PRELOAD);
                }
                break;
            case READY_FOR_PRELOAD:
                if(outtakeLift.getCurrentPosition() > 300){
                    intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                    intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                    ll.turnOn();
                    follower.startTeleopDrive();
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
                intakeDiffyClaw.setPowerScale(0.3);
                if(ll.isResultValid()){
                    setPathState(AutoState.PRELOAD_DETECTED_SAMPLE);
                }
                if(intakeDiffyClaw.extensionReachedTarget()){
                    follower.followPath(
                            follower.pathBuilder()
                                    .addBezierLine(new Point(follower.getPose()), new Point(follower.getPose().getX(), follower.getPose().getX()+5))
                                    .setConstantHeadingInterpolation(follower.getPose().getHeading())
                                    .build(),
                            false
                    );
                    setPathState(AutoState.DETECT_RETRACT);
                }
                break;
            case DETECT_RETRACT:
                intakeDiffyClaw.stopVision();
                intakeDiffyClaw.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                intakeDiffyClaw.setPowerScale(0.3);
                if(ll.isResultValid()){
                    setPathState(AutoState.PRELOAD_DETECTED_SAMPLE);
                }
                if(intakeDiffyClaw.extensionReachedTarget()){
                    counter++; // TODO: Counter UNUSED. ADD TIME OR COUNTER LIMIT
                    follower.followPath(
                            follower.pathBuilder()
                                    .addBezierLine(new Point(follower.getPose()), new Point(follower.getPose().getX(), follower.getPose().getX()+5))
                                    .setConstantHeadingInterpolation(follower.getPose().getHeading())
                                    .build(),
                            false
                    );

                    setPathState(AutoState.DETECT_EXTEND);
                }
                break;
            case PRELOAD_DETECTED_SAMPLE: // CALL WHEN LL DETECT
                if(ll.isResultValid()){
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

                    follower.setTeleOpMovementVectors((frontScoreX - follower.getPose().getX()) * 0.01, ll.getTy()*0.01, 0.05 * -follower.getPose().getHeading());
                    if(Math.abs(ll.getTx() -16) < 1.5 && Math.abs(ll.getTy()) < 1.5 || pathTimer.getElapsedTime() > 2){
                        follower.setTeleOpMovementVectors(0, 0, 0);
                        setPathState(AutoState.PRELOAD_DONE_ALIGNING);
                    }
                } else if (pathTimer.getElapsedTime() > 2){
                    follower.setTeleOpMovementVectors(0, 0, 0);
                    setPathState(AutoState.PRELOAD_DONE_ALIGNING);
                }
                break;
            case PRELOAD_DONE_ALIGNING:
                firstPickupY = follower.getPose().getY();
                follower.holdPoint(new Pose(frontScoreX, firstPickupY, 0));
                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_SCORE_DONE);
                intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
                ll.turnOff();
                if (pathTimer.getElapsedTime() > 0.2){
                    intakeDiffyClaw.setClawState(Intake_DiffyClaw.CLAW_STATE.CLOSED);
                    outtake.setClawOpen(true);
                }
                if (pathTimer.getElapsedTime() > 0.4){
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
                    intakeDiffyClaw.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER_WAIT);
                }
                if(pathTimer.getElapsedTime() > 0.5){
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
        ll = new IntakeLimelightSubsys();
        ll.init();
        ll.setPipelineNumber(Storage.isRed ? 6 : 5);
        outtake = new Outtake(hardwareMap);
        outtakeLift = new OuttakeLiftSubsys();
        outtakeLift.init();
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
        double translatedY = gamepad1.touchpad_finger_1_x  * 16.4 + 72; // Range is 55.6 to 88.4

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


    PathChain scorePreloadPath;
    private void createPreloadPath(){
        scorePreloadPath = follower.pathBuilder()
                .addBezierCurve(new Point(startPose), new Point(firstPickupX, firstPickupY))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
    }

    @Override
    public void start(){
        createPreloadPath();
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