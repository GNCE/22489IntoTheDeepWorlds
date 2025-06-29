

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.FConstants_6_0;
import pedroPathing.constants.FConstants_Teleop_Autoscore;
import pedroPathing.constants.LConstants;
import subsystems.DriveSubsys;
import subsystems.HangServoSubsys;
import subsystems.IntakeLimelightSubsys;
import subsystems.Intake_DiffyClaw;
import subsystems.LynxModules;
import subsystems.Outtake;
import subsystems.OuttakeLiftSubsys;
import subsystems.SubsysCore;
import subsystems.UnifiedTelemetry;
import utils.MedianSmoother;
import utils.Storage;


@TeleOp(name = "Main TeleOp", group = "_TeleOp")
@Config
public class Main_TeleOp extends OpMode {
    public static double mx =  -0.008, my =  -0.021;
    public static double targetX = 16, targetY = 0;
    private Follower follower;
    public static double hangDelay = 1.4; //for safety cuz at 11.9 volts this is perfect.
    private LynxModules lynxModules;
    private MedianSmoother medianSmoother;
    private Intake_DiffyClaw diffyClawIntake;
    private DriveSubsys driveSubsys;
    private IntakeLimelightSubsys ll;
    private OuttakeLiftSubsys outtakeLift;
    private Outtake outtake;
    private HangServoSubsys hangServos;
    private ElapsedTime elapsedTime, intakeSequenceTime, resetEncoderDelay, outtakeSequenceTime, hangTimer, loopTime;
    private final Pose startPose = Storage.CurrentPose;
    private double targetHeading = 180, headingError, headingCorrection;
    int flip = 1;
    int initfsm = 0;
    public static String key = "sigmaboy";
    private UnifiedTelemetry tel;

    ToggleButton takeSnapshotButton = new ToggleButton(false);


    @Override
    public void init() {

        follower = new Follower(hardwareMap, FConstants_Teleop_Autoscore.class, LConstants.class);
        follower.setStartingPose(startPose);
        elapsedTime = new ElapsedTime();
        intakeSequenceTime = new ElapsedTime();
        resetEncoderDelay = new ElapsedTime();
        outtakeSequenceTime = new ElapsedTime();
        avoidIntakeFsmTimer = new ElapsedTime();
        hangTimer = new ElapsedTime();
        loopTime = new ElapsedTime();
        autoScoreTimer = new ElapsedTime();
        tel = new UnifiedTelemetry();
        tel.init(this.telemetry);
        SubsysCore.setGlobalParameters(hardwareMap, this);

        lowBasketToggleButton.setVal(false);


        lynxModules = new LynxModules();
        lynxModules.init();
        driveSubsys = new DriveSubsys();
        driveSubsys.init();
        diffyClawIntake = new Intake_DiffyClaw();
        ll = new IntakeLimelightSubsys();
        ll.init();
        outtakeLift = new OuttakeLiftSubsys();
        outtakeLift.init();
        outtake = new Outtake();
        outtake.init();
        hangServos = new HangServoSubsys();
        hangServos.init();
        medianSmoother = new MedianSmoother(2000);

        intakeSequenceTime.startTime();
        elapsedTime.startTime();
        resetEncoderDelay.startTime();
        outtakeSequenceTime.startTime();
        avoidIntakeFsmTimer.startTime();
        hangTimer.startTime();
        autoScoreTimer.startTime();
        initfsm = 1;
        autoScore = AUTO_SCORE.NOTHING;
        buildAutoScorePaths();
        follower.startTeleopDrive();
    }

    private ToggleButton teamColorButton = new ToggleButton(Storage.isRed);
    private ToggleButton controlFlipButton = new ToggleButton(true);

    @Override
    public void init_loop(){
        lynxModules.loop();
        teamColorButton.input(gamepad1.dpad_up);
        Storage.isRed = teamColorButton.getVal();
        specModeToggleButton.input(gamepad2.right_bumper);
        Storage.specMode = specModeToggleButton.getVal();

        follower.setTeleOpMovementVectors(
                flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_y),
                flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_x),
                0.225 * Math.tan(1.12 * -gamepad1.right_stick_x), true);


        follower.update();
        Storage.CurrentPose = follower.getPose();
        telemetry.addLine("DO NOT TOUCH IF THIS IS REAL GAME, or make sure you dont misclick.");
        telemetry.addData("Team Color:", Storage.isRed ? "Red" : "Blue");
        telemetry.addData("Mode", Storage.specMode ? "Specimen" : "Sample");
        telemetry.update();
    }

    @Override
    public void start() {
        diffyClawIntake.init();
        loopTime.startTime();
    }

    // Forward (not bucket sequence): RETRACT > READY > GRAB > RETRACT
    // Forward (bucket sequence): RETRACT > TRANSFER (when isTransferred is false) > (automatically) RETRACT >
    // RETRACT < READY < GRAB
    public enum INTAKE_SEQUENCE{
        READY, GRAB, HOLD, RETRACT, VISION, VISION_MOVE, VISION_GO, TRANSFER_WAIT;
        private static final INTAKE_SEQUENCE[] vals = values();
        public INTAKE_SEQUENCE next(){
            return vals[(this.ordinal() + 1) % vals.length];
        }
        public INTAKE_SEQUENCE prev(){
            return vals[(this.ordinal() - 1 + vals.length) % vals.length];
        }
    }
    boolean isTransferred = true;
    INTAKE_SEQUENCE intakeSequence = INTAKE_SEQUENCE.RETRACT;
    private ToggleButton intakeSequenceNextButton = new ToggleButton(true), intakeSequencePreviousButton = new ToggleButton(true), ALignmentButtonNext = new ToggleButton(true),ALignmentButtonPrev = new ToggleButton(true), autoALignmentButton = new ToggleButton(true);
    //outtake stuff
    public enum BUCKET_SEQUENCE{
        TRANSFER, GRAB_AND_LIFT, SCORE;
        private static final BUCKET_SEQUENCE[] vals = values();
        public BUCKET_SEQUENCE next(){
            return vals[(this.ordinal() + 1) % vals.length];
        }
        public BUCKET_SEQUENCE prev(){
            return vals[(this.ordinal() - 1 + vals.length) % vals.length];
        }
    }

    public enum BACK_SPECIMEN_SEQUENCE {
        OPEN_CLAW, FRONT_GRAB, CLOSE_CLAW, BACK_SCORE;
        private static final BACK_SPECIMEN_SEQUENCE[] vals = values();

        public BACK_SPECIMEN_SEQUENCE next() {
            return vals[(this.ordinal() + 1) % vals.length];
        }

        public BACK_SPECIMEN_SEQUENCE prev() {
            return vals[(this.ordinal() - 1 + vals.length) % vals.length];
        }
    }
    public enum FRONT_SPECIMEN_SEQUENCE {
        BACK_GRAB, DROP_SAMPLE, CLOSE_CLAW, FRONT_SCORE, OPEN_CLAW;
        private static final FRONT_SPECIMEN_SEQUENCE[] vals = values();

        public FRONT_SPECIMEN_SEQUENCE next() {
            return vals[(this.ordinal() + 1) % vals.length];
        }

        public FRONT_SPECIMEN_SEQUENCE prev() {
            return vals[(this.ordinal() - 1 + vals.length) % vals.length];
        }
    }
    public enum ASCENT_SEQUENCE {
        SLIDES_UP_LOW, SLIDES_DOWN_LOW, SERVO_HOOKS, SLIDES_UP_HIGH, SLIDE_WAIT, SLIDES_UP_GO;
        private static final ASCENT_SEQUENCE[] vals = values();

        public ASCENT_SEQUENCE next() {
            return vals[(this.ordinal() + 1) % vals.length];
        }

        public ASCENT_SEQUENCE prev() {
            return vals[(this.ordinal() - 1 + vals.length) % vals.length];
        }
    }

    enum OUTTAKE_SEQUENCE {
        BUCKET_SEQUENCE,
        BACK_SPEC_SEQUENCE,
        FRONT_SPEC_SEQUENCE,
        ASCENT,
    }


    BUCKET_SEQUENCE bucketSequence = BUCKET_SEQUENCE.TRANSFER;
    BACK_SPECIMEN_SEQUENCE backSpecimenSequence = BACK_SPECIMEN_SEQUENCE.BACK_SCORE;
    FRONT_SPECIMEN_SEQUENCE frontSpecimenSequence = FRONT_SPECIMEN_SEQUENCE.OPEN_CLAW;
    OUTTAKE_SEQUENCE outtakeSequence = OUTTAKE_SEQUENCE.BUCKET_SEQUENCE;
    ASCENT_SEQUENCE ascentSequence = ASCENT_SEQUENCE.SLIDES_UP_GO;

    private ToggleButton bucketSequenceNextButton = new ToggleButton(true), bucketSequencePrevButton = new ToggleButton(true), backSpecSeqNextButton = new ToggleButton(true), backSpecSeqPrevButton = new ToggleButton(true), ascentSequencePrevButton = new ToggleButton(true), ascentSequenceNextButton = new ToggleButton(true);
    private ToggleButton bucketSequenceNextButton2 = new ToggleButton(true);
    private ToggleButton intakeSequenceNextButton2 = new ToggleButton(true), intakeSequencePreviousButton2 = new ToggleButton(true), intakePipelineSwitchButon = new ToggleButton(true);
    private ToggleButton frontSpecSeqNextButton = new ToggleButton(true), frontSpecSeqPrevButton = new ToggleButton(true);
    private ToggleButton headingLockButton = new ToggleButton(false);
    private ToggleButton headingLockDuringVision = new ToggleButton(false);
    private ToggleButton pipelineToggleButton = new ToggleButton(false);
    private ToggleButton autoScoreToggleButton = new ToggleButton(false);
    private ToggleButton specModeToggleButton = new ToggleButton(false);

    private final Pose outtakePickupPose = new Pose(15, 35, Math.toRadians(180));
    private final Pose outtakePickupControl1 = new Pose(26, 66, Math.toRadians(180));
    private final Pose outtakePickupControl2 = new Pose(35, 37, Math.toRadians(180));
    private final Pose scorePose = new Pose(44.75, 67, Math.toRadians(200)); //can revert if coocked
    private final Pose scorePoseControl = new Pose(20, 66, Math.toRadians(200)); //can revert if coocked

    private final Pose twohundeAngle = new Pose(44.25, 65, Math.toRadians(186));

    public static double hp = 1, hi = 0, hd = 0.12;
    public static double angleThreshold = 2.5;
    PIDController headingPIDController = new PIDController(hp, hi, hd);

    enum AVOID_INTAKE_FSM {
        LIFT_SLIDES,
        MOVE_INTAKE,
        LOWER_SLIDES,
        NOTHING,
    }

    AVOID_INTAKE_FSM avoidIntakeFsm = AVOID_INTAKE_FSM.NOTHING;
    ElapsedTime avoidIntakeFsmTimer;
    public static double DELAY = 0.5;

    private ToggleButton sampleClawLooseToggle = new ToggleButton(false);

    enum AUTO_SCORE {
        DRIVE_TO_PICKUP, PICKUP_AND_GO, DRIVE_TO_SCORE, READY_SCORE, SCORE_AND_GO, NOTHING;
    }
    AUTO_SCORE autoScore;
    ElapsedTime autoScoreTimer;

    public PathChain pickupSpec, scoreSpec;

    public void setAutoScoreState(AUTO_SCORE newState){
        autoScore = newState;
        autoScoreTimer.reset();
    }

    public void buildAutoScorePaths(){
        pickupSpec = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(outtakePickupControl1), new Point(outtakePickupControl2), new Point(outtakePickupPose)))
                .setLinearHeadingInterpolation(twohundeAngle.getHeading(), outtakePickupPose.getHeading())
                .setZeroPowerAccelerationMultiplier(1.7)
                .setPathEndTimeoutConstraint(5)
                .setPathEndTValueConstraint(0.78)
                .build();
        scoreSpec = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePickupPose), new Point(scorePoseControl),new Point(scorePose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(),twohundeAngle.getHeading())
                .setZeroPowerAccelerationMultiplier(5.420)
                .setPathEndTValueConstraint(0.93)
                .setPathEndTimeoutConstraint(2.5)
                .build();
    }

    boolean prevAutoTransfer = false;
    INTAKE_SEQUENCE prevIntakeSequence = INTAKE_SEQUENCE.HOLD;
    private ToggleButton lowBasketToggleButton = new ToggleButton(false);
    private ToggleButton colorSensorDisableButton = new ToggleButton(false);
    private ToggleButton limelightDisableButton = new ToggleButton(false);
    private ToggleButton useVisionIntakeButton = new ToggleButton(false);
    boolean samplePivot = false;

    public static double visionScanTime = 0.67;

    @Override
    public void loop() {
        limelightDisableButton.input(gamepad1.square);
        colorSensorDisableButton.input(gamepad1.triangle);

        lynxModules.loop();
        boolean justChanged = autoScoreToggleButton.input(gamepad2.left_bumper);
        if(justChanged){
            if(autoScoreToggleButton.getVal()){
                ll.turnOff();
                autoScore = AUTO_SCORE.PICKUP_AND_GO;
                follower.setPose(outtakePickupPose);
                follower.update();
            }
            else {
                follower.breakFollowing();
                follower.startTeleopDrive();
                autoScore = AUTO_SCORE.NOTHING;
            }
        }


        specModeToggleButton.input(gamepad2.right_bumper);
        Storage.specMode = specModeToggleButton.getVal();
        telemetry.addData("Mode", Storage.specMode ? "Specimen" : "Sample");

        switch(autoScore){
            case DRIVE_TO_PICKUP:
                if (autoScoreTimer.time() > 0.575){
                    backSpecimenSequence = BACK_SPECIMEN_SEQUENCE.FRONT_GRAB;
                }
                if(!follower.isBusy()){
                    outtakeSequenceTime.reset();
                    setAutoScoreState(AUTO_SCORE.PICKUP_AND_GO);
                }
                break;
            case PICKUP_AND_GO:
                if (autoScoreTimer.time() > 0){
                    outtakeSequenceTime.reset();
                    backSpecimenSequence = BACK_SPECIMEN_SEQUENCE.CLOSE_CLAW;
                    setAutoScoreState(AUTO_SCORE.READY_SCORE);
                }
                break;
            case READY_SCORE:
                if (autoScoreTimer.time() > 0.15){
                    follower.followPath(scoreSpec, true);
                    setAutoScoreState(AUTO_SCORE.DRIVE_TO_SCORE);
                }
                break;
            case DRIVE_TO_SCORE:
                if(!follower.isBusy()){
                    outtakeSequenceTime.reset();
                    backSpecimenSequence = BACK_SPECIMEN_SEQUENCE.OPEN_CLAW;
                    setAutoScoreState(AUTO_SCORE.SCORE_AND_GO);
                }
                break;
            case SCORE_AND_GO:
                if(autoScoreTimer.time() > 0.15){
                    follower.followPath(pickupSpec, true);
                    outtakeSequenceTime.reset();
                    setAutoScoreState(AUTO_SCORE.DRIVE_TO_PICKUP);
                }
                break;
            case NOTHING:
                break;
        }

        sampleClawLooseToggle.input(gamepad2.dpad_left || gamepad1.dpad_left);



        diffyClawIntake.loop();
        diffyClawIntake.HoldExtension();
        ll.loop();
        outtakeLift.loop();
        if(intakeSequenceNextButton2.input(gamepad1.left_bumper)) {
            prevIntakeSequence = intakeSequence;
            prevAutoTransfer = false;
            if (intakeSequence == INTAKE_SEQUENCE.RETRACT){
                if(outtakeSequence != OUTTAKE_SEQUENCE.BUCKET_SEQUENCE || isTransferred || specModeToggleButton.getVal()) {
                    intakeSequence = INTAKE_SEQUENCE.READY;
                } else {
                    intakeSequence = INTAKE_SEQUENCE.TRANSFER_WAIT;
                }
            } else {
                if(intakeSequence == INTAKE_SEQUENCE.HOLD) isTransferred = false;
                intakeSequence = intakeSequence.next();
            }
            intakeSequenceTime.reset();
        } else if(intakeSequencePreviousButton2.input(gamepad1.right_bumper)){
            prevIntakeSequence = intakeSequence;
            prevAutoTransfer = false;
            intakeSequenceTime.reset();
            if(intakeSequence == INTAKE_SEQUENCE.READY){
                intakeSequence = INTAKE_SEQUENCE.RETRACT;
            } else if(intakeSequence == INTAKE_SEQUENCE.HOLD){
                intakeSequence = INTAKE_SEQUENCE.READY;
            } else if(intakeSequence == INTAKE_SEQUENCE.TRANSFER_WAIT) {
                intakeSequence = INTAKE_SEQUENCE.RETRACT;
            } else {
                intakeSequence = intakeSequence.prev();
            }
        }
        if(intakeSequence == INTAKE_SEQUENCE.TRANSFER_WAIT && outtakeSequence != OUTTAKE_SEQUENCE.BUCKET_SEQUENCE){
            intakeSequence = INTAKE_SEQUENCE.RETRACT;
        }


        if(headingLockButton.input(gamepad1.dpad_right) || headingLockDuringVision.input(gamepad1.right_stick_button)){
            targetHeading = follower.getPose().getHeading();
        }

        if(headingLockButton.getVal() || gamepad1.right_stick_button){
            headingError = targetHeading - follower.getPose().getHeading();
            headingError = Math.IEEEremainder(headingError + 2*Math.PI, 2*Math.PI);
            if(headingError > 2*Math.PI - headingError){
                headingError = headingError - 2*Math.PI;
            }

            if(Math.abs(headingError) < Math.toRadians(angleThreshold)){
                headingCorrection = 0;
            } else {
                headingPIDController.setPID(hp, hi, hd);
                headingCorrection = headingPIDController.calculate(headingError);
            }
        }

        if(!Arrays.asList(INTAKE_SEQUENCE.VISION, INTAKE_SEQUENCE.VISION_GO, INTAKE_SEQUENCE.VISION_MOVE).contains(intakeSequence)){
        if(headingLockButton.getVal() && !gamepad1.right_stick_button){
            follower.setTeleOpMovementVectors(
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_y),
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_x), -headingCorrection);
        } else if (intakeSequence == INTAKE_SEQUENCE.READY) { //if intake is down, then we slow down the driving.
            follower.setTeleOpMovementVectors(
                    flip * 0.15 * Math.tan(1.12 * -gamepad1.left_stick_y),
                    flip * 0.15 * Math.tan(1.12 * -gamepad1.left_stick_x),
                    0.1 * Math.tan(1.12 * -gamepad1.right_stick_x), true);
        } else {
            follower.setTeleOpMovementVectors(
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_y),
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_x),
                    0.225 * Math.tan(1.12 * -gamepad1.right_stick_x), true);
        }}

        diffyClawIntake.setUseColorSensor(!colorSensorDisableButton.getVal() && Arrays.asList(INTAKE_SEQUENCE.GRAB, INTAKE_SEQUENCE.HOLD, INTAKE_SEQUENCE.TRANSFER_WAIT).contains(intakeSequence));

        if(!limelightDisableButton.getVal() && intakeSequence == INTAKE_SEQUENCE.READY) ll.turnOn();
        else ll.turnOff();

        switch (intakeSequence){
            case READY:
                diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                diffyClawIntake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.FULL_EXTENSION);
                diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                if (ALignmentButtonNext.input(gamepad1.right_trigger == 1)){
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED += 55;
                    if (Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED > 110){
                        Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = -110;
                    }
                } else if (ALignmentButtonPrev.input(gamepad1.left_trigger == 1)){
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED -= 55;
                    if (Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED < -110){
                        Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 110;
                    }
                }
                if (ll.isRunning() && ll.isResultValid() && gamepad1.right_stick_button) {
                    follower.setTeleOpMovementVectors((targetX - ll.getTx()) * mx, (targetY -  ll.getTy()) * my, -headingCorrection);
                    double angle = ll.getAngle(); // Output 0 is sample angle
                    if(Math.abs(angle) > 85){
                        if(Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED >= 0) angle = 85;
                        else angle = -85;
                    }
                    if(angle < -90) angle = -90;
                    else if(angle > 90) angle = 90;

                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = angle * 10.5/9;
                }
                break;
            case GRAB:
                diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
                //diffyClawIntake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.FULL_EXTENSION);
                if (intakeSequenceTime.time() > 0.15){
                    diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.CLOSED);
                }
                if (intakeSequenceTime.time() > 0.35){
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
                    diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                }
                if(intakeSequenceTime.time() > 0.5){
                    intakeSequence = INTAKE_SEQUENCE.HOLD;
                }
                break;
            case HOLD:
                if(prevIntakeSequence == INTAKE_SEQUENCE.RETRACT){
                    diffyClawIntake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.FULL_EXTENSION);
                }
                diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);

                Intake_DiffyClaw.SENSOR_READING cur = diffyClawIntake.getCurrentSampleState(specModeToggleButton.getVal());

                if(!colorSensorDisableButton.getVal() && intakeSequenceTime.time() > (prevIntakeSequence == INTAKE_SEQUENCE.READY ? 0 : 0.8)){
                    if(cur == Intake_DiffyClaw.SENSOR_READING.INCORRECT || cur == Intake_DiffyClaw.SENSOR_READING.NOTHING){
                        intakeSequence = INTAKE_SEQUENCE.READY;
                    } else if(cur == Intake_DiffyClaw.SENSOR_READING.CORRECT && !specModeToggleButton.getVal()){
                        intakeSequence = INTAKE_SEQUENCE.TRANSFER_WAIT;
                    } else if(prevIntakeSequence == INTAKE_SEQUENCE.READY && cur == Intake_DiffyClaw.SENSOR_READING.CORRECT && specModeToggleButton.getVal()){
                        intakeSequence = INTAKE_SEQUENCE.RETRACT;
                    }
                }
                break;
            case RETRACT:
                medianSmoother.clear();
                diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.CLOSED);
                diffyClawIntake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                if(outtakeSequence == OUTTAKE_SEQUENCE.BUCKET_SEQUENCE && specModeToggleButton.getVal()){
                    diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_SAFE);
                } else if(outtakeSequence != OUTTAKE_SEQUENCE.BACK_SPEC_SEQUENCE && outtakeSequence != OUTTAKE_SEQUENCE.ASCENT){
                    diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_RETRACT_HOLD);
                }
                if (useVisionIntakeButton.input(gamepad1.right_stick_button)){
                    follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
                    follower.update();
                    intakeSequence = INTAKE_SEQUENCE.VISION;
                    intakeSequenceTime.reset();
                }
                break;
            case VISION:
                ll.turnOn();
                ll.setAlliance(Storage.isRed ? IntakeLimelightSubsys.Alliance.RED : IntakeLimelightSubsys.Alliance.BLUE);
                ll.setSampleType(specModeToggleButton.getVal() ? IntakeLimelightSubsys.SampleType.ALLIANCE : IntakeLimelightSubsys.SampleType.BOTH);

                diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.VISION);
                diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                if(intakeSequenceTime.time() > 0.15){
                    if(ll.isResultValid()) {
                        medianSmoother.add(ll.getHoriz(), ll.getVert(), ll.getAngle());
                    }
                    if (intakeSequenceTime.time() > visionScanTime) { // Time it takes for claw to open
                        MedianSmoother.Sample detectedSample = medianSmoother.getMedian();
                        if (medianSmoother.getSize() > 0) {
                            follower.followPath(
                                    follower.pathBuilder()
                                            .addPath(new BezierLine(follower.getPose(), new Pose(follower.getPose().getX(), follower.getPose().getY() - detectedSample.getX())))
                                            .setConstantHeadingInterpolation(follower.getPose().getHeading())
                                            .setZeroPowerAccelerationMultiplier(1.8)
                                            .setPathEndTValueConstraint(0.75)
                                            .setPathEndTimeoutConstraint(100)
                                            .build(),
                                    true
                            );
                            diffyClawIntake.ExtendTo(detectedSample.getY(), Intake_DiffyClaw.ExtensionUnits.ticks);
                            diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                            Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = detectedSample.getAngle() * 110 / 90;
                            diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                            intakeSequence = INTAKE_SEQUENCE.VISION_GO;
                            intakeSequenceTime.reset();
                        } else {
                            intakeSequence = INTAKE_SEQUENCE.RETRACT;
                            intakeSequenceTime.reset();
                        }
                    }
                }
                if (useVisionIntakeButton.input(gamepad1.right_stick_button)){
                    intakeSequence = INTAKE_SEQUENCE.RETRACT;
                    intakeSequenceTime.reset();
                }
                break;
            case VISION_GO:
                ll.turnOff();
                if (!follower.isBusy() && diffyClawIntake.extensionReachedTarget()){
                    follower.startTeleopDrive();
                    follower.update();
                    intakeSequence = INTAKE_SEQUENCE.GRAB;
                    intakeSequenceTime.reset();
                }
                if (useVisionIntakeButton.input(gamepad1.right_stick_button)){
                    intakeSequence = INTAKE_SEQUENCE.RETRACT;
                    intakeSequenceTime.reset();
                }
                break;
            case TRANSFER_WAIT:
                if (intakeSequenceTime.time() > 0.7){
                    diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.LOOSE);
                }
                diffyClawIntake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER);
                boolean asdf = !colorSensorDisableButton.getVal() && intakeSequenceTime.time() > 1.3 && diffyClawIntake.getCurrentPosition() < 20 && diffyClawIntake.getCurrentSampleState(specModeToggleButton.getVal()) == Intake_DiffyClaw.SENSOR_READING.CORRECT && !specModeToggleButton.getVal();
                if(asdf && !prevAutoTransfer){
                    bucketSequence = BUCKET_SEQUENCE.GRAB_AND_LIFT;
                    outtakeSequenceTime.reset();
                }
                prevAutoTransfer = asdf;
                break;
        }
        if(takeSnapshotButton.input(gamepad1.circle)){
            ll.captureSnapshot(key+Math.round(elapsedTime.time()));
        }

        //outtake stuff
        if(bucketSequenceNextButton.input(gamepad2.a) || bucketSequenceNextButton2.input(gamepad1.a)){
            outtakeSequence = OUTTAKE_SEQUENCE.BUCKET_SEQUENCE;
            bucketSequence = bucketSequence.next();
            backSpecimenSequence = BACK_SPECIMEN_SEQUENCE.vals[BACK_SPECIMEN_SEQUENCE.vals.length-1];
            frontSpecimenSequence = FRONT_SPECIMEN_SEQUENCE.vals[FRONT_SPECIMEN_SEQUENCE.vals.length-1];
            ascentSequence = ASCENT_SEQUENCE.vals[ASCENT_SEQUENCE.vals.length-1];
            outtakeSequenceTime.reset();
        } else if(bucketSequencePrevButton.input(gamepad2.b)){
            outtakeSequence = OUTTAKE_SEQUENCE.BUCKET_SEQUENCE;
            backSpecimenSequence = BACK_SPECIMEN_SEQUENCE.vals[BACK_SPECIMEN_SEQUENCE.vals.length-1];
            frontSpecimenSequence = FRONT_SPECIMEN_SEQUENCE.vals[FRONT_SPECIMEN_SEQUENCE.vals.length-1];
            ascentSequence = ASCENT_SEQUENCE.vals[ASCENT_SEQUENCE.vals.length-1];
            bucketSequence = bucketSequence.prev();
            outtakeSequenceTime.reset();
        } else if(backSpecSeqNextButton.input(gamepad2.x)){
            outtakeSequence = OUTTAKE_SEQUENCE.BACK_SPEC_SEQUENCE;
            backSpecimenSequence = backSpecimenSequence.next();
            bucketSequence = BUCKET_SEQUENCE.vals[BUCKET_SEQUENCE.vals.length-1];
            frontSpecimenSequence = FRONT_SPECIMEN_SEQUENCE.vals[FRONT_SPECIMEN_SEQUENCE.vals.length-1];
            ascentSequence = ASCENT_SEQUENCE.vals[ASCENT_SEQUENCE.vals.length-1];
            outtakeSequenceTime.reset();
        } else if(backSpecSeqPrevButton.input(gamepad2.y)){
            outtakeSequence = OUTTAKE_SEQUENCE.BACK_SPEC_SEQUENCE;
            backSpecimenSequence = backSpecimenSequence.prev();
            bucketSequence = BUCKET_SEQUENCE.vals[BUCKET_SEQUENCE.vals.length-1];
            frontSpecimenSequence = FRONT_SPECIMEN_SEQUENCE.vals[FRONT_SPECIMEN_SEQUENCE.vals.length-1];
            ascentSequence = ASCENT_SEQUENCE.vals[ASCENT_SEQUENCE.vals.length-1];
            outtakeSequenceTime.reset();
        } else if(ascentSequenceNextButton.input(gamepad2.dpad_up)){
            outtakeSequence = OUTTAKE_SEQUENCE.ASCENT;
            ascentSequence = ascentSequence.next();
            bucketSequence = BUCKET_SEQUENCE.vals[BUCKET_SEQUENCE.vals.length-1];
            frontSpecimenSequence = FRONT_SPECIMEN_SEQUENCE.vals[FRONT_SPECIMEN_SEQUENCE.vals.length-1];
            backSpecimenSequence = BACK_SPECIMEN_SEQUENCE.vals[BACK_SPECIMEN_SEQUENCE.vals.length-1];
            hangTimer.reset();
        } else if(frontSpecSeqNextButton.input(gamepad2.dpad_right)){
            outtakeSequence = OUTTAKE_SEQUENCE.FRONT_SPEC_SEQUENCE;
            frontSpecimenSequence = frontSpecimenSequence.next();
            bucketSequence = BUCKET_SEQUENCE.vals[BUCKET_SEQUENCE.vals.length-1];
            backSpecimenSequence = BACK_SPECIMEN_SEQUENCE.vals[BACK_SPECIMEN_SEQUENCE.vals.length-1];
            ascentSequence = ASCENT_SEQUENCE.vals[ASCENT_SEQUENCE.vals.length-1];
            outtakeSequenceTime.reset();
        } else if(frontSpecSeqPrevButton.input(gamepad2.dpad_down)){
            outtakeSequence = OUTTAKE_SEQUENCE.FRONT_SPEC_SEQUENCE;
            frontSpecimenSequence = frontSpecimenSequence.prev();
            bucketSequence = BUCKET_SEQUENCE.vals[BUCKET_SEQUENCE.vals.length-1];
            backSpecimenSequence = BACK_SPECIMEN_SEQUENCE.vals[BACK_SPECIMEN_SEQUENCE.vals.length-1];
            ascentSequence = ASCENT_SEQUENCE.vals[ASCENT_SEQUENCE.vals.length-1];
            outtakeSequenceTime.reset();
        }

        lowBasketToggleButton.input(gamepad1.dpad_down);

        switch(outtakeSequence){
            case BUCKET_SEQUENCE:
                if(diffyClawIntake.intakeState == Intake_DiffyClaw.IntakeState.INTAKE_REST || diffyClawIntake.intakeState == Intake_DiffyClaw.IntakeState.DEPOSIT){
                    avoidIntakeFsm = AVOID_INTAKE_FSM.LIFT_SLIDES;
                    avoidIntakeFsmTimer.reset();
                }
                switch(avoidIntakeFsm){
                    case LIFT_SLIDES:
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.AVOID_INTAKE);
                        if(!outtakeLift.isBusy()){
                            diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_RETRACT_HOLD);
                            avoidIntakeFsmTimer.reset();
                            avoidIntakeFsm = AVOID_INTAKE_FSM.MOVE_INTAKE;
                        }
                        break;
                    case MOVE_INTAKE:
                        if(avoidIntakeFsmTimer.time() > DELAY){
                            avoidIntakeFsm = AVOID_INTAKE_FSM.NOTHING;
                            bucketSequence = BUCKET_SEQUENCE.TRANSFER;
                        }
                        break;
                    case NOTHING:
                        switch (bucketSequence) {
                            case TRANSFER:
                                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.TRANSFER);
                                outtake.setOuttakeState(specModeToggleButton.getVal() ? Outtake.OuttakeState.SAFE_POS : Outtake.OuttakeState.TRANSFER);
                                outtake.setClawState(Outtake.ClawStates.OPEN);
                                break;
                            case GRAB_AND_LIFT:
                                outtake.setClawState(Outtake.ClawStates.CLOSED);
                                if(outtakeSequenceTime.time() > 0.1){
                                    diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                                }
                                if (outtakeSequenceTime.time() > 0.3) {
                                    outtakeLift.LiftTo(lowBasketToggleButton.getVal() ? OuttakeLiftSubsys.OuttakeLiftPositions.LIFT_LOW_BASKET : OuttakeLiftSubsys.OuttakeLiftPositions.LIFT_HIGH_BASKET);
                                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLE_SCORE_WAIT);
                                    intakeSequence = INTAKE_SEQUENCE.RETRACT;
                                    isTransferred = true;
                                    resetEncoderDelay.reset();
                                }
                                if (outtakeSequenceTime.time() < 0.75){
                                    samplePivot = false;
                                } else {
                                    if (Math.abs(OuttakeLiftSubsys.target - outtakeLift.getCurrentPosition()) < 50){
                                        samplePivot = true;
                                    }
                                }
                                if (samplePivot){
                                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLESCORE);
                                    outtake.setClawState(Outtake.ClawStates.LOOSE_CLOSED);
                                }
                                break;
                            case SCORE:
                                outtake.setClawState(Outtake.ClawStates.OPEN);
                                if (resetEncoderDelay.time() > 0.3) {
                                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLE_SCORE_WAIT);
                                }
                                break;
                        }
                        break;
                }
                break;
            case BACK_SPEC_SEQUENCE:
                if(intakeSequence == INTAKE_SEQUENCE.RETRACT && avoidIntakeFsm == AVOID_INTAKE_FSM.NOTHING && diffyClawIntake.intakeState != Intake_DiffyClaw.IntakeState.INTAKE_REST){
                    avoidIntakeFsm = AVOID_INTAKE_FSM.LIFT_SLIDES;
                    avoidIntakeFsmTimer.reset();
                }

                switch(avoidIntakeFsm){
                    case LIFT_SLIDES:
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.AVOID_INTAKE);
                        if(!outtakeLift.isBusy()){
                            diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_REST);
                            avoidIntakeFsmTimer.reset();
                            avoidIntakeFsm = AVOID_INTAKE_FSM.MOVE_INTAKE;
                        }
                        break;
                    case MOVE_INTAKE:
                        if(avoidIntakeFsmTimer.time() > DELAY){
                            avoidIntakeFsm = AVOID_INTAKE_FSM.NOTHING;
                            backSpecimenSequence = BACK_SPECIMEN_SEQUENCE.FRONT_GRAB;
                        }
                        break;
                    case NOTHING:
                        switch (backSpecimenSequence){
                            case OPEN_CLAW:
                                if (outtakeSequenceTime.time() < 0.02){
                                gamepad1.rumbleBlips(1);}
                                outtake.setClawState(Outtake.ClawStates.OPEN);
                                if (outtakeSequenceTime.time() > 0.3){
                                    outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCOREOUT);
                                }
                                break;
                            case FRONT_GRAB:
                                outtake.setClawState(Outtake.ClawStates.OPEN);
                                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
                                outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
                                break;
                            case CLOSE_CLAW:
                                outtake.setClawState(Outtake.ClawStates.CLOSED);
                                if(outtakeSequenceTime.time() > 0.15){
                                    backSpecimenSequence = BACK_SPECIMEN_SEQUENCE.BACK_SCORE;
                                    outtakeSequenceTime.reset();
                                }
                                break;
                            case BACK_SCORE:
                                if (outtakeSequenceTime.time() < 0.02){
                                    gamepad1.rumbleBlips(1);}
                                outtake.setClawState(Outtake.ClawStates.CLOSED);
                                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.BACK_SCORE);
                                outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCORE);
                                break;
                        }
                        break;
                }
                break;
            case FRONT_SPEC_SEQUENCE:
                switch (frontSpecimenSequence){
                    case BACK_GRAB:
                        outtake.setClawState(Outtake.ClawStates.OPEN);
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.BACK_PICKUP_WAIT);
                        outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKPICKUP);
                        if(intakeSequence == INTAKE_SEQUENCE.RETRACT){
                            diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.DEPOSIT);
                            if(outtakeSequenceTime.time() > 0.2){
                                diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.LOOSE);
                            }
                        }
                        break;
                    case DROP_SAMPLE:
                        diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                        if(intakeSequence == INTAKE_SEQUENCE.RETRACT){
                            diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.DEPOSIT);
                        }
                        if(outtakeSequenceTime.time() > 0.35){
                            diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_REST);
                            outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.BACK_PICKUP);
                        }
                        break;
                    case CLOSE_CLAW:
                        outtake.setClawState(Outtake.ClawStates.CLOSED);
                        if(intakeSequence == INTAKE_SEQUENCE.RETRACT){
                            diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.DEPOSIT);
                            diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                        }
                        if (outtakeSequenceTime.time() > 0.3){
                            outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_SCORE);
                        }
                        break;
                    case FRONT_SCORE:
                        outtake.setClawState(Outtake.ClawStates.CLOSED);
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_SCORE);
                        outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTSCOREWAIT);
                        break;
                    case OPEN_CLAW:
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_SCORE);
                        outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTSCOREDONE);
                        if(outtakeSequenceTime.time() > 0.3){
                            outtake.setClawState(Outtake.ClawStates.OPEN);
                        }
                        break;
                }
                break;
            case ASCENT:
                if(intakeSequence == INTAKE_SEQUENCE.RETRACT && avoidIntakeFsm == AVOID_INTAKE_FSM.NOTHING && diffyClawIntake.intakeState != Intake_DiffyClaw.IntakeState.INTAKE_REST && diffyClawIntake.intakeState != Intake_DiffyClaw.IntakeState.HANG){
                    avoidIntakeFsm = AVOID_INTAKE_FSM.LIFT_SLIDES;
                    avoidIntakeFsmTimer.reset();
                }

                switch(avoidIntakeFsm){
                    case LIFT_SLIDES:
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.AVOID_INTAKE);
                        diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_REST);
                        avoidIntakeFsmTimer.reset();
                        avoidIntakeFsm = AVOID_INTAKE_FSM.MOVE_INTAKE;
                        break;
                    case MOVE_INTAKE:
                        if(avoidIntakeFsmTimer.time() > 0){
                            avoidIntakeFsm = AVOID_INTAKE_FSM.NOTHING;
                        }
                        break;
                    case NOTHING:
                        switch (ascentSequence){
                            case SLIDES_UP_LOW:
                                outtakeLift.stopHang();
                                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.LOW_BAR_WAIT);
                                break;
                            case SLIDES_DOWN_LOW:
                                outtakeLift.useHang();
                                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.LOW_BAR_DONE);
                                if (hangTimer.time() > 0.2){
                                    ascentSequence = ASCENT_SEQUENCE.SERVO_HOOKS;
                                }
                                break;
                            case SERVO_HOOKS:
                                hangServos.hang();
                                break;
                            case SLIDES_UP_HIGH:
                                outtakeLift.stopHang();
                                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.HIGH_BAR_WAIT);
                                ascentSequence = ASCENT_SEQUENCE.SLIDE_WAIT;
                                break;
                            case SLIDE_WAIT:
                                if (outtakeLift.getCurrentPosition() > 3250){
                                    ascentSequence = ASCENT_SEQUENCE.SLIDES_UP_GO;
                                    hangTimer.reset();
                                }
                                break;
                            case SLIDES_UP_GO:
                                outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKPICKUP);
                                if(hangTimer.time() > 0.2){
                                    outtakeLift.useHang();
                                    outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.HIGH_BAR_DONE);
                                }
                                if(hangTimer.time() > 0.4){
                                    hangServos.rest();
                                }
                                if(hangTimer.time() > 3){
                                    diffyClawIntake.useHang();
                                    diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.HANG);
                                    hangServos.hang();
                                    outtake.setOuttakeState(Outtake.OuttakeState.RESET_ENCODER);
                                }
                                break;
                        }
                        break;
                }
                break;
        }

        outtakeLift.holdLift();
        outtake.loop();


        controlFlipButton.input(gamepad1.dpad_up);
        flip = controlFlipButton.getVal() ? 1 : -1;



        follower.update();
        Storage.CurrentPose = follower.getPose();

        tel.addData("Control:", controlFlipButton.getVal() ? "Normal" : "Flipped");
        tel.addData("Pipeline", pipelineToggleButton.getVal() ? "Yellow" : "Alliance");
        if(outtakeSequence == OUTTAKE_SEQUENCE.BUCKET_SEQUENCE && !specModeToggleButton.getVal()){

            tel.addData("Bucket Orientation", sampleClawLooseToggle.getVal() ? "Loose" : "Tight");
            tel.addData("Basket Height", lowBasketToggleButton.getVal() ? "Low" : "High");
        }
        tel.addData("Limelight Enabled", !limelightDisableButton.getVal());
        tel.addData("Color Sensor Enabled", !colorSensorDisableButton.getVal());
        tel.addData("Target Heading in Degrees", Math.toDegrees(targetHeading));
        tel.addData("Angle Error in Degrees", Math.toDegrees(headingError));
        tel.addData("Correction Vector in Degrees", Math.toDegrees(headingCorrection));
        tel.addData("X", follower.getPose().getX());
        tel.addData("Y", follower.getPose().getY());
        tel.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        tel.addData("Elapsed Time", elapsedTime.toString());
        tel.addData("prevIntakeSequence", prevIntakeSequence.name());
        tel.addData("Intake Sequence", intakeSequence.name());
        tel.addData("Outtake Sequence", outtakeSequence.name());
        switch (outtakeSequence){
            case BUCKET_SEQUENCE:
                tel.addData("Bucket Sequence", bucketSequence.name());
                break;
            case BACK_SPEC_SEQUENCE:
                tel.addData("Back Specimen Sequence", backSpecimenSequence.name());
                break;
            case FRONT_SPEC_SEQUENCE:
                tel.addData("Front Specimen Sequence", frontSpecimenSequence.name());
                break;
            case ASCENT:
                tel.addData("Ascent Sequence", ascentSequence.name());
                break;
        }
        tel.addData("Loop Time", loopTime.toString());
        tel.update();
        loopTime.reset();
    }
}
