

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import config.subsystems.DriveSubsys;
import config.subsystems.HangServoSubsys;
import config.subsystems.IntakeLimelightSubsys;
import config.subsystems.Outtake;
import config.subsystems.Lift;
import config.core.utils.SubsystemCore;
import config.subsystems.UnifiedTelemetry;


@TeleOp(name = "Main TeleOp", group = "_TeleOp")
@Config
public class Main_TeleOp extends OpMode {
    public static double mx =  -0.008, my =  -0.021;
    public static double targetX = 16, targetY = 0;
    private Follower follower;

    private Intake_DiffyClaw diffyClawIntake;
    private DriveSubsys driveSubsys;
    private IntakeLimelightSubsys ll;
    private Lift outtakeLift;
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
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
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
        SubsystemCore.setGlobalParameters(hardwareMap, this);

        driveSubsys = new DriveSubsys();
        driveSubsys.init();
        diffyClawIntake = new Intake_DiffyClaw();
        ll = new IntakeLimelightSubsys();
        ll.init();
        outtakeLift = new Lift();
        outtakeLift.init();
        outtake = new Outtake(hardwareMap);
        hangServos = new HangServoSubsys();
        hangServos.init();

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
    }

    private ToggleButton teamColorButton = new ToggleButton(Storage.isRed);
    private ToggleButton controlFlipButton = new ToggleButton(true);

    @Override
    public void init_loop(){
        teamColorButton.input(gamepad1.dpad_up);
        Storage.isRed = teamColorButton.getVal();
        follower.update();
        Storage.CurrentPose = follower.getPose();
        telemetry.addLine("DO NOT TOUCH IF THIS IS REAL GAME, or make sure you dont misclick.");
        telemetry.addData("Team Color:", Storage.isRed ? "Red" : "Blue");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        diffyClawIntake.init();
        loopTime.startTime();
    }
    public enum INTAKE_SEQUENCE{
        TRANSFER_WAIT, READY, GRAB;
        private static final INTAKE_SEQUENCE[] vals = values();
        public INTAKE_SEQUENCE next(){
            return vals[(this.ordinal() + 1) % vals.length];
        }
        public INTAKE_SEQUENCE prev(){
            return vals[(this.ordinal() - 1 + vals.length) % vals.length];
        }
    }
    INTAKE_SEQUENCE intakeSequence = INTAKE_SEQUENCE.TRANSFER_WAIT;
    private ToggleButton intakeSequenceNextButton = new ToggleButton(true), intakeSequencePreviousButton = new ToggleButton(true), ALignmentButtonNext = new ToggleButton(true),ALignmentButtonPrev = new ToggleButton(true), autoALignmentButton = new ToggleButton(true);
    //outtake stuff
    public enum BUCKET_SEQUENCE{
        TRANSFER, GRAB_AND_LIFT, SCORE, RESET;
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
        BACK_GRAB, CLOSE_CLAW, FRONT_SCORE, OPEN_CLAW;
        private static final FRONT_SPECIMEN_SEQUENCE[] vals = values();

        public FRONT_SPECIMEN_SEQUENCE next() {
            return vals[(this.ordinal() + 1) % vals.length];
        }

        public FRONT_SPECIMEN_SEQUENCE prev() {
            return vals[(this.ordinal() - 1 + vals.length) % vals.length];
        }
    }
    public enum ASCENT_SEQUENCE {
        SLIDES_UP_LOW, SLIDES_DOWN_LOW, SERVO_HOOKS, SLIDES_UP_HIGH, SLIDES_UP_GO;
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
    private ToggleButton intakeSequenceNextButton2 = new ToggleButton(true), intakeSequencePreviousButton2 = new ToggleButton(true), intakePipelineSwitchButon = new ToggleButton(true);
    private ToggleButton frontSpecSeqNextButton = new ToggleButton(true), frontSpecSeqPrevButton = new ToggleButton(true);
    private ToggleButton headingLockButton = new ToggleButton(false);
    private ToggleButton headingLockDuringVision = new ToggleButton(false);
    private ToggleButton pipelineToggleButton = new ToggleButton(false);
    private ToggleButton autoScoreToggleButton = new ToggleButton(false);
    public static Pose[] scorePoses = {
            new Pose(11.1, 32, Math.toRadians(180)),
            new Pose(20, 32, Math.toRadians(180)),
            new Pose(20, 32, Math.toRadians(180)),
            new Pose(30, 65, Math.toRadians(180)),
            new Pose(39.3, 68, Math.toRadians(180))
    };
    public static Pose[] pickupPoses = {
            new Pose(39.3, 68, Math.toRadians(180)),
            new Pose(35, 68, Math.toRadians(180)),
            new Pose(30, 68, Math.toRadians(180)),
            new Pose(20, 32, Math.toRadians(180)),
            new Pose(15, 32, Math.toRadians(180)),
            new Pose(11.1, 32, Math.toRadians(180))
    };


    public static double hp = 0.4, hi = 0, hd = 0.00008;
    public static double angleThreshold = 0.05;
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

    enum AUTO_SCORE {
        DRIVE_TO_PICKUP, PICKUP_AND_GO, DRIVE_TO_SCORE, SCORE_AND_GO, NOTHING;
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
                .addPath(new BezierCurve(pickupPoses))
                .setLinearHeadingInterpolation(pickupPoses[0].getHeading(), pickupPoses[pickupPoses.length-1].getHeading())
                .build();
        scoreSpec = follower.pathBuilder()
                .addPath(new BezierCurve(scorePoses))
                .setLinearHeadingInterpolation(scorePoses[0].getHeading(), scorePoses[scorePoses.length-1].getHeading())
                .build();
    }

    @Override
    public void loop() {
        boolean justChanged = autoScoreToggleButton.input(gamepad2.left_bumper);
        if(justChanged){
            if(autoScoreToggleButton.getVal()){
                autoScore = AUTO_SCORE.PICKUP_AND_GO;
                follower.setPose(scorePoses[0]);
                follower.update();
            }
            else autoScore = AUTO_SCORE.NOTHING;
        }

        switch(autoScore){
            case DRIVE_TO_PICKUP:
                if(!follower.isBusy()){
                    setAutoScoreState(AUTO_SCORE.PICKUP_AND_GO);
                }
                break;
            case PICKUP_AND_GO:
                backSpecimenSequence = BACK_SPECIMEN_SEQUENCE.CLOSE_CLAW;
                if(autoScoreTimer.time() > 0.5){
                    follower.followPath(scoreSpec, true);
                    setAutoScoreState(AUTO_SCORE.DRIVE_TO_SCORE);
                }
                break;
            case DRIVE_TO_SCORE:
                if(autoScoreTimer.time() > 0.2){
                    backSpecimenSequence = BACK_SPECIMEN_SEQUENCE.BACK_SCORE;
                }
                if(!follower.isBusy()){
                    setAutoScoreState(AUTO_SCORE.SCORE_AND_GO);
                }
                break;
            case SCORE_AND_GO:
                backSpecimenSequence = BACK_SPECIMEN_SEQUENCE.OPEN_CLAW;
                if(autoScoreTimer.time() > 0.5){
                    follower.followPath(pickupSpec, true);
                    backSpecimenSequence = BACK_SPECIMEN_SEQUENCE.FRONT_GRAB;
                    setAutoScoreState(AUTO_SCORE.DRIVE_TO_PICKUP);
                }
                break;
            case NOTHING:
                if(follower.isBusy()) follower.breakFollowing();
                break;
        }


        diffyClawIntake.loop();
        diffyClawIntake.HoldExtension();
        ll.loop();
        outtakeLift.loop();
        if(intakeSequenceNextButton2.input(gamepad1.left_bumper)){
            intakeSequence = intakeSequence.next();
            intakeSequenceTime.reset();
        } else if(intakeSequencePreviousButton2.input(gamepad1.right_bumper)){
            intakeSequence = intakeSequence.prev();
            intakeSequenceTime.reset();
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

        if(headingLockButton.getVal() && !gamepad1.right_stick_button){
            follower.setTeleOpMovementVectors(
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_y),
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_x), -headingCorrection);
        } else if (intakeSequence == INTAKE_SEQUENCE.TRANSFER_WAIT || intakeSequence == INTAKE_SEQUENCE.GRAB) {
            follower.setTeleOpMovementVectors(
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_y),
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_x),
                    0.225 * Math.tan(1.12 * -gamepad1.right_stick_x), true);
        } else { //if intake is down, then we slow down the driving.
            follower.setTeleOpMovementVectors(
                    flip * 0.15 * Math.tan(1.12 * -gamepad1.left_stick_y),
                    flip * 0.15 * Math.tan(1.12 * -gamepad1.left_stick_x),
                    0.1 * Math.tan(1.12 * -gamepad1.right_stick_x), true);
        }
        if (intakePipelineSwitchButon.input(gamepad1.a)){
            diffyClawIntake.changePipeline(4);
        } else {
            if (Storage.isRed) {
                diffyClawIntake.changePipeline(6);
            } else {
                diffyClawIntake.changePipeline(5);
            }
        }
        switch (intakeSequence){
            case READY:
                diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                diffyClawIntake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.FULL_EXTENSION);
                ll.turnOn();
                diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                if (ALignmentButtonNext.input(gamepad1.right_trigger == 1)){
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
//                    if (Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED > 100){
//                        Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = -100;
//                    }
                } else if (ALignmentButtonPrev.input(gamepad1.left_trigger == 1)){
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 110;
//                    if (Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED < -100){
//                        Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 100;
//                    }
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
                ll.turnOff();
                diffyClawIntake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.FULL_EXTENSION);
                if (intakeSequenceTime.time() > 0.2){
                    diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.CLOSED);
                }
                if (intakeSequenceTime.time() > 0.4){
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
                    diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                }
                break;
            case TRANSFER_WAIT:
                ll.turnOff();
                if (intakeSequenceTime.time() > 0.7){
                    diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.LOOSE);
                }
                if(outtakeSequence != OUTTAKE_SEQUENCE.BACK_SPEC_SEQUENCE && outtakeSequence != OUTTAKE_SEQUENCE.ASCENT){
                    diffyClawIntake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                    diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER_WAIT);
                }
                break;
        }
        if(takeSnapshotButton.input(gamepad1.circle)){
            ll.captureSnapshot(key+Math.round(elapsedTime.time()));
        }

        pipelineToggleButton.input(gamepad1.square);
        ll.setPipelineNumber(pipelineToggleButton.getVal() ? 4 : (Storage.isRed ?  6 : 5));

        //outtake stuff
        if(bucketSequenceNextButton.input(gamepad2.a)){
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
            frontSpecimenSequence = frontSpecimenSequence.next();
            bucketSequence = BUCKET_SEQUENCE.vals[BUCKET_SEQUENCE.vals.length-1];
            backSpecimenSequence = BACK_SPECIMEN_SEQUENCE.vals[BACK_SPECIMEN_SEQUENCE.vals.length-1];
            ascentSequence = ASCENT_SEQUENCE.vals[ASCENT_SEQUENCE.vals.length-1];
            outtakeSequenceTime.reset();
        }

        switch(outtakeSequence){
            case BUCKET_SEQUENCE:
                switch (bucketSequence){
                    case TRANSFER:
                        outtakeLift.LiftTo(Lift.OuttakeLiftPositions.TRANSFER);
                        outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER_WAIT);
                        outtake.setClawOpen(true);
                        break;
                    case GRAB_AND_LIFT:
                        diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER);
                        outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER);
                        if (outtakeSequenceTime.time() > 0.23){
                            outtake.setClawOpen(false);
                        }
                        if (outtakeSequenceTime.time() > 0.74){
                            diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                        }
                        if(outtakeSequenceTime.time() > 1){
                            outtakeLift.LiftTo(Lift.OuttakeLiftPositions.LIFT_BUCKET);
                            outtake.setOuttakeState(Outtake.OuttakeState.SAMPLESCORE);
                            resetEncoderDelay.reset();
                        }
                        break;
                    case SCORE:
                        outtake.setClawOpen(true);
                        if (resetEncoderDelay.time() > 0.3){
                            outtake.setOuttakeState(Outtake.OuttakeState.SAMPLE_SCORE_WAIT);
                        }
                        break;
                }
                break;
            case BACK_SPEC_SEQUENCE:
                if(intakeSequence == INTAKE_SEQUENCE.TRANSFER_WAIT && avoidIntakeFsm == AVOID_INTAKE_FSM.NOTHING && diffyClawIntake.intakeState != Intake_DiffyClaw.IntakeState.INTAKE_REST){
                    avoidIntakeFsm = AVOID_INTAKE_FSM.LIFT_SLIDES;
                    avoidIntakeFsmTimer.reset();
                }

                switch(avoidIntakeFsm){
                    case LIFT_SLIDES:
                        outtakeLift.LiftTo(Lift.OuttakeLiftPositions.AVOID_INTAKE);
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
                                outtake.setClawOpen(true);
                                if (outtakeSequenceTime.time() > 0.3){
                                    outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCOREOUT);
                                }
                                break;
                            case FRONT_GRAB:
                                outtake.setClawOpen(true);
                                outtakeLift.LiftTo(Lift.OuttakeLiftPositions.FRONT_PICKUP);
                                outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
                                break;
                            case CLOSE_CLAW:
                                outtake.setClawOpen(false);
                                break;
                            case BACK_SCORE:
                                if (outtakeSequenceTime.time() < 0.02){
                                    gamepad1.rumbleBlips(1);}
                                outtake.setClawOpen(false);
                                outtakeLift.LiftTo(Lift.OuttakeLiftPositions.BACK_SCORE);
                                outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCORE);
                                break;
                        }
                        break;
                }
                break;
            case FRONT_SPEC_SEQUENCE:
                switch (frontSpecimenSequence){
                    case BACK_GRAB:
                        outtake.setClawOpen(true);
                        outtakeLift.LiftTo(Lift.OuttakeLiftPositions.BACK_PICKUP);
                        outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKPICKUP);
                        if(intakeSequence == INTAKE_SEQUENCE.TRANSFER_WAIT){
                            diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.DEPOSIT);
                        }
                        break;
                    case CLOSE_CLAW:
                        outtake.setClawOpen(false);
                        if(intakeSequence == INTAKE_SEQUENCE.TRANSFER_WAIT){
                            diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.DEPOSIT);
                            diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
                        }
                        if (outtakeSequenceTime.time() > 0.5){
                            outtakeLift.LiftTo(Lift.OuttakeLiftPositions.FRONT_SCORE_WAIT);
                        }
                        break;
                    case FRONT_SCORE:
                        outtake.setClawOpen(false);
                        outtakeLift.LiftTo(Lift.OuttakeLiftPositions.FRONT_SCORE_WAIT);
                        outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTSCORE);
                        break;
                    case OPEN_CLAW:
                        outtakeLift.LiftTo(Lift.OuttakeLiftPositions.FRONT_SCORE_DONE);
                        outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTSCORE);
                        if(outtakeSequenceTime.time() > 1){
                            outtake.setClawOpen(true);
                        }
                        break;
                }
                break;
            case ASCENT:
                if(intakeSequence == INTAKE_SEQUENCE.TRANSFER_WAIT && avoidIntakeFsm == AVOID_INTAKE_FSM.NOTHING && diffyClawIntake.intakeState != Intake_DiffyClaw.IntakeState.INTAKE_REST && diffyClawIntake.intakeState != Intake_DiffyClaw.IntakeState.HANG){
                    avoidIntakeFsm = AVOID_INTAKE_FSM.LIFT_SLIDES;
                    avoidIntakeFsmTimer.reset();
                }

                switch(avoidIntakeFsm){
                    case LIFT_SLIDES:
                        outtakeLift.LiftTo(Lift.OuttakeLiftPositions.AVOID_INTAKE);
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
                                outtakeLift.LiftTo(Lift.OuttakeLiftPositions.LOW_BAR_WAIT);
                                break;
                            case SLIDES_DOWN_LOW:
                                outtakeLift.useHang();
                                outtakeLift.LiftTo(Lift.OuttakeLiftPositions.LOW_BAR_DONE);
                                break;
                            case SERVO_HOOKS:
                                hangServos.hang();
                                break;
                            case SLIDES_UP_HIGH:
                                outtakeLift.stopHang();
                                outtakeLift.LiftTo(Lift.OuttakeLiftPositions.HIGH_BAR_WAIT);
                                break;
                            case SLIDES_UP_GO:
                                outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKPICKUP);
                                if(hangTimer.time() > 0.2){
                                    outtakeLift.useHang();
                                    outtakeLift.LiftTo(Lift.OuttakeLiftPositions.HIGH_BAR_DONE);
                                }
                                if(hangTimer.time() > 0.5){
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
        outtake.outtakeLoop();


        controlFlipButton.input(gamepad1.dpad_up);
        flip = controlFlipButton.getVal() ? 1 : -1;



        follower.update();
        Storage.CurrentPose = follower.getPose();

//        tel.addData("Control:", controlFlipButton.getVal() ? "Normal" : "Flipped");
        tel.addData("Pipeline", pipelineToggleButton.getVal() ? "Yellow" : "Alliance");
//        tel.addData("Target Heading in Degrees", Math.toDegrees(targetHeading));
//        tel.addData("Angle Error in Degrees", Math.toDegrees(headingError));
//        tel.addData("Correction Vector in Degrees", Math.toDegrees(headingCorrection));
//        tel.addData("X", follower.getPose().getX());
//        tel.addData("Y", follower.getPose().getY());
//        tel.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
//        tel.addData("Elapsed Time", elapsedTime.toString());
//        tel.addData("Loop Time", loopTime.toString());
        tel.update();
        loopTime.reset();
    }
}