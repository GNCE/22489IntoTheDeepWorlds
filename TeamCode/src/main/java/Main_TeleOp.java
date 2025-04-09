

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import subsystems.DriveSubsys;
import subsystems.HangServoSubsys;
import subsystems.IntakeLimelightSubsys;
import subsystems.OuttakeLiftSubsys;
import subsystems.SubsysCore;
import subsystems.UnifiedTelemetry;


@TeleOp(name = "Main TeleOp", group = "_TeleOp")
@Config
public class Main_TeleOp extends OpMode {
    public static double mx =  -0.012, my =  -0.012;
    public static double targetX = 16, targetY = 0;
    private Follower follower;

    private Intake_DiffyClaw diffyClawIntake;
    private DriveSubsys driveSubsys;
    private IntakeLimelightSubsys ll;
    private OuttakeLiftSubsys outtakeLift;
    private Outtake outtake;
    private HangServoSubsys hangServos;
    private ElapsedTime elapsedTime, intakeSequenceTime, resetEncoderDelay, outtakeSequenceTime, hangTimer;
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

        tel = new UnifiedTelemetry();
        tel.init(this.telemetry);
        SubsysCore.setGlobalParameters(hardwareMap, this);

        driveSubsys = new DriveSubsys();
        driveSubsys.init();
        diffyClawIntake = new Intake_DiffyClaw();
        ll = new IntakeLimelightSubsys();
        ll.init();
        outtakeLift = new OuttakeLiftSubsys();
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
        initfsm = 1;
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
    private ToggleButton pipelineToggleButton = new ToggleButton(false);
    public static double hp = 0.8, hi = 0, hd = 0.00013;
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

    @Override
    public void loop() {
        diffyClawIntake.loop();
        diffyClawIntake.HoldExtension();
        ll.loop();
        outtakeLift.loop();
        if(intakeSequenceNextButton.input(gamepad2.left_bumper)||intakeSequenceNextButton2.input(gamepad1.left_bumper)){
            intakeSequence = intakeSequence.next();
            intakeSequenceTime.reset();
        } else if(intakeSequencePreviousButton.input(gamepad2.right_bumper)||intakeSequencePreviousButton2.input(gamepad1.right_bumper)){
            intakeSequence = intakeSequence.prev();
            intakeSequenceTime.reset();
        }

        if(headingLockButton.input(gamepad1.dpad_right)){
            targetHeading = follower.getPose().getHeading();
        }
        if(headingLockButton.getVal()){
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

            follower.setTeleOpMovementVectors(
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_y),
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_x), -headingCorrection);
        } else if (intakeSequence == INTAKE_SEQUENCE.TRANSFER_WAIT) {
            follower.setTeleOpMovementVectors(
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_y),
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_x),
                    0.25 * Math.tan(1.12 * -gamepad1.right_stick_x), true);
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
                diffyClawIntake.setClawOpen(Intake_DiffyClaw.CLAW_STATE.OPEN);
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
                    follower.setTeleOpMovementVectors((targetX - ll.getTx()) * mx, (targetY -  ll.getTy()) * my, 0);
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
                    diffyClawIntake.setClawOpen(Intake_DiffyClaw.CLAW_STATE.CLOSED);
                }
                if (intakeSequenceTime.time() > 0.4){
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
                    diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                }
                break;
            case TRANSFER_WAIT:
                ll.turnOff();
                if (intakeSequenceTime.time() > 0.3){
                    diffyClawIntake.setClawOpen(Intake_DiffyClaw.CLAW_STATE.LOOSE);
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
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.TRANSFER);
                        outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER);
                        outtake.setClawOpen(true);
                        break;
                    case GRAB_AND_LIFT:
                        diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER);
                        if (outtakeSequenceTime.time() > 0.23){
                            outtake.setClawOpen(false);
                        }
                        if (outtakeSequenceTime.time() > 0.74){
                            diffyClawIntake.setClawOpen(Intake_DiffyClaw.CLAW_STATE.OPEN);
                        }
                        if(outtakeSequenceTime.time() > 1){
                            outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.LIFT_BUCKET);
                            outtake.setOuttakeState(Outtake.OuttakeState.SAMPLESCORE);
                            resetEncoderDelay.reset();
                        }
                        break;
                    case SCORE:
                        outtake.setClawOpen(true);
                        if (resetEncoderDelay.time() > 0.8){
                            outtake.setOuttakeState(Outtake.OuttakeState.RESET_ENCODER);
                        }
                        break;
                    case RESET:
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.RESET_ENCODER);
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
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.AVOID_INTAKE);
                        if(avoidIntakeFsmTimer.time() > 1){
                            diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_REST);
                            avoidIntakeFsmTimer.reset();
                            avoidIntakeFsm = AVOID_INTAKE_FSM.MOVE_INTAKE;
                        }
                        break;
                    case MOVE_INTAKE:
                        if(avoidIntakeFsmTimer.time() > 1.5){
                            avoidIntakeFsm = AVOID_INTAKE_FSM.NOTHING;
                        }
                        break;
                    case NOTHING:
                        switch (backSpecimenSequence){
                            case OPEN_CLAW:
                                outtake.setClawOpen(true);
                                break;
                            case FRONT_GRAB:
                                outtake.setClawOpen(true);
                                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
                                outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
                                break;
                            case CLOSE_CLAW:
                                outtake.setClawOpen(false);
                                break;
                            case BACK_SCORE:
                                outtake.setClawOpen(false);
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
                        outtake.setClawOpen(true);
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.BACK_PICKUP);
                        outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKPICKUP);
                        if(intakeSequence == INTAKE_SEQUENCE.TRANSFER_WAIT){
                            diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.DEPOSIT);
                        }
                        break;
                    case CLOSE_CLAW:
                        outtake.setClawOpen(false);
                        if(intakeSequence == INTAKE_SEQUENCE.TRANSFER_WAIT){
                            diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.DEPOSIT);
                            diffyClawIntake.setClawOpen(Intake_DiffyClaw.CLAW_STATE.OPEN);
                        }
                        break;
                    case FRONT_SCORE:
                        outtake.setClawOpen(false);
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_SCORE_WAIT);
                        outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTSCORE);
                        break;
                    case OPEN_CLAW:
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_SCORE_DONE);
                        outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTSCORE);
                        if(outtakeSequenceTime.time() > 1){
                            outtake.setClawOpen(true);
                        }
                        break;
                }
                break;
            case ASCENT:
                if(intakeSequence == INTAKE_SEQUENCE.TRANSFER_WAIT && avoidIntakeFsm == AVOID_INTAKE_FSM.NOTHING && diffyClawIntake.intakeState != Intake_DiffyClaw.IntakeState.INTAKE_REST){
                    avoidIntakeFsm = AVOID_INTAKE_FSM.LIFT_SLIDES;
                    avoidIntakeFsmTimer.reset();
                }

                switch(avoidIntakeFsm){
                    case LIFT_SLIDES:
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.AVOID_INTAKE);
                        if(avoidIntakeFsmTimer.time() > 1){
                            diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_REST);
                            avoidIntakeFsmTimer.reset();
                            avoidIntakeFsm = AVOID_INTAKE_FSM.MOVE_INTAKE;
                        }
                        break;
                    case MOVE_INTAKE:
                        if(avoidIntakeFsmTimer.time() > 1.5){
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
                                break;
                            case SERVO_HOOKS:
                                hangServos.hang();
                                break;
                            case SLIDES_UP_HIGH:
                                outtakeLift.stopHang();
                                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.HIGH_BAR_WAIT);
                                break;
                            case SLIDES_UP_GO:
                                outtakeLift.useHang();
                                outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKPICKUP);
                                outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.HIGH_BAR_DONE);
                                if(hangTimer.time() > 0.5){
                                    hangServos.rest();
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

        tel.addData("Control:", controlFlipButton.getVal() ? "Normal" : "Flipped");
        tel.addData("Pipeline", pipelineToggleButton.getVal() ? "Yellow" : "Alliance");
        tel.addData("Target Heading in Degrees", Math.toDegrees(targetHeading));
        tel.addData("Angle Error in Degrees", Math.toDegrees(headingError));
        tel.addData("Correction Vector in Degrees", Math.toDegrees(headingCorrection));
        tel.addData("X", follower.getPose().getX());
        tel.addData("Y", follower.getPose().getY());
        tel.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        tel.addData("Elapsed Time", elapsedTime.toString());
        tel.update();
    }
}