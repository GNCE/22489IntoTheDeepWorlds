

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import subsystems.DriveSubsys;
import subsystems.IntakeLimelightSubsys;
import subsystems.OuttakeLiftSubsys;
import subsystems.SubsysCore;
import subsystems.UnifiedTelemetry;


@TeleOp(name = "new teleop", group = "Real OpModes")
@Config
public class newfull_Tele_Op_22489 extends OpMode {
    public static double mx =  -0.012, my =  -0.012;
    public static double targetX = 14, targetY = 0;
    private Follower follower;

    private Intake_DiffyClaw diffyClawIntake;
    private DriveSubsys driveSubsys;
    private IntakeLimelightSubsys ll;
    private OuttakeLiftSubsys outtakeLift;
    private Outtake outtake;
    private ElapsedTime elapsedTime, intakeSequenceTime, resetEncoderDelay, outtakeSequenceTime;
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

        intakeSequenceTime.startTime();
        elapsedTime.startTime();
        resetEncoderDelay.startTime();
        outtakeSequenceTime.startTime();
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

    public enum SPECIMEN_SEQUENCE {
        OPEN_CLAW, FRONT_GRAB, CLOSE_CLAW, BACK_SCORE;
        private static final SPECIMEN_SEQUENCE[] vals = values();

        public SPECIMEN_SEQUENCE next() {
            return vals[(this.ordinal() + 1) % vals.length];
        }

        public SPECIMEN_SEQUENCE prev() {
            return vals[(this.ordinal() - 1 + vals.length) % vals.length];
        }
    }

    enum OUTTAKE_SEQUENCE {
        BUCKET_SEQUENCE,
        SPECIMEN_SEQUENCE,
        OVERRIDE_TO_SPEC,
        OVERRIDE_TO_INTAKE
    }


    BUCKET_SEQUENCE bucketSequence = BUCKET_SEQUENCE.SCORE;
    SPECIMEN_SEQUENCE specimenSequence = SPECIMEN_SEQUENCE.OPEN_CLAW;
    OUTTAKE_SEQUENCE outtakeSequence = OUTTAKE_SEQUENCE.BUCKET_SEQUENCE;

    private ToggleButton bucketSequenceNextButton = new ToggleButton(true), bucketSequencePrevButton = new ToggleButton(true), specimenSequenceNextButton = new ToggleButton(true), specimenSequencePrevButton = new ToggleButton(true);
    private ToggleButton intakeSequenceNextButton2 = new ToggleButton(true), intakeSequencePreviousButton2 = new ToggleButton(true);
    private boolean isTransfering = false;
    private boolean isScoringSpecs = false;
    @Override
    public void loop() {
        diffyClawIntake.loop();
        diffyClawIntake.HoldExtension();
        ll.loop();
        outtakeLift.loop();
        diffyClawIntake.changePipeline(4);
        if(intakeSequenceNextButton.input(gamepad2.left_bumper)||intakeSequenceNextButton2.input(gamepad1.left_bumper)){
            intakeSequence = intakeSequence.next();
            intakeSequenceTime.reset();
        } else if(intakeSequencePreviousButton.input(gamepad2.right_bumper)||intakeSequencePreviousButton2.input(gamepad1.right_bumper)){
            intakeSequence = intakeSequence.prev();
            intakeSequenceTime.reset();
        }
        if (intakeSequence == INTAKE_SEQUENCE.TRANSFER_WAIT) {
            follower.setTeleOpMovementVectors(
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_y),
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_x),
                    0.25 * Math.tan(1.12 * -gamepad1.right_stick_x), true);
        } else { //if intake is down, then we slow down the driving.
            follower.setTeleOpMovementVectors(
                    flip * 0.30 * Math.tan(1.12 * -gamepad1.left_stick_y),
                    flip * 0.30 * Math.tan(1.12 * -gamepad1.left_stick_x),
                    0.2 * Math.tan(1.12 * -gamepad1.right_stick_x), true);
        }
        switch (intakeSequence){
            case READY:
                diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                diffyClawIntake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.FULL_EXTENSION);
                ll.turnOn();

                if (!diffyClawIntake.isExtensionBusy()){
                    diffyClawIntake.setClawOpen(true);
                }
                if (ALignmentButtonNext.input(gamepad1.right_trigger == 1)){
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED += 45;
                    if (Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED > 100){
                        Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = -100;
                    }
                } else if (ALignmentButtonPrev.input(gamepad1.left_trigger == 1)){
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED -= 45;
                    if (Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED < -100){
                        Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 100;
                    }
                }
                if (ll.isRunning() && ll.isResultValid() && gamepad1.right_stick_button) {
                    follower.setTeleOpMovementVectors((targetX - ll.getTx()) * mx, (targetY -  ll.getTy()) * my, 0);
                    double angle = ll.getAngle(); // Output 0 is sample angle
                    if(Math.abs(angle) > 80){
                        if(Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED >= 0) angle = 80;
                        else angle = -80;
                    }
                    if(angle < -90) angle = -90;
                    else if(angle > 90) angle = 90;

                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = angle;
                }
                break;
            case GRAB:
                diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
                ll.turnOff();
                diffyClawIntake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.FULL_EXTENSION);
                if (intakeSequenceTime.time() > 0.2){
                    diffyClawIntake.setClawOpen(false);
                }
                if (intakeSequenceTime.time() > 0.4){
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
                    diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                }
                break;
            case TRANSFER_WAIT:
                ll.turnOff();
                diffyClawIntake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
                if (!diffyClawIntake.isExtensionBusy()){
                    if (!isScoringSpecs){
                    diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER_WAIT);}
                    else{
                        diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_REST);
                    }
                }

                break;
        }
        if(takeSnapshotButton.input(gamepad1.a)){
            ll.captureSnapshot(key+Math.round(elapsedTime.time()));
        }
        //outtake stuff
        if(bucketSequenceNextButton.input(gamepad2.a)){
                outtakeSequence = OUTTAKE_SEQUENCE.BUCKET_SEQUENCE;
            bucketSequence = bucketSequence.next();
            specimenSequence = SPECIMEN_SEQUENCE.vals[SPECIMEN_SEQUENCE.vals.length-1];
            outtakeSequenceTime.reset();
        } else if(bucketSequencePrevButton.input(gamepad2.b)){
                outtakeSequence = OUTTAKE_SEQUENCE.BUCKET_SEQUENCE;
            bucketSequence = bucketSequence.prev();
            outtakeSequenceTime.reset();
        } else if(specimenSequenceNextButton.input(gamepad2.x)){
                outtakeSequence = OUTTAKE_SEQUENCE.SPECIMEN_SEQUENCE;
            specimenSequence = specimenSequence.next();
            bucketSequence = BUCKET_SEQUENCE.vals[BUCKET_SEQUENCE.vals.length-1];
            outtakeSequenceTime.reset();
        } else if(specimenSequencePrevButton.input(gamepad2.y)){
            outtakeSequence = OUTTAKE_SEQUENCE.SPECIMEN_SEQUENCE;
            specimenSequence = specimenSequence.prev();
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
                        isTransfering = true;
                        diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER);
                        if (outtakeSequenceTime.time() > 0.23){
                            outtake.setClawOpen(false);
                        }
                        if (outtakeSequenceTime.time() > 0.74){
                            diffyClawIntake.setClawOpen(true);
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
                        if (outtakeLift.getCurrentPosition() != 30){
                            outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.RESET_ENCODER);
                        }
                        break;


                }
                break;
            case SPECIMEN_SEQUENCE:
                switch (specimenSequence){
                    case OPEN_CLAW:
                        outtake.setClawOpen(true);
                        isScoringSpecs = true;
                        break;
                    case FRONT_GRAB:
                        isScoringSpecs = true;
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.FRONT_PICKUP);
                        outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
                        break;
                    case CLOSE_CLAW:
                        isScoringSpecs = true;
                        outtake.setClawOpen(false);
                        break;
                    case BACK_SCORE:
                        isScoringSpecs = false;
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.BACK_SCORE);
                        outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCORE);
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