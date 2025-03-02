

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.prefs.BackingStoreException;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@TeleOp(name = "Full Main - TeleOp", group = "Real OpModes")
@Config
public class EC_TeleOp_Full extends OpMode {
    private Follower follower;
    private Outtake outtake;
    private OuttakeLift outtakeLift;
    private DiffyClawIntake diffyClawIntake;
    private ElapsedTime elapsedTime, intakeSequenceTime, resetEncoderDelay, outtakeSequenceTime;
    private final Pose startPose = Storage.CurrentPose;
    private double targetHeading = 180, headingError, headingCorrection;
    int flip = 1;
    int initfsm = 0;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        elapsedTime = new ElapsedTime();
        intakeSequenceTime = new ElapsedTime();
        outtakeSequenceTime = new ElapsedTime();
        resetEncoderDelay = new ElapsedTime();
        diffyClawIntake = new DiffyClawIntake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        outtakeLift = new OuttakeLift(hardwareMap,this);
        intakeSequenceTime.startTime();
        outtakeSequenceTime.startTime();
        elapsedTime.startTime();
        resetEncoderDelay.startTime();
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
        outtake.setOuttakeState(Outtake.OuttakeState.RESET_ENCODER);
        diffyClawIntake.setIntakeState(DiffyClawIntake.IntakeState.INTAKE_REST);

    }

    // ***** ------------------ INTAKE SEQUENCE ------------------------- ***** \\
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
    private ToggleButton intakeSequenceNextButton = new ToggleButton(true), intakeSequencePreviousButton = new ToggleButton(true), ALignmentButtonNext = new ToggleButton(true),ALignmentButtonPrev = new ToggleButton(true);

    // ***** ------------ OUTTAKE SEQUENCE ------------- ***** \\

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

    private boolean isTransfering = false;
    private boolean isScoringSpecs = false;
    @Override
    public void loop() {

        // ***** -------------- INTAKE SEQUENCE ------------- ***** \\
        if(intakeSequenceNextButton.input(gamepad1.left_bumper)){
            intakeSequence = intakeSequence.next();
            intakeSequenceTime.reset();
        } else if(intakeSequencePreviousButton.input(gamepad1.right_bumper)){
            intakeSequence = intakeSequence.prev();
            intakeSequenceTime.reset();
        }
        switch (intakeSequence){
            case READY:
                diffyClawIntake.setIntakeState(DiffyClawIntake.IntakeState.INTAKE_ARM_READY);
                diffyClawIntake.setExtensionTarget(DiffyClawIntake.FULL_EXTENSION);
                if (!diffyClawIntake.isExtensionBusy()){
                    diffyClawIntake.setClawOpen(true);
                }
                if (ALignmentButtonNext.input(gamepad1.left_trigger == 1)){
                    DiffyClawIntake.DIFFY_POSITIONS.ORIENTATION_ALIGNED += 45;
                    if (DiffyClawIntake.DIFFY_POSITIONS.ORIENTATION_ALIGNED > 100){
                        DiffyClawIntake.DIFFY_POSITIONS.ORIENTATION_ALIGNED = -45;
                    }
                } else if (ALignmentButtonPrev.input(gamepad1.right_trigger == 1)){
                    DiffyClawIntake.DIFFY_POSITIONS.ORIENTATION_ALIGNED -= 45;
                    if (DiffyClawIntake.DIFFY_POSITIONS.ORIENTATION_ALIGNED < -45){
                        DiffyClawIntake.DIFFY_POSITIONS.ORIENTATION_ALIGNED = 100;
                    }
                }
                break;
            case GRAB:
                diffyClawIntake.setIntakeState(DiffyClawIntake.IntakeState.INTAKE_ARM_PICKUP);
                diffyClawIntake.setExtensionTarget(DiffyClawIntake.FULL_EXTENSION);
                if (intakeSequenceTime.time() > 0.2){
                    diffyClawIntake.setClawOpen(false);
                }
                if (intakeSequenceTime.time() > 0.4){
                    DiffyClawIntake.DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
                    diffyClawIntake.setIntakeState(DiffyClawIntake.IntakeState.INTAKE_ARM_READY);
                }
                break;
            case TRANSFER_WAIT:
                diffyClawIntake.setExtensionTarget(DiffyClawIntake.TRANSFER_EXTENSION_POS);
                if (!diffyClawIntake.isExtensionBusy() && !isTransfering && !isScoringSpecs){
                    diffyClawIntake.setIntakeState(DiffyClawIntake.IntakeState.TRANSFER_WAIT);}
                break;
        }
        diffyClawIntake.intakeLoop();

        // ***** -------------- OUTTAKE SEQUENCE --------------- ***** \\
        if(bucketSequenceNextButton.input(gamepad1.a)){
            if (isScoringSpecs){
                outtakeSequence = OUTTAKE_SEQUENCE.OVERRIDE_TO_INTAKE;
                isScoringSpecs = false;
            } else {
                outtakeSequence = OUTTAKE_SEQUENCE.BUCKET_SEQUENCE;
            }
            bucketSequence = bucketSequence.next();
            specimenSequence = SPECIMEN_SEQUENCE.vals[SPECIMEN_SEQUENCE.vals.length-1];
            outtakeSequenceTime.reset();
        } else if(bucketSequencePrevButton.input(gamepad1.b)){
            if (isScoringSpecs){
                outtakeSequence = OUTTAKE_SEQUENCE.OVERRIDE_TO_INTAKE;
                isScoringSpecs = false;
            } else {
                outtakeSequence = OUTTAKE_SEQUENCE.BUCKET_SEQUENCE;
            }
            bucketSequence = bucketSequence.prev();
            outtakeSequenceTime.reset();
        } else if(specimenSequenceNextButton.input(gamepad1.x)){
            if (!isScoringSpecs) {
                isScoringSpecs = true;
                outtakeSequence = OUTTAKE_SEQUENCE.OVERRIDE_TO_SPEC;

            } else {
                outtakeSequence = OUTTAKE_SEQUENCE.SPECIMEN_SEQUENCE;
            }
            specimenSequence = specimenSequence.next();
            bucketSequence = BUCKET_SEQUENCE.vals[BUCKET_SEQUENCE.vals.length-1];
            outtakeSequenceTime.reset();
        } else if(specimenSequencePrevButton.input(gamepad1.y)){
            outtakeSequence = OUTTAKE_SEQUENCE.SPECIMEN_SEQUENCE;
            specimenSequence = specimenSequence.prev();
            outtakeSequenceTime.reset();
        }

        switch(outtakeSequence){
            case BUCKET_SEQUENCE:
                switch (bucketSequence){
                    case TRANSFER:
                        outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.TRANSFER);
                        outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER);
                        outtake.setClawOpen(true);
                        break;
                    case GRAB_AND_LIFT:
                        isTransfering = true;
                        diffyClawIntake.setIntakeState(DiffyClawIntake.IntakeState.TRANSFER);
                        if (outtakeSequenceTime.time() > 0.23){
                            outtake.setClawOpen(false);
                        }
                        if (outtakeSequenceTime.time() > 0.74){
                            diffyClawIntake.setClawOpen(true);
                        }
                        if(outtakeSequenceTime.time() > 1){
                            outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.LIFT_BUCKET);
                            outtake.setOuttakeState(Outtake.OuttakeState.SAMPLESCORE);
                            resetEncoderDelay.reset();
                        }
                        break;
                    case SCORE:
                        outtake.setClawOpen(true);
                        if (resetEncoderDelay.time() > 0.4){
                            outtake.setOuttakeState(Outtake.OuttakeState.RESET_ENCODER);
                        }
                        if ((resetEncoderDelay.time() > 0.6) && outtakeLift.target != 30){
                            outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.RESET_ENCODER);
                        }
                        break;


                }
                break;
            case SPECIMEN_SEQUENCE:
                isScoringSpecs = true;
                switch (specimenSequence){
                    case OPEN_CLAW:
                        outtake.setClawOpen(true);
                        break;
                    case FRONT_GRAB:
                        outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.FRONT_PICKUP);
                        outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
                    break;
                    case CLOSE_CLAW:
                        outtake.setClawOpen(false);
                        break;
                    case BACK_SCORE:
                        outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.BACK_SCORE);
                        outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCORE);
                        break;
                }
                break;
            case OVERRIDE_TO_SPEC:
                outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.AVOID_INTAKE);
                if (!outtakeLift.isBusy()){
                    diffyClawIntake.setIntakeState(DiffyClawIntake.IntakeState.INTAKE_REST);
                    outtakeSequence = OUTTAKE_SEQUENCE.SPECIMEN_SEQUENCE;
                }
                break;
            case OVERRIDE_TO_INTAKE:
                outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.AVOID_INTAKE);
                if (!outtakeLift.isBusy()){
                    diffyClawIntake.setIntakeState(DiffyClawIntake.IntakeState.TRANSFER_WAIT);
                    outtakeSequence = OUTTAKE_SEQUENCE.BUCKET_SEQUENCE;
                }
                break;
        }

        outtakeLift.HoldLift();
        outtake.outtakeLoop();


        // ***** ------------- Drive Control and Telemetry ------------ ***** \\
        controlFlipButton.input(gamepad1.dpad_up);
        flip = controlFlipButton.getVal() ? 1 : -1;


        if (intakeSequence == INTAKE_SEQUENCE.TRANSFER_WAIT) {
            follower.setTeleOpMovementVectors(
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_y),
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_x),
                    0.21 * Math.tan(1.12 * -gamepad1.right_stick_x), true);
        } else { //if intake is down, then we slow down the driving.
            follower.setTeleOpMovementVectors(
                    flip * 0.42 * Math.tan(1.12 * -gamepad1.left_stick_y),
                    flip * 0.42 * Math.tan(1.12 * -gamepad1.left_stick_x),
                    0.135 * Math.tan(1.12 * -gamepad1.right_stick_x), true);

        }
        follower.update();
        Storage.CurrentPose = follower.getPose();

        telemetry.addData("Control:", controlFlipButton.getVal() ? "Normal" : "Flipped");
        telemetry.addData("Target Heading in Degrees", Math.toDegrees(targetHeading));
        telemetry.addData("Angle Error in Degrees", Math.toDegrees(headingError));
        telemetry.addData("Correction Vector in Degrees", Math.toDegrees(headingCorrection));
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addLine();
        telemetry.addData("Elapsed Time", elapsedTime.toString());
        telemetry.addLine();
        telemetry.addData("Horizontal Extension Target Position", Intake.extPos);
        telemetry.addData("Horizontal Extension Servo Angle", diffyClawIntake.leintake.getPosition());

        telemetry.update();
    }
}