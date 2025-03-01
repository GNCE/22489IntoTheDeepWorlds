

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@TeleOp(name = "Intake Only - TeleOp", group = "Real OpModes")
@Config
public class EC_TeleOp_DIFFYIntakeOnly extends OpMode {
    private Follower follower;

    private DiffyClawIntake diffyClawIntake;
    private ElapsedTime elapsedTime, intakeSequenceTime, resetEncoderDelay;
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
        resetEncoderDelay = new ElapsedTime();
        diffyClawIntake = new DiffyClawIntake(hardwareMap);

        intakeSequenceTime.startTime();
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
        diffyClawIntake.setIntakeState(DiffyClawIntake.IntakeState.INTAKE_REST);

    }
    public enum INTAKE_SEQUENCE{
        TRANSFER, READY, GRAB;
        private static final INTAKE_SEQUENCE[] vals = values();
        public INTAKE_SEQUENCE next(){
            return vals[(this.ordinal() + 1) % vals.length];
        }
        public INTAKE_SEQUENCE prev(){
            return vals[(this.ordinal() - 1 + vals.length) % vals.length];
        }
    }
    INTAKE_SEQUENCE intakeSequence = INTAKE_SEQUENCE.TRANSFER;
    private ToggleButton intakeSequenceNextButton = new ToggleButton(true), intakeSequencePreviousButton = new ToggleButton(true), ALignmentButtonNext = new ToggleButton(true),ALignmentButtonPrev = new ToggleButton(true);
    @Override
    public void loop() {
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
                    if (DiffyClawIntake.DIFFY_POSITIONS.ORIENTATION_ALIGNED > 90){
                        DiffyClawIntake.DIFFY_POSITIONS.ORIENTATION_ALIGNED = -45;
                    }
                } else if (ALignmentButtonPrev.input(gamepad1.right_trigger == 1)){
                    DiffyClawIntake.DIFFY_POSITIONS.ORIENTATION_ALIGNED -= 45;
                    if (DiffyClawIntake.DIFFY_POSITIONS.ORIENTATION_ALIGNED < -45){
                        DiffyClawIntake.DIFFY_POSITIONS.ORIENTATION_ALIGNED = 90;
                    }
                }
                break;
            case GRAB:
                diffyClawIntake.setIntakeState(DiffyClawIntake.IntakeState.INTAKE_ARM_PICKUP);
                diffyClawIntake.setExtensionTarget(DiffyClawIntake.FULL_EXTENSION);
                if (intakeSequenceTime.time() > 0.2){
                    diffyClawIntake.setClawOpen(false);
                }
                break;
            case TRANSFER:
                diffyClawIntake.setExtensionTarget(DiffyClawIntake.TRANSFER_EXTENSION_POS);
                DiffyClawIntake.DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
                if (!diffyClawIntake.isExtensionBusy()){
                diffyClawIntake.setIntakeState(DiffyClawIntake.IntakeState.TRANSFER);}
                break;
        }
        diffyClawIntake.intakeLoop();

        controlFlipButton.input(gamepad1.dpad_up);
        flip = controlFlipButton.getVal() ? 1 : -1;


        if (intakeSequence == INTAKE_SEQUENCE.TRANSFER) {
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