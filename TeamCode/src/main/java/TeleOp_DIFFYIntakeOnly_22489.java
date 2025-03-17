

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import subsystems.UnifiedTelemetry;


@TeleOp(name = "Intake Only - TeleOp", group = "Real OpModes")
@Config
public class TeleOp_DIFFYIntakeOnly_22489 extends OpMode {
    public static double mx =  -0.01, my =  -0.01;
    public static double targetX = 7, targetY = 0;
    private Follower follower;

    private Intake_DiffyClaw diffyClawIntake;
    private ElapsedTime elapsedTime, intakeSequenceTime, resetEncoderDelay;
    private final Pose startPose = Storage.CurrentPose;
    private double targetHeading = 180, headingError, headingCorrection;
    int flip = 1;
    int initfsm = 0;
    public static String key = "sigmaboy";
    private UnifiedTelemetry tel;

    ToggleButton takeSnapshotButton = new ToggleButton(false);

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        elapsedTime = new ElapsedTime();
        intakeSequenceTime = new ElapsedTime();
        resetEncoderDelay = new ElapsedTime();
        diffyClawIntake = new Intake_DiffyClaw(hardwareMap);

        tel = new UnifiedTelemetry();
        tel.init(this.telemetry);

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
        diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_REST);

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
    private ToggleButton intakeSequenceNextButton = new ToggleButton(true), intakeSequencePreviousButton = new ToggleButton(true), ALignmentButtonNext = new ToggleButton(true),ALignmentButtonPrev = new ToggleButton(true);
    @Override
    public void loop() {
        LLResult result = diffyClawIntake.limelight.getLatestResult();
        if(intakeSequenceNextButton.input(gamepad1.left_bumper)){
            intakeSequence = intakeSequence.next();
            intakeSequenceTime.reset();
        } else if(intakeSequencePreviousButton.input(gamepad1.right_bumper)){
            intakeSequence = intakeSequence.prev();
            intakeSequenceTime.reset();
        }
        if (intakeSequence == INTAKE_SEQUENCE.TRANSFER_WAIT) {
            follower.setTeleOpMovementVectors(
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_y),
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_x),
                    0.21 * Math.tan(1.12 * -gamepad1.right_stick_x), true);
        } else { //if intake is down, then we slow down the driving.
            follower.setTeleOpMovementVectors(
                    flip * 0.20 * Math.tan(1.12 * -gamepad1.left_stick_y),
                    flip * 0.20 * Math.tan(1.12 * -gamepad1.left_stick_x),
                    0.1 * Math.tan(1.12 * -gamepad1.right_stick_x), true);

        }
        switch (intakeSequence){
            case READY:
                diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                diffyClawIntake.setExtensionTarget(Intake_DiffyClaw.FULL_EXTENSION);
                if (!diffyClawIntake.isExtensionBusy()){
                    diffyClawIntake.setClawOpen(true);
                }
                if (ALignmentButtonNext.input(gamepad1.left_trigger == 1)){
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED += 45;
                    if (Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED > 100){
                        Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = -100;
                    }
                } else if (ALignmentButtonPrev.input(gamepad1.right_trigger == 1)){
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED -= 45;
                    if (Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED < -100){
                        Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 100;
                    }
                }
                if (result != null && diffyClawIntake.limelight.isRunning()) {
                    double tx = result.getTx(); // How far left or right the target is (degrees)
                    double ty = result.getTy(); // How far up or down the target is (degrees)
                    double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                    if(ta > 4){ // Ignore small spots
                        follower.setTeleOpMovementVectors((targetX - tx) * mx, (targetY -  ty) * my, 0);
                        double angle = -result.getPythonOutput()[0]; // Output 0 is sample angle
                        if(Math.abs(angle) > 80){
                            if(Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED >= 0) angle = 80;
                            else angle = -80;
                        }
                        if(angle < -90) angle = -90;
                        else if(angle > 90) angle = 90;

                        Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = angle;
                    }
                }
                break;
            case GRAB:
                diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
                diffyClawIntake.setExtensionTarget(Intake_DiffyClaw.FULL_EXTENSION);
                if (intakeSequenceTime.time() > 0.2){
                    diffyClawIntake.setClawOpen(false);
                }
                if (intakeSequenceTime.time() > 0.4){
                    Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
                    diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
                }
                break;
            case TRANSFER_WAIT:
                diffyClawIntake.setExtensionTarget(Intake_DiffyClaw.TRANSFER_EXTENSION_POS);
                if (!diffyClawIntake.isExtensionBusy()){
                diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.TRANSFER_WAIT);}
                break;
        }
        if(takeSnapshotButton.input(gamepad1.a)){
            diffyClawIntake.limelight.captureSnapshot(key+Math.round(elapsedTime.time()));
        }

        diffyClawIntake.intakeLoop();

        controlFlipButton.input(gamepad1.dpad_up);
        flip = controlFlipButton.getVal() ? 1 : -1;



        follower.update();
        Storage.CurrentPose = follower.getPose();

        if (result != null) {
            telemetry.addData("Data Validity", result.isValid());
            telemetry.addData("Data Staleness", result.getStaleness());

            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)
            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);

            double[] pythonOutputs = result.getPythonOutput();
            if(pythonOutputs != null && pythonOutputs.length > 0){
                telemetry.addData("Python Output", pythonOutputs[0]);
            }
        } else {
            tel.addData("Limelight", "Null");
        }

        tel.addData("Control:", controlFlipButton.getVal() ? "Normal" : "Flipped");
        tel.addData("Target Heading in Degrees", Math.toDegrees(targetHeading));
        tel.addData("Angle Error in Degrees", Math.toDegrees(headingError));
        tel.addData("Correction Vector in Degrees", Math.toDegrees(headingCorrection));
        tel.addData("X", follower.getPose().getX());
        tel.addData("Y", follower.getPose().getY());
        tel.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        tel.addData("Elapsed Time", elapsedTime.toString());
        tel.addData("Horizontal Extension Target Position", Old_Intake_DoNotUse.extPos);
        tel.addData("Horizontal Extension Servo Angle", diffyClawIntake.leintake.getPosition());
        tel.addData("Horizontal Extension Target Reached?", diffyClawIntake.isExtensionBusy());
        tel.update();
    }
}