//
//
//import com.acmerobotics.dashboard.config.Config;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.util.Constants;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//import subsystems.IntakeLimelightSubsys;
//import subsystems.SubsysCore;
//import subsystems.UnifiedTelemetry;
//
//
//@TeleOp(name = "Intake Only - TeleOp", group = "Real OpModes")
//@Config
//public class TeleOp_DIFFYIntakeOnly_22489 extends OpMode {
//    public static double mx =  -0.01, my =  -0.01;
//    public static double targetX = 7, targetY = 0;
//    private Follower follower;
//
//    private subsystems.Intake_DiffyClaw diffyClawIntake;
//    private IntakeLimelightSubsys ll;
//    private ElapsedTime elapsedTime, intakeSequenceTime, resetEncoderDelay;
//    private final Pose startPose = Storage.CurrentPose;
//    private double targetHeading = 180, headingError, headingCorrection;
//    int flip = 1;
//    int initfsm = 0;
//    public static String key = "sigmaboy";
//    private UnifiedTelemetry tel;
//
//    ToggleButton takeSnapshotButton = new ToggleButton(false);
//
//    @Override
//    public void init() {
//        Constants.setConstants(FConstants.class, LConstants.class);
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//        elapsedTime = new ElapsedTime();
//        intakeSequenceTime = new ElapsedTime();
//        resetEncoderDelay = new ElapsedTime();
//
//        tel = new UnifiedTelemetry();
//        tel.init(this.telemetry);
//        SubsysCore.setGlobalParameters(hardwareMap, this);
//
//        diffyClawIntake = new subsystems.Intake_DiffyClaw();
//        ll = new IntakeLimelightSubsys();
//        ll.init();
//
//        intakeSequenceTime.startTime();
//        elapsedTime.startTime();
//        resetEncoderDelay.startTime();
//        initfsm = 1;
//    }
//
//    private ToggleButton teamColorButton = new ToggleButton(Storage.isRed);
//    private ToggleButton controlFlipButton = new ToggleButton(true);
//
//    @Override
//    public void init_loop(){
//        teamColorButton.input(gamepad1.dpad_up);
//        Storage.isRed = teamColorButton.getVal();
//        follower.update();
//        Storage.CurrentPose = follower.getPose();
//        telemetry.addLine("DO NOT TOUCH IF THIS IS REAL GAME, or make sure you dont misclick.");
//        telemetry.addData("Team Color:", Storage.isRed ? "Red" : "Blue");
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        follower.startTeleopDrive();
//        diffyClawIntake.init();
//    }
//    public enum INTAKE_SEQUENCE{
//        TRANSFER_WAIT, READY, GRAB;
//        private static final INTAKE_SEQUENCE[] vals = values();
//        public INTAKE_SEQUENCE next(){
//            return vals[(this.ordinal() + 1) % vals.length];
//        }
//        public INTAKE_SEQUENCE prev(){
//            return vals[(this.ordinal() - 1 + vals.length) % vals.length];
//        }
//    }
//    INTAKE_SEQUENCE intakeSequence = INTAKE_SEQUENCE.TRANSFER_WAIT;
//    private ToggleButton intakeSequenceNextButton = new ToggleButton(true), intakeSequencePreviousButton = new ToggleButton(true), ALignmentButtonNext = new ToggleButton(true),ALignmentButtonPrev = new ToggleButton(true), autoALignmentButton = new ToggleButton(true);
//    @Override
//    public void loop() {
//        ll.loop();
//        if(intakeSequenceNextButton.input(gamepad1.left_bumper)){
//            intakeSequence = intakeSequence.next();
//            intakeSequenceTime.reset();
//        } else if(intakeSequencePreviousButton.input(gamepad1.right_bumper)){
//            intakeSequence = intakeSequence.prev();
//            intakeSequenceTime.reset();
//        }
//        if (intakeSequence == INTAKE_SEQUENCE.TRANSFER_WAIT) {
//            follower.setTeleOpMovementVectors(
//                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_y),
//                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_x),
//                    0.48 * Math.tan(1.12 * -gamepad1.right_stick_x), true);
//        } else { //if intake is down, then we slow down the driving.
//            follower.setTeleOpMovementVectors(
//                    flip * 0.20 * Math.tan(1.12 * -gamepad1.left_stick_y),
//                    flip * 0.20 * Math.tan(1.12 * -gamepad1.left_stick_x),
//                    0.1 * Math.tan(1.12 * -gamepad1.right_stick_x), true);
//        }
//        switch (intakeSequence){
//            case READY:
//                diffyClawIntake.setIntakeState(subsystems.Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
//                diffyClawIntake.ExtendTo(subsystems.Intake_DiffyClaw.IntakeExtensionStates.FULL_EXTENSION);
//                ll.turnOn();
//
//                if (!diffyClawIntake.isExtensionBusy()){
//                    diffyClawIntake.setClawState(Outtake.ClawStates.OPEN);
//                }
//                if (ALignmentButtonNext.input(gamepad1.left_trigger == 1)){
//                    subsystems.Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED += 45;
//                    if (subsystems.Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED > 100){
//                        subsystems.Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = -100;
//                    }
//                } else if (ALignmentButtonPrev.input(gamepad1.right_trigger == 1)){
//                    subsystems.Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED -= 45;
//                    if (subsystems.Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED < -100){
//                        subsystems.Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 100;
//                    }
//                }
//                if (ll.isRunning() && ll.isResultValid() && gamepad1.right_stick_button) {
//                    follower.setTeleOpMovementVectors((targetX - ll.getTx()) * mx, (targetY -  ll.getTy()) * my, 0);
//                    double angle = -ll.getAngle(); // Output 0 is sample angle
//                    if(Math.abs(angle) > 80){
//                        if(subsystems.Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED >= 0) angle = 80;
//                        else angle = -80;
//                    }
//                    if(angle < -90) angle = -90;
//                    else if(angle > 90) angle = 90;
//
//                    subsystems.Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = angle;
//                }
//                break;
//            case GRAB:
//                diffyClawIntake.setIntakeState(subsystems.Intake_DiffyClaw.IntakeState.INTAKE_ARM_PICKUP);
//                ll.turnOff();
//                diffyClawIntake.ExtendTo(subsystems.Intake_DiffyClaw.IntakeExtensionStates.FULL_EXTENSION);
//                if (intakeSequenceTime.time() > 0.2){
//                    diffyClawIntake.setClawState(Outtake.ClawStates.CLOSED);
//                }
//                if (intakeSequenceTime.time() > 0.4){
//                    subsystems.Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
//                    diffyClawIntake.setIntakeState(subsystems.Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
//                }
//                break;
//            case TRANSFER_WAIT:
//                ll.turnOff();
//                diffyClawIntake.ExtendTo(subsystems.Intake_DiffyClaw.IntakeExtensionStates.RETRACTED);
//                if (!diffyClawIntake.isExtensionBusy()){
//                diffyClawIntake.setIntakeState(subsystems.Intake_DiffyClaw.IntakeState.TRANSFER_WAIT);}
//                break;
//        }
//        if(takeSnapshotButton.input(gamepad1.a)){
//            ll.captureSnapshot(key+Math.round(elapsedTime.time()));
//        }
//
//        diffyClawIntake.loop();
//        diffyClawIntake.HoldExtension();
//
//        controlFlipButton.input(gamepad1.dpad_up);
//        flip = controlFlipButton.getVal() ? 1 : -1;
//
//
//
//        follower.update();
//        Storage.CurrentPose = follower.getPose();
//
//        tel.addData("Control:", controlFlipButton.getVal() ? "Normal" : "Flipped");
//        tel.addData("Target Heading in Degrees", Math.toDegrees(targetHeading));
//        tel.addData("Angle Error in Degrees", Math.toDegrees(headingError));
//        tel.addData("Correction Vector in Degrees", Math.toDegrees(headingCorrection));
//        tel.addData("X", follower.getPose().getX());
//        tel.addData("Y", follower.getPose().getY());
//        tel.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
//        tel.addData("Elapsed Time", elapsedTime.toString());
//        tel.update();
//    }
//}