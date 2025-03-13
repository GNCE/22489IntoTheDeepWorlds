

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Disabled
@TeleOp(name = " OLD Main TeleOp", group = "Real OpModes")
@Config
public class EC_OLD_TeleOp extends OpMode {
    private Follower follower;
    private Outtake outtake;
    private Old_Intake_DoNotUse intake;
    private OuttakeLift outtakeLift;
    private Misc misc;
    private ElapsedTime elapsedTime, sequenceTime, resetEncoderDelay;
    private final Pose startPose = Storage.CurrentPose;
    private double targetHeading = 180, headingError, headingCorrection;
    int flip = 1;
    int initfsm = 0;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        outtake = new Outtake(hardwareMap);
        elapsedTime = new ElapsedTime();
        sequenceTime = new ElapsedTime();
        resetEncoderDelay = new ElapsedTime();
        intake = new Old_Intake_DoNotUse(hardwareMap,this);
        outtakeLift = new OuttakeLift(hardwareMap, this);
        misc = new Misc(hardwareMap);
        misc.initiate();

        sequenceTime.startTime();
        elapsedTime.startTime();
        resetEncoderDelay.startTime();
        initfsm = 1;
    }

    private ToggleButton teamColorButton = new ToggleButton(Storage.isRed);
    private ToggleButton controlFlipButton = new ToggleButton(true);

    @Override
    public void init_loop(){
//        outtake.loop();
//        switch (initfsm){
//            case 1:
//                outtakeLift.rlift1.setPower(.4);
//                outtakeLift.llift1.setPower(.4);
//                outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONT);
//                intake.setIntakeState(Intake.IntakeState.FLIP_UP);
//                if (elapsedTime.seconds() > 2){
//                    initfsm = 2;
//                }
//            break;
//            case 2:
//                outtakeLift.rlift1.setPower(-0.6);
//                outtakeLift.llift1.setPower(-0.6);
//                if (elapsedTime.seconds() > 4){
//                    initfsm = 3;
//                }
//            break;
//            case 3:
//                outtakeLift.rlift1.setPower(0);
//                outtakeLift.llift1.setPower(0);
//                initfsm = -1;
//            break;
//        }

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
        intake.initiate();
        follower.startTeleopDrive();
        outtake.setOuttakeState(Outtake.OuttakeState.RESET_ENCODER);
        intake.setIntakeState(Old_Intake_DoNotUse.IntakeState.TRANSFER);
    }

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
    }


    BUCKET_SEQUENCE bucketSequence = BUCKET_SEQUENCE.SCORE;
    SPECIMEN_SEQUENCE specimenSequence = SPECIMEN_SEQUENCE.OPEN_CLAW;
    OUTTAKE_SEQUENCE outtakeSequence = OUTTAKE_SEQUENCE.BUCKET_SEQUENCE;

    private ToggleButton bucketSequenceNextButton = new ToggleButton(true), bucketSequencePrevButton = new ToggleButton(true), specimenSequenceNextButton = new ToggleButton(true), specimenSequencePrevButton = new ToggleButton(true);
    private ToggleButton specimenHeadingLockButton = new ToggleButton(false);

    public static double hp = 0.01, hi = 0, hd = 0.001;
    PIDController headingPIDController = new PIDController(hp, hi, hd);

    @Override
    public void loop() {
        if (gamepad1.left_bumper) intake.setIntakeState(Old_Intake_DoNotUse.IntakeState.INTAKE);
        if (gamepad1.right_stick_button) intake.setIntakeState(Old_Intake_DoNotUse.IntakeState.SHOOT);
        if (gamepad1.left_stick_button && (bucketSequence != BUCKET_SEQUENCE.TRANSFER)){
            intake.setIntakeState(Old_Intake_DoNotUse.IntakeState.FLIP_UP);
            intake.startReverseIntake();
        }
        intake.TeleopExtend(gamepad1.left_trigger);
        if (gamepad1.right_trigger > 0.2) intake.setIntakeState(Old_Intake_DoNotUse.IntakeState.TRANSFER);
        if (gamepad1.right_bumper) intake.startReverseIntake();

        if(bucketSequenceNextButton.input(gamepad1.a)){
            outtakeSequence = OUTTAKE_SEQUENCE.BUCKET_SEQUENCE;
            bucketSequence = bucketSequence.next();
            specimenSequence = SPECIMEN_SEQUENCE.vals[SPECIMEN_SEQUENCE.vals.length-1];
            sequenceTime.reset();
        } else if(bucketSequencePrevButton.input(gamepad1.b)){
            outtakeSequence = OUTTAKE_SEQUENCE.BUCKET_SEQUENCE;
            bucketSequence = bucketSequence.prev();
            sequenceTime.reset();
        } else if(specimenSequenceNextButton.input(gamepad1.x)){
            outtakeSequence = OUTTAKE_SEQUENCE.SPECIMEN_SEQUENCE;
            specimenSequence = specimenSequence.next();
            bucketSequence = BUCKET_SEQUENCE.vals[BUCKET_SEQUENCE.vals.length-1];
            sequenceTime.reset();
        } else if(specimenSequencePrevButton.input(gamepad1.y)){
            outtakeSequence = OUTTAKE_SEQUENCE.SPECIMEN_SEQUENCE;
            specimenSequence = specimenSequence.prev();
            sequenceTime.reset();
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
                        outtake.setClawOpen(false);
                        if(sequenceTime.time() > 0.1){
                            intake.startReverseIntake();
                        }
                        if(sequenceTime.time() > 0.4){
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
                switch (specimenSequence){
                    case OPEN_CLAW:
                        outtake.setClawOpen(true);
                        break;
                    case FRONT_GRAB:
                            {
                        outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.FRONT_PICKUP);
                        outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);}
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
        }

//        if (gamepad2.y){
//            outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONT);
//        }else if (gamepad2.b){
//            //outtake.pivotToPickupBack();
//        }else if (gamepad2.x){
//            outtake.setOuttakeState(Outtake.OuttakeState.SAMPLESCORE);
//            outtakeLift.LiftTarget(750);
//            misc.undoor();
//        }else if (gamepad2.a){
//            outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER);
//            misc.door();
//        }else if (gamepad2.dpad_up){
//            outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCORE);
//        }
//
//        if (gamepad2.left_bumper){
//            outtake.setClawOpen(true);
//        } else {
//            outtake.setClawOpen(false);
//        }

        if (gamepad1.right_bumper) misc.startSweep();
        else misc.endSweep();

        outtakeLift.HoldLift();
        outtake.outtakeLoop();
        intake.intakeLoop();
        misc.loop();

        controlFlipButton.input(gamepad1.dpad_up);
        flip = controlFlipButton.getVal() ? 1 : -1;

        if(specimenHeadingLockButton.input(gamepad1.dpad_right)){
            targetHeading = follower.getPose().getHeading();
        }
        if(specimenHeadingLockButton.getVal()){
            headingError = targetHeading - follower.getPose().getHeading();
            headingError = Math.IEEEremainder(headingError + 2*Math.PI, 2*Math.PI);
            if(headingError > 2*Math.PI - headingError){
                headingError = headingError - 2*Math.PI;
            }

            if(Math.abs(headingError) < Math.toRadians(3)){
                headingCorrection = 0;
            } else {
                headingPIDController.setPID(hp, hi, hd);
                headingCorrection = headingPIDController.calculate(headingError);
            }

            follower.setTeleOpMovementVectors(
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_y),
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_x), -headingCorrection);
        } else {
            follower.setTeleOpMovementVectors(
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_y),
                    flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_x),
                    0.21 * Math.tan(1.12 * -gamepad1.right_stick_x), true);
        }
        follower.update();
        Storage.CurrentPose = follower.getPose();

        telemetry.addData("Control:", controlFlipButton.getVal() ? "Normal" : "Flipped");
        telemetry.addData("Heading Lock:", specimenHeadingLockButton.getVal() ? "Locked" : "Unlocked");
        telemetry.addData("Target Heading in Degrees", Math.toDegrees(targetHeading));
        telemetry.addData("Angle Error in Degrees", Math.toDegrees(headingError));
        telemetry.addData("Correction Vector in Degrees", Math.toDegrees(headingCorrection));
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addLine();
        telemetry.addData("Elapsed Time", elapsedTime.toString());
        telemetry.addLine();
        telemetry.addData("Lift Position",outtakeLift.getCurrentPosition());
        telemetry.addData("LLift1", outtakeLift.llift1.getCurrentPosition());
        telemetry.addData("LLift2", outtakeLift.llift2.getCurrentPosition());
        telemetry.addData("RLift1", outtakeLift.rlift1.getCurrentPosition());
        telemetry.addData("RLift2", outtakeLift.rlift2.getCurrentPosition());
        telemetry.addData("Lift Target", outtakeLift.getTargetPosition());

        telemetry.addData("Horizontal Extension Target Position", Old_Intake_DoNotUse.extPos);
        telemetry.addData("Horizontal Extension Servo Angle", intake.leintake.getPosition());
        telemetry.addLine()
                        .addData("TeleOp Outtake Sequence", outtakeSequence)
                        .addData("TeleOp Sample Sequence", bucketSequence)
                        .addData("TeleOp Specimen Sequence", specimenSequence);
        telemetry.update();
    }
}