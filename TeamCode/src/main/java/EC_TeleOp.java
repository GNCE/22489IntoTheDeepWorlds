

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@TeleOp(name = "Main TeleOp", group = "Real OpModes")
public class EC_TeleOp extends OpMode {
    private Follower follower;
    private Outtake outtake;
    private Intake intake;
    private OuttakeLift outtakeLift;
    private Misc misc;
    private ElapsedTime elapsedTime;
    private final Pose startPose = Storage.CurrentPose;
    int flip = 1;
    int initfsm = 0;
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        outtake = new Outtake(hardwareMap);
        elapsedTime = new ElapsedTime();
        intake = new Intake(hardwareMap,this);
        outtakeLift = new OuttakeLift(hardwareMap, this);
        //misc = new Misc(hardwareMap);
        //misc.initiate();

        elapsedTime.startTime();
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
        telemetry.addLine("DO NOT TOUCH IF THIS IS REAL GAME, or make sure you dont misclick.");
        telemetry.addData("Team Color:", Storage.isRed ? "Red" : "Blue");
        telemetry.update();
    }

    @Override
    public void start() {
        intake.initiate();
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) intake.setIntakeState(Intake.IntakeState.INTAKE);
        if (gamepad1.right_stick_button) intake.setIntakeState(Intake.IntakeState.SHOOT);
        intake.TeleopExtend(gamepad1.left_trigger);
        if (gamepad1.right_trigger > 0.2) intake.setIntakeState(Intake.IntakeState.TRANSFER);
        if (gamepad1.right_bumper) intake.startReverseIntake();

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

//        if (gamepad1.right_bumper) misc.startSweep();

        outtakeLift.HoldLift();
        //outtake.loop();
        intake.intakeLoop();
//        misc.loop();

        controlFlipButton.input(gamepad1.dpad_up);
        flip = controlFlipButton.getVal() ? 1 : -1;
        // Movement Speed Adjustments for Intake
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
//        if (intake.isIntakeDown()) {
//            follower.setTeleOpMovementVectors(
//                    flip * 0.42 * Math.tan(1.12 * -gamepad1.left_stick_y),
//                    flip * 0.42 * Math.tan(1.12 * -gamepad1.left_stick_x),
//                    0.26 * Math.tan(1.12 * -gamepad1.right_stick_x), true);
//        } else {
//            follower.setTeleOpMovementVectors(
//                    flip * 0.42 * Math.tan(1.12 * -gamepad1.left_stick_y),
//                    flip * 0.42 * Math.tan(1.12 * -gamepad1.left_stick_x),
//                    -gamepad1.right_stick_x, true);
//        }
        follower.update();

        telemetry.addData("Control:", controlFlipButton.getVal() ? "Normal" : "Flipped");
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addLine();
        telemetry.addData("Elapsed Time", elapsedTime.toString());
        telemetry.addLine();
        telemetry.addData("lift position",outtakeLift.rlift1.getCurrentPosition());
        telemetry.addData("extendo target position", Intake.extPos);
        telemetry.addData("extendo position", intake.leintake.getPosition());
        telemetry.update();
    }
}