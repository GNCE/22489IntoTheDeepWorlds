

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
    boolean tranfer = false;
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        outtake = new Outtake(hardwareMap);
        elapsedTime = new ElapsedTime();
        intake = new Intake(hardwareMap,this);
        outtakeLift = new OuttakeLift(hardwareMap, this);
        misc = new Misc(hardwareMap);
        misc.initiate();

        elapsedTime.startTime();
        initfsm = 1;
    }

    private ToggleButton teamColorButton = new ToggleButton(Storage.isRed);
    private ToggleButton controlFlipButton = new ToggleButton(true);

    @Override
    public void init_loop(){
        outtake.loop();
        switch (initfsm){
            case 1:
                outtakeLift.rlift.setPower(.4);
                outtakeLift.llift.setPower(.4);
                outtake.pivotToFront();
                intake.flipUp();
                intake.extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intake.extendo.setPower(-0.2);
                if (elapsedTime.seconds() > 2){
                    initfsm = 2;
                }
            break;
            case 2:
                outtakeLift.rlift.setPower(-0.6);
                outtakeLift.llift.setPower(-0.6);
                if (elapsedTime.seconds() > 4){
                    initfsm = 3;
                }
            break;
            case 3:
                outtakeLift.rlift.setPower(0);
                outtakeLift.llift.setPower(0);
                intake.extendo.setPower(0);
                initfsm = -1;
            break;
        }

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
        resetRuntime();
    }

    private static double prevTime;
    private int transferFSM = -1;
    public void TransferFSM(){
        switch (transferFSM){
            case 0:
                elapsedTime.reset();
                transferFSM = 1;
                break;
            case 1://reset stuff
                outtakeLift.LiftTarget(600);
                if(!outtakeLift.isBusy()){
                    outtake.pivotToFront();
                    elapsedTime.reset();
                    transferFSM = 2;
                }
                break;
            case 2:
                if (elapsedTime.seconds() >1){
                    outtakeLift.LiftTarget(-100);
                    if (outtakeLift.touchSensor.isPressed()){
                        transferFSM = -1;}

                }
                break;
            case 3:
                elapsedTime.reset();
                transferFSM = 4;
                break;
            case 4:
                outtake.pivotToTransfer();
                outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.TRANSFER_WAIT);
                outtake.setClaw(true);
                if (elapsedTime.seconds() > 1){
                    elapsedTime.reset();
                    transferFSM = 5;
                }
                break;
            case 5:
                outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.TRANSFER_GRAB);
                if (!outtakeLift.isBusy()){
                    outtake.setClaw(false);
                    elapsedTime.reset();
                    transferFSM = 6;
                }
                break;
            case 6:
                if (elapsedTime.seconds() > .3){
                    outtake.pivotToScoreSamp();
                    outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.LIFT_BUCKET);
                    transferFSM = -1;
                }
                break;
            default:
                break;


        }
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) intake.startIntake();
        if (gamepad1.right_stick_button) intake.shootOut();
        intake.TeleopExtend(); //left trigger
        if (gamepad1.right_trigger > 0.2) intake.depositandflip();
        if (gamepad2.dpad_up){
            outtake.pivotToFront();
            tranfer = false;
        }else if (gamepad2.dpad_right){
            outtake.pivotToPickupBack();
            tranfer = false;
        }else if (gamepad2.x){
            outtake.setClaw(false);
            outtake.pivotToScoreSamp();
            outtakeLift.LiftTarget(750);
            misc.undoor();
            tranfer = false;
        } else if (gamepad2.a){
            outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.TRANSFER_WAIT);
            outtake.pivotToTransfer();
            misc.door();
        } else if (gamepad2.b){
            transferFSM = 3;
        } else if (gamepad2.dpad_left){
            outtake.pivotToScoreSpecBack();
        } else if (gamepad2.y){
            transferFSM = 0;
        }

        if (gamepad2.left_bumper && !tranfer){
            outtake.setClaw(true);
        } else {
            outtake.setClaw(false);
        }

        if (gamepad1.right_bumper) misc.setSweep(true);
        else{
            if(misc.reachedSweepTarget()) misc.setSweep(false);
        }
        TransferFSM();
        outtakeLift.HoldLift();
        outtake.loop();
        intake.intakeLoop();
        intake.extendoLoop();
        misc.loop();

        controlFlipButton.input(gamepad1.dpad_up);
        flip = controlFlipButton.getVal() ? 1 : -1;
        follower.setTeleOpMovementVectors(
                flip * 0.68 * Math.tan(1.12 * -gamepad1.left_stick_y),
                flip * 0.68 * Math.tan(1.12 * -gamepad1.left_stick_x),
                 0.22 * Math.tan(1.12 * -gamepad1.right_stick_x), true);
        follower.update();

        telemetry.addLine()
                        .addData("Intake State", intake.intakeState.name());
        telemetry.addLine()
                        .addData("Detected Color", intake.getSensedColorName());
        telemetry.addLine()
                .addData("Red", "%.3f", intake.sensedColor.red)
                .addData("Green", "%.3f", intake.sensedColor.green)
                .addData("Blue", "%.3f", intake.sensedColor.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", intake.hsvValues[0])
                .addData("Saturation", "%.3f", intake.hsvValues[1])
                .addData("Value", "%.3f", intake.hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", intake.sensedColor.alpha);

        if(intake.colorSensor instanceof DistanceSensor){
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) intake.colorSensor).getDistance(DistanceUnit.CM));
        }

        telemetry.addData("Control:", controlFlipButton.getVal() ? "Normal" : "Flipped");
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addLine();
        telemetry.addData("Run Time", "%.3f", getRuntime());
        telemetry.addLine()
                        .addData("Loop Time", elapsedTime.seconds() - prevTime);
        prevTime = elapsedTime.seconds();
        telemetry.addLine();
        telemetry.addData("lift position",outtakeLift.rlift.getCurrentPosition());
        telemetry.addData("extendo position", intake.extendo.getCurrentPosition());
        telemetry.addData("Touch Sensor Pressed?", outtakeLift.touchSensor.isPressed());
        telemetry.update();
    }
}