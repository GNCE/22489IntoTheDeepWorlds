

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@TeleOp(name = "Red TeleOp", group = "Real OpModes")
public class Teleop_26111 extends OpMode {
    private Follower follower;
    private Outtake outtake;
    private Intake intake;
    private OuttakeLift outtakeLift;
    private Misc misc;
    private ElapsedTime elapsedTime;
    private final Pose startPose = PoseStorage.CurrentPose;
    private int transferSimpleFSM = 0;
    private int transferRealFSM = 0;
    private int simpletoggleTransfer = 0;
    int flip = 1;
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
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        intake.initiate();
        misc.initiate();
        elapsedTime.startTime();
    }
    public void pickupTransfer(){
        switch (transferRealFSM){
            case 1:
                outtake.openClaw();
                if (elapsedTime.seconds()>.5){
                outtakeLift.LiftTarget(252);
                outtakeLift.GetLiftPos();
                if (gamepad2.dpad_left){
                    transferRealFSM = 0;
                }
                if (outtakeLift.GotLiftPos <=270 && elapsedTime.seconds() > 1.5){
                    elapsedTime.reset();
                    transferRealFSM = 2;
                }}
                break;
            case 2:
                outtake.closeClaw();
                if (elapsedTime.seconds()>1){
                outtake.pivotToScoreSamp();
                outtakeLift.LiftTarget(750);
                transferRealFSM = 0;
                transferSimpleFSM =0;
                }
                break;
        }
    }
    @Override
    public void loop() {
        outtakeLift.HoldLift();
        if (gamepad1.left_bumper){
            intake.flipDown();
        }
        intake.check();
        intake.TeleopExtend(); //left trigger
        if (gamepad1.right_trigger > 0.2){
            intake.flipUp();
            intake.deposit();
        }
        if ((transferRealFSM == 0)){
            if (gamepad2.y){
                outtake.pivotToFront();
                transferSimpleFSM = 0;
            }else if (gamepad2.b){
                outtake.pivotToPickupBack();
                transferSimpleFSM = 0;
            }else if (gamepad2.x){
                outtake.pivotToScoreSamp();
                outtakeLift.LiftTarget(750);
                transferSimpleFSM = 0;
                misc.undoor();
            }else if ((gamepad2.dpad_up || gamepad2.a) && transferSimpleFSM ==0){
                outtake.pivotToTransfer();
                transferSimpleFSM = 1;

                misc.door();
            }else if (gamepad2.dpad_down && transferSimpleFSM ==1){
                elapsedTime.reset();
                transferRealFSM = 1; //switches pickup transfer on
            }
            if (gamepad2.left_bumper){
                outtake.openClaw();
            } else {
                outtake.closeClaw();
            }
        }
        if (gamepad1.right_bumper){
            misc.sweep();
        } else {
            misc.unsweep();
        }
        pickupTransfer();//check if it is switched on
        outtake.updatePivPosition();
        intake.moveThings();
        misc.moveThings();
        if (gamepad1.dpad_up){
            flip = 1;
        } else if (gamepad1.dpad_down){
            flip = -1;
        }
        follower.setTeleOpMovementVectors(
                flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_y),
                flip * 0.48 * Math.tan(1.12 * -gamepad1.left_stick_x),
                 0.28 * Math.tan(1.12 * -gamepad1.right_stick_x), true);
        follower.update();
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("transferstate", transferRealFSM);
        telemetry.addData("liftpos",outtakeLift.GotLiftPos);
        telemetry.update();

    }

}