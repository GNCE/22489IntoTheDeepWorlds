

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@TeleOp(name = "Red TeleOp", group = "Real OpModes")
public class Teleop_26111 extends OpMode {
    private Follower follower;
    private Outtake outtake;
    private Intake intake;
    private OuttakeLift outtakeLift;
    private Misc misc;
    private final Pose startPose = PoseStorage.CurrentPose;
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap,this);
        outtakeLift = new OuttakeLift(hardwareMap, this);
        misc = new Misc(hardwareMap);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        intake.initiate();
        outtakeLift.HoldLift();
    }
    @Override
    public void loop() {

        if (gamepad1.left_bumper){
            intake.flipDown();
        }
        intake.TeleopExtend(); //left trigger
        //right bumper cancels flipdown
        if (gamepad1.right_trigger > 0.2){
            intake.deposit();
        }
        if (gamepad1.y){
            outtakeLift.LiftTarget(250);
            outtake.pivotToScoreSpecFront();
        }else if (gamepad1.b){
            outtakeLift.LiftTarget(100);
            outtake.pivotToPickup();
        }else if (gamepad1.x){
            outtakeLift.LiftTarget(250);
            outtake.pivotToScoreSpecBack();
        }else if (gamepad1.dpad_down){
            outtakeLift.LiftTarget(250);
            outtake.pivotToTransfer();
            misc.door();
        } else if (gamepad1.dpad_up){
            outtakeLift.LiftTarget(400);
            outtake.pivotToScoreSamp();
            misc.undoor();
        }
        if (gamepad1.dpad_left){
            outtake.openClaw();
        } else {
            outtake.closeClaw();
        }
        if (gamepad1.dpad_right){
            misc.sweep();
        } else {
            misc.unsweep();
        }

        outtake.updatePivPosition();
        intake.moveThings();
        misc.moveThings();
        follower.setTeleOpMovementVectors(
                0.48 * Math.tan(1.12 * -gamepad1.left_stick_y),
                0.48 * Math.tan(1.12 * -gamepad1.left_stick_x),
                -gamepad1.right_stick_x, true);
        follower.update();
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();

    }

}