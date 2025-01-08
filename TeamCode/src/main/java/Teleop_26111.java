

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
    private final Pose startPose = PoseStorage.CurrentPose;
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap,this);
        outtakeLift = new OuttakeLift(hardwareMap, this);
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
        intake.TeleopExtend();
        //right bumper cancels flipdown
        if (gamepad1.right_trigger > 0.2){
            intake.deposit();
        }
        if (gamepad1.y){
            outtakeLift.LiftTarget(250);
            outtake.pivotToScoreSpec();
        }else if (gamepad1.b){
            outtakeLift.LiftTarget(100);
            outtake.pivotToPickup();
        }else if (gamepad1.x){
            outtakeLift.LiftTarget(700);
            outtake.pivotToScoreSamp();
        }else if (gamepad1.a){
            outtakeLift.LiftTarget(250);
            outtake.pivotToTransfer();
        }
        if (gamepad1.dpad_up){
            outtake.openClaw();
        } else {
            outtake.closeClaw();
        }

        outtake.updatePivPosition();
        intake.moveThings();
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