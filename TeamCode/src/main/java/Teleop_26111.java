

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
        misc.initiate();
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
        if (gamepad2.y){
            outtakeLift.LiftTarget(500);
            outtake.pivotToScoreorpickupSpecFront();
        }else if (gamepad2.b){
            outtakeLift.LiftTarget(500);
            outtake.pivotToPickupBack();
        }else if (gamepad2.x){
            outtake.pivotToScoreSampandBackSpec();
            outtakeLift.LiftTarget(750);
        }else if (gamepad2.a){
            outtake.pivotToTransfer();
            outtakeLift.LiftTarget(500);
        }
        if (gamepad2.left_bumper){
            outtake.openClaw();
        } else {
            outtake.closeClaw();
        }
        if (gamepad1.right_bumper){
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
                0.28 * Math.tan(1.12 * -gamepad1.right_stick_x), true);
        follower.update();
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();

    }

}