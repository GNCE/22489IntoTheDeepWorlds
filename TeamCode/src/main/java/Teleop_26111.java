

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
    @Override
    public void loop() {
        if (gamepad1.left_bumper){
            intake.flipDown();
        }
        intake.TeleopExtend(); //left trigger
        if (gamepad1.right_trigger > 0.2){
            intake.flipUp();
            intake.deposit();
        }
        if (gamepad2.y){
            outtake.pivotToFront();
        }else if (gamepad2.b){
            outtake.pivotToPickupBack();
        }else if (gamepad2.x){
            outtake.pivotToScoreSamp();
            outtakeLift.LiftTarget(750);
            misc.undoor();
        }else if (gamepad2.a){
            outtake.pivotToTransfer();
            misc.door();
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
        outtakeLift.HoldLift();
        outtake.updatePivPosition();
        intake.check();
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
        telemetry.addLine();
        telemetry.addData("Elapsed Time", elapsedTime.toString());
        telemetry.addLine();
        telemetry.addData("lift position",outtakeLift.rlift.getCurrentPosition());
        telemetry.addData("extendo position", intake.extendo.getCurrentPosition());
        telemetry.update();

    }

}