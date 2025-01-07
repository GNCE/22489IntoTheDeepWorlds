

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@TeleOp(name = "Example Robot-Centric Teleop", group = "Examples")
public class Teleop_26111 extends OpMode {
    private Follower follower;
    private Outtake outtake;
    private Intake intake;
    private final Pose startPose = new Pose(0,0,0);
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap,this);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {


        follower.setTeleOpMovementVectors(0.48 * Math.tan(1.12 * -gamepad1.left_stick_y), 0.48 * Math.tan(1.12 * -gamepad1.left_stick_x), -gamepad1.right_stick_x, true);
        follower.update();
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}