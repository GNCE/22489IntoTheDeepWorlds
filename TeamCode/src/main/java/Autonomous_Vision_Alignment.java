import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import config.subsystems.IntakeLimelightSubsys;
import config.core.utils.SubsystemCore;
import config.subsystems.UnifiedTelemetry;

@Autonomous(name = "Autonomous Vision Alignment")
public class Autonomous_Vision_Alignment extends OpMode {
    private Follower follower;
    private IntakeLimelightSubsys ll;
    private UnifiedTelemetry tel;
    private Intake_DiffyClaw diffyClawIntake;
    public static double mx =  -0.008, my =  -0.021;
    public static double targetX = 16, targetY = 0;
    private double targetHeading = 180, headingError, headingCorrection;

    public static double hp = 0.4, hi = 0, hd = 0.00008;
    public static double angleThreshold = 0.05;
    PIDController headingPIDController = new PIDController(hp, hi, hd);
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    @Override
    public void init(){
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        tel = new UnifiedTelemetry();
        tel.init(this.telemetry);
        SubsystemCore.setGlobalParameters(hardwareMap, this);

        diffyClawIntake = new Intake_DiffyClaw();
        diffyClawIntake.init();
        ll = new IntakeLimelightSubsys();
        ll.init();
    }
    @Override
    public void start(){
        follower.startTeleopDrive();
        ll.turnOn();
        diffyClawIntake.ExtendTo(Intake_DiffyClaw.IntakeExtensionStates.FULL_EXTENSION);
        diffyClawIntake.setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_ARM_READY);
        diffyClawIntake.setClawState(Intake_DiffyClaw.CLAW_STATE.OPEN);
    }
    @Override
    public void loop(){
        ll.loop();
        diffyClawIntake.HoldExtension();
        diffyClawIntake.loop();
        follower.update();
        if (ll.isRunning() && ll.isResultValid() && gamepad1.right_stick_button) {
            headingError = targetHeading - follower.getPose().getHeading();
            headingError = Math.IEEEremainder(headingError + 2*Math.PI, 2*Math.PI);
            if(headingError > 2*Math.PI - headingError){
                headingError = headingError - 2*Math.PI;
            }
            if(Math.abs(headingError) < Math.toRadians(angleThreshold)){
                headingCorrection = 0;
            } else {
                headingPIDController.setPID(hp, hi, hd);
                headingCorrection = headingPIDController.calculate(headingError);
            }
            follower.setTeleOpMovementVectors((targetX - ll.getTx()) * mx, (targetY -  ll.getTy()) * my, -headingCorrection);
            double angle = ll.getAngle(); // Output 0 is sample angle
            if(Math.abs(angle) > 85){
                if(Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED >= 0) angle = 85;
                else angle = -85;
            }
            if(angle < -90) angle = -90;
            else if(angle > 90) angle = 90;

            Intake_DiffyClaw.INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED = angle * 10.5/9;
        }

    }
}
