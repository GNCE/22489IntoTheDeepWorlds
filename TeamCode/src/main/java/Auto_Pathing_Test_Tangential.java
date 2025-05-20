import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import config.subsystems.Outtake;
import config.subsystems.Lift;
import config.core.utils.SubsystemCore;
import config.subsystems.UnifiedTelemetry;


@Disabled
@Autonomous (name = "Auto_Pathing_Test_Tangential")
public class Auto_Pathing_Test_Tangential extends OpMode {
    private Follower follower;
    private Intake_DiffyClaw intakeDiffyClaw;
    private Lift outtakeLift;
    private Outtake outtake;
    private Timer pathTimer;
    private final double scoreX = 39.3;
    private final double scoreY = 80;

    private final Pose startPose = new Pose(6.495, 65.45, Math.toRadians(180));
    private final Pose pickupPose = new Pose(10.5, 35, Math.toRadians(180));
    private final Pose scorePose = new Pose(39, 75, Math.toRadians(180));
    private final Pose scoreControl1 = new Pose(15, 35);
    private final Pose scoreControl2 = new Pose(25, 35);
    private final Pose scoreControl3 = new Pose(25, 75);
    private final Pose scoreControl4 = new Pose(33, 75);

    PathChain pickUpPath, scorePath;

    public void buildPaths(){
        pickUpPath = follower.pathBuilder()
                .addBezierLine(new Point(startPose), new Point(pickupPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickupPose.getHeading())
                .build();
        scorePath = follower.pathBuilder()
                .addBezierCurve(new Point(pickupPose), new Point(scoreControl1), new Point(scoreControl2), new Point(scoreControl3), new Point(scoreControl4), new Point(scorePose))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();
    }

    enum AutoState {
        DRIVE,
        SCORE,
        END,
    }
    private AutoState autoState = AutoState.DRIVE;

    public void setPathState(AutoState newState){
        autoState = newState;
        pathTimer.resetTimer();
    }

    private int counter = 0;
    public void autonomousPathUpdate(){
        switch (autoState){
            case DRIVE:
                follower.followPath(pickUpPath, true);
                setPathState(AutoState.SCORE);
                break;
            case SCORE:
                if(!follower.isBusy()){
                    follower.followPath(scorePath, true);
                    setPathState(AutoState.END);
                }
                break;
            case END:
                break;
            default:
                break;
        }
    }
    @Override
    public void init(){
        pathTimer = new Timer();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        tel = new UnifiedTelemetry();
        tel.init(this.telemetry);
        SubsystemCore.setGlobalParameters(hardwareMap, this);

        intakeDiffyClaw = new Intake_DiffyClaw();
        intakeDiffyClaw.init();
        outtake = new Outtake(hardwareMap);
        outtakeLift = new Lift();
        outtakeLift.init();
        buildPaths();
    }

    private final ToggleButton teamColorButton = new ToggleButton(Storage.isRed);
    private UnifiedTelemetry tel;
    @Override
    public void init_loop(){
        teamColorButton.input(gamepad1.dpad_up);
        Storage.isRed = teamColorButton.getVal();
        outtake.setOuttakeState(Outtake.OuttakeState.Auto_Wait);
        outtake.outtakeLoop();
        outtake.setClawOpen(false);
        tel.addData("Team Color:", Storage.isRed ? "Red" : "Blue");
        tel.update();
    }
    @Override
    public void loop(){
        follower.update();
        autonomousPathUpdate();
        outtake.outtakeLoop();
        outtakeLift.holdLift();
        outtakeLift.loop();
        intakeDiffyClaw.loop();


        Storage.CurrentPose = follower.getPose();
        telemetry.update();
    }
}