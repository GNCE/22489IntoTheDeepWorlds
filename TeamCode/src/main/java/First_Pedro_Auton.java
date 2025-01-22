import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous (name = "first Pedro auton")
public class First_Pedro_Auton extends OpMode{
    private Follower follower;

    private Intake intake;
    private OuttakeLift outtakeLift;
    private Outtake outtake;
    private Misc misc;
    private int transferRealFSM =0;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(7, 62, Math.toRadians(0));
    private final Pose scorePose = new Pose(14, 129, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(20, 22, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(20, 11, Math.toRadians(0));
    private final Pose pickupSpecimen = new Pose(7.5, 34, Math.toRadians(0));

    private Path scorePreload, scoreCycle, pickupSpeciman2;
    private PathChain grabPickup1, grabPickup2, pickupSpeciman1;
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());
        scoreCycle = new Path(new BezierLine(new Point(pickupSpecimen), new Point(scorePose)));
        scoreCycle.setConstantHeadingInterpolation(0);
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();
        pickupSpeciman1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(pickupSpecimen)))
                .setConstantHeadingInterpolation(0)
                .build();
        pickupSpeciman2 = new Path(new BezierLine(new Point(scorePose),new Point(pickupSpecimen)));
        pickupSpeciman2.setConstantHeadingInterpolation(0);

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                outtakeLift.LiftTarget(250);
                outtake.pivotToFront();
                setPathState(1);
                break;
            case 1:
                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                if (follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    //add score code if needed
                    follower.followPath(grabPickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (follower.getPose().getX() > (pickup1Pose.getX() - 1) && follower.getPose().getY() > (pickup1Pose.getY() - 1)) {
                    intake.flipDown();
                    intake.ManualExtend();
                    if (intake.isRed()){
                        intake.ManualRetract();
                        intake.flipUp();
                        follower.followPath(grabPickup2, true);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if (follower.getPose().getX() > (pickup2Pose.getX() - 1) && follower.getPose().getY() > (pickup2Pose.getY() - 1)) {
                    intake.deposit();
                    intake.flipDown();
                    intake.ManualExtend();
                    if (intake.isRed()){
                        intake.ManualRetract();
                        intake.flipUp();
                        outtake.openClaw();
                        follower.followPath(pickupSpeciman1, true);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (follower.getPose().getX() > (pickupSpecimen.getX() - 1) && follower.getPose().getY() > (pickupSpecimen.getY() - 1)) {
                    intake.deposit();
                    outtakeLift.LiftTarget(100);
                    outtake.pivotToPickupBack();
                    outtake.closeClaw();

                    follower.followPath(scoreCycle, true);
                    setPathState(5);
                }
                break;
            case 5:
                outtakeLift.LiftTarget(250);
                outtake.pivotToFront();
                if (follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    //add score code if needed
                    follower.followPath(pickupSpeciman2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (follower.getPose().getX() > (pickupSpecimen.getX() - 1) && follower.getPose().getY() > (pickupSpecimen.getY() - 1)) {
                    outtakeLift.LiftTarget(100);
                    outtake.pivotToPickupBack();
                    outtake.closeClaw();

                    follower.followPath(scoreCycle, true);
                    setPathState(7);
                }
                break;
            case 7:
                outtakeLift.LiftTarget(250);
                outtake.pivotToFront();
                if (follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    //add score code if needed
                    follower.followPath(pickupSpeciman2, true);
                    setPathState(-1); //ends code
                }
                break;

        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        intake = new Intake(hardwareMap, this);
        outtake = new Outtake(hardwareMap);
        outtakeLift = new OuttakeLift(hardwareMap,this);
        buildPaths();
    }
    @Override
    public void start(){
        outtakeLift.HoldLift();
    }
    public void pickupTransfer(){
        switch (transferRealFSM){
            case 1:
                outtake.openClaw();
                outtakeLift.LiftTarget(250);
                outtakeLift.GetLiftPos();
                if (outtakeLift.GotLiftPos <=250){
                    transferRealFSM = 2;
                }
                break;
            case 2:
                outtake.closeClaw();
                outtake.pivotToScoreSamp();
                outtakeLift.LiftTarget(750);
                transferRealFSM = 0;
                break;
        }
    }
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        intake.moveThings();
        pickupTransfer();
        outtake.updatePivPosition();
        outtakeLift.Auton();
        outtakeLift.HoldLift();

        PoseStorage.CurrentPose = follower.getPose();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

}
