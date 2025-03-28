import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Misc;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import subsystems.OuttakeLiftSubsys;

@Disabled
@Autonomous (name = "0+4 auton pls worky")
public class Auto_0_4 extends OpMode{
    private Follower follower;

    private Old_Intake_DoNotUse intake;
    private OuttakeLiftSubsys outtakeLift;
    private Outtake outtake;
    private Misc misc;
    private int transferRealFSM = -1;
    private Timer pathTimer, opmodeTimer;
    private ElapsedTime sequenceTime, resetEncoderDelay;
    private int pathState;
    /** Start Pose of our robot */
    private final Pose startPose = new Pose(7.35, 113.625, Math.toRadians(270));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(13, 130, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(24, 121, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(24, 131, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(24, 132, Math.toRadians(15));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(60, 100, Math.toRadians(270));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(60, 120, Math.toRadians(270));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
    private PathChain[] grabPickups, scorePickups;
    ElapsedTime elapsedTime;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());

        grabPickups = new PathChain[]{grabPickup1, grabPickup2, grabPickup3};
        scorePickups = new PathChain[]{scorePickup1, scorePickup2, scorePickup3};
    }

    private int sampleCounter = 0;

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(scorePreload);
                outtakeSequence = OUTTAKE_SEQUENCE.BUCKET_SEQUENCE;
                bucketSequence = BUCKET_SEQUENCE.GRAB_AND_LIFT;
                sequenceTime.reset();
                setPathState(1);
                break;
            case 1:
                // Preload
                if(!follower.isBusy() && !outtakeLift.isBusy()) {
                        sequenceTime.reset();
                        setPathState(2);
                }
                break;
            case 2:
                // Score
                bucketSequence = BUCKET_SEQUENCE.SCORE;
                if ( pathTimer.getElapsedTimeSeconds() > 2){
                    sequenceTime.reset();
                    setPathState(5);
                }
                break;
            case 5:
                // Loop Begins
                // Outtake ready to transfer & goes to grab a sample
                bucketSequence = BUCKET_SEQUENCE.TRANSFER;
                follower.followPath(grabPickups[sampleCounter],true);

                setPathState(6);
                break;
            case 6:
                if(!follower.isBusy()){
                    intake.setIntakeState(Old_Intake_DoNotUse.IntakeState.INTAKE);
                    intake.setExtensionTarget(pathTimer.getElapsedTimeSeconds()*200);
                    if (pathTimer.getElapsedTimeSeconds() > 3 || intake.getCurrentSampleState(false) == Old_Intake_DoNotUse.SENSOR_READING.CORRECT){
                    setPathState(7);}
                }
                break;
            case 7:
                    intake.setIntakeState(Old_Intake_DoNotUse.IntakeState.TRANSFER);
                    intake.setExtensionTarget(Old_Intake_DoNotUse.TRANSFER_EXTENSION_POS);
                if (pathTimer.getElapsedTimeSeconds() >  2) {
                    sequenceTime.reset();
                    setPathState(8);
                }
                break;
            case 8:
                    bucketSequence = BUCKET_SEQUENCE.GRAB_AND_LIFT;
                    follower.followPath(scorePickups[sampleCounter], true);
                    if (pathTimer.getElapsedTimeSeconds() > 3) {
                        sequenceTime.reset();
                        setPathState(9);
                    }
                break;
            case 9:
                // Score
                bucketSequence = BUCKET_SEQUENCE.SCORE;
                if (pathTimer.getElapsedTimeSeconds() > 3){
                    sequenceTime.reset();
                    setPathState(10);
                }
            case 10:
                // check
                sampleCounter++;
                if (sampleCounter < 3) {
                    setPathState(5);
                } else {
                    outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCORE);
                    follower.followPath(park, false);
                    setPathState(-1);
                }
                break;
            default:
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
        intake = new Old_Intake_DoNotUse(hardwareMap, this);
        misc = new Misc(hardwareMap);
        outtake = new Outtake(hardwareMap);
        outtakeLift = new OuttakeLift(hardwareMap,this);
        elapsedTime = new ElapsedTime();
        resetEncoderDelay = new ElapsedTime();
        sequenceTime = new ElapsedTime();
        buildPaths();
    }
    @Override
    public void start(){
        intake.initiate();
        outtakeLift.holdLift();
       // misc.initiate();
    }

    private final ToggleButton teamColorButton = new ToggleButton(Storage.isRed);

    @Override
    public void init_loop(){
        teamColorButton.input(gamepad1.dpad_up);

        Storage.isRed = teamColorButton.getVal();

        telemetry.addData("Team Color:", Storage.isRed ? "Red" : "Blue");
        telemetry.update();
    }

    private static double prevTime;
    public enum BUCKET_SEQUENCE{
        TRANSFER, GRAB_AND_LIFT, SCORE;
        private static final BUCKET_SEQUENCE[] vals = values();
        public BUCKET_SEQUENCE next(){
            return vals[(this.ordinal() + 1) % vals.length];
        }
        public BUCKET_SEQUENCE prev(){
            return vals[(this.ordinal() - 1 + vals.length) % vals.length];
        }
    }
    enum OUTTAKE_SEQUENCE {
        BUCKET_SEQUENCE,
        SPECIMEN_SEQUENCE,
    }
    BUCKET_SEQUENCE bucketSequence = BUCKET_SEQUENCE.TRANSFER;
    OUTTAKE_SEQUENCE outtakeSequence = OUTTAKE_SEQUENCE.BUCKET_SEQUENCE;

    @Override
    public void loop() {
        switch(outtakeSequence) {
            case BUCKET_SEQUENCE:
                switch (bucketSequence) {
                    case TRANSFER:
                        outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.TRANSFER);
                        outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER);
                        outtake.setClawOpen(true);
                        break;
                    case GRAB_AND_LIFT:
                        intake.startReverseIntake();
                        outtake.setClawOpen(false);
                        if (sequenceTime.time() > 0.4) {
                            outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.LIFT_BUCKET);
                            outtake.setOuttakeState(Outtake.OuttakeState.SAMPLESCORE);
                            resetEncoderDelay.reset();
                        }
                        break;
                    case SCORE:
                        outtake.setClawOpen(true);
                        if (resetEncoderDelay.time() > 0.4) {
                            outtake.setOuttakeState(Outtake.OuttakeState.RESET_ENCODER);
                        }
                        if ((resetEncoderDelay.time() > 0.6) && outtakeLift.target != 30) {
                            outtakeLift.LiftTo(OuttakeLiftSubsys.OuttakeLiftPositions.RESET_ENCODER);
                        }
                        break;


                }
                break;
        }
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        intake.intakeLoop();
        outtake.outtakeLoop();
        outtakeLift.holdLift();
        misc.loop();

        Storage.CurrentPose = follower.getPose();


        telemetry.addLine()
                .addData("Intake State", intake.intakeState.name());

        if(intake.colorSensor instanceof DistanceSensor){
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) intake.colorSensor).getDistance(DistanceUnit.CM));
        }

        // Feedback to Driver Hub
        telemetry.addLine()
                        .addData("Loop Time", opmodeTimer.getElapsedTimeSeconds() - prevTime);
        prevTime = opmodeTimer.getElapsedTimeSeconds();
        telemetry.addData("path state", pathState);
        telemetry.addData("Scored ground samples", sampleCounter);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}