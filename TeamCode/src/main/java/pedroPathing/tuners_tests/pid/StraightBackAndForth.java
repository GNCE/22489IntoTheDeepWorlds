package pedroPathing.tuners_tests.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "Straight Back And Forth", group = "PIDF Tuning")
public class StraightBackAndForth extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean forward = true;

    private Follower follower;

    private PathChain forwards;
    private PathChain backwards;
    private ElapsedTime elapsedTime;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        elapsedTime = new ElapsedTime();
        elapsedTime.startTime();
        forwards = follower.pathBuilder()
                .addBezierLine(new Point(0,0, Point.CARTESIAN), new Point(DISTANCE,0, Point.CARTESIAN))
                .setConstantHeadingInterpolation(0)
                .build();
        backwards = follower.pathBuilder()
                .addBezierLine(new Point(DISTANCE,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN))
                .setConstantHeadingInterpolation(0)
                .build();

        follower.followPath(forwards, true);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                            + " inches forward. The robot will go forward and backward continuously"
                            + " along the path. Make sure you have enough room.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    boolean followerPrevBusy;
    @Override
    public void loop() {
        follower.update();
        if (!follower.isBusy()) {
            if(followerPrevBusy != follower.isBusy()){
                elapsedTime.reset();
            }
            if(elapsedTime.time() > 1) {
                if (forward) {
                    forward = false;
                    follower.followPath(backwards, true);
                } else {
                    forward = true;
                    follower.followPath(forwards, true);
                }
            }
        }

        followerPrevBusy = follower.isBusy();

        telemetryA.addData("going forward", forward);
        follower.telemetryDebug(telemetryA);
    }
}
