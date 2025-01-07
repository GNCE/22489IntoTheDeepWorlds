import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.PathBuilder;

public class RedSpecPath {

    public RedSpecPath() {
        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(7.100, 54.600, Point.CARTESIAN),
                                new Point(30.000, 73.000, Point.CARTESIAN),
                                new Point(40.000, 73.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(40.000, 73.000, Point.CARTESIAN),
                                new Point(23.000, 23.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(23.000, 23.000, Point.CARTESIAN),
                                new Point(23.000, 12.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(23.000, 12.000, Point.CARTESIAN),
                                new Point(23.000, 12.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-25))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(23.000, 12.000, Point.CARTESIAN),
                                new Point(10.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-25), Math.toRadians(0))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(10.000, 34.000, Point.CARTESIAN),
                                new Point(40.000, 70.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(40.000, 70.000, Point.CARTESIAN),
                                new Point(10.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(10.000, 34.000, Point.CARTESIAN),
                                new Point(40.000, 67.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(40.000, 67.000, Point.CARTESIAN),
                                new Point(10.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(10.000, 34.000, Point.CARTESIAN),
                                new Point(40.000, 64.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 11
                        new BezierLine(
                                new Point(40.000, 64.000, Point.CARTESIAN),
                                new Point(10.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(10.000, 34.000, Point.CARTESIAN),
                                new Point(40.000, 61.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
    }
}