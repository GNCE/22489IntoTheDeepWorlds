import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.PathBuilder;

public class GeneratedPath {
    public GeneratedPath() {
        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(7.100, 113.400, Point.CARTESIAN),
                                new Point(7.500, 123.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(7.500, 123.000, Point.CARTESIAN),
                                new Point(30.000, 121.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(30.000, 121.000, Point.CARTESIAN),
                                new Point(17.000, 125.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(17.000, 125.000, Point.CARTESIAN),
                                new Point(25.000, 131.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(25.000, 131.000, Point.CARTESIAN),
                                new Point(17.000, 125.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(17.000, 125.000, Point.CARTESIAN),
                                new Point(20.000, 128.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(28))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(20.000, 128.000, Point.CARTESIAN),
                                new Point(17.000, 125.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(28), Math.toRadians(-45))
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(17.000, 125.000, Point.CARTESIAN),
                                new Point(60.000, 96.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(60.000, 96.000, Point.CARTESIAN),
                                new Point(17.000, 125.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(17.000, 125.000, Point.CARTESIAN),
                                new Point(70.000, 96.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
                .addPath(
                        // Line 11
                        new BezierLine(
                                new Point(70.000, 96.000, Point.CARTESIAN),
                                new Point(17.000, 125.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45));
    }
}

