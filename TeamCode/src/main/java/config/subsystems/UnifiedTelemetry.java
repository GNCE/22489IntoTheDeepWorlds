package config.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class UnifiedTelemetry {
    private static Telemetry internal;
    private static TelemetryPacket currentDashPack;
    private boolean dashEnabled = true;

    private boolean sendTel = true;

    public void turnOff() {
        sendTel = false;
    }
    public void turnOn() {
        sendTel = true;
    }

    public UnifiedTelemetry addLine(){
        if (internal != null) internal.addLine();
        if (dashEnabled) currentDashPack.addLine("");
        return this;

    }

    public UnifiedTelemetry addLine(String line){
        if (internal != null) internal.addLine(line);
        if (dashEnabled) currentDashPack.addLine(line);
        return this;

    }

    public UnifiedTelemetry addData(String caption, String format, Object... args) {
        if (internal != null) internal.addData(caption, format, args);
        if (dashEnabled)
            currentDashPack.put(caption, String.format(format, args));

        return this;
    }

    public UnifiedTelemetry addData(String caption, Object value) {
        if (internal != null) internal.addData(caption, value);
        if (dashEnabled)
            currentDashPack.put(caption, value);

        return this;
    }

    public boolean isTelemetryEnabled(){
        return sendTel;
    }

//    public void drawHeading(double heading) {
//        currentDashPack.fieldOverlay()
//            .setStroke("red")
//            .setRotation(-heading)
//            .strokeLine(0, 0, 10, 0)
//            .setRotation(0);
//    }
//
//    private void drawRobo(double x, double y, double rad, String color) {
//        currentDashPack.fieldOverlay()
//            .setTranslation(x, y)
//            .setRotation(-rad)
//            .setStroke(color)
//            .strokeLine(0, 0, 0, 12)
//            .strokePolygon(
//                new double[]{
//                    9,
//                    9,
//                    -9,
//                    -9
//                },
//                new double[]{
//                    9,
//                    -9,
//                    -9,
//                    9
//                }
//            )
//            .setRotation(0)
//            .setTranslation(0, 0);
//    }

//    public void drawAprilTagSpots(Map<Integer, Pose2d> mapping) {
//        for (Pose2d pose: mapping.values()) {
//            currentDashPack.fieldOverlay()
//                .setTranslation(pose.x, pose.y)
//                .setFill("limegreen")
//                .fillCircle(0, 0, 2);
//        }
//
//        currentDashPack.fieldOverlay()
//            .setTranslation(0, 0);
//    }

    public void update() {
        if (internal != null) internal.update();
        if (dashEnabled) {
            try {
                FtcDashboard.getInstance().sendTelemetryPacket(currentDashPack);
            } catch(NullPointerException e) {
                // do nothing
            }
            currentDashPack = new TelemetryPacket();
            currentDashPack.clearLines();
        }
    }

    public void disable_dash() {
        dashEnabled = false;
        currentDashPack = null;
    }

    // maybe todo: value producer addDatas
    // I don't remember what this is

    public void init(Telemetry telemetry){
        internal = telemetry;
        dashEnabled = true;
        currentDashPack = new TelemetryPacket();
        currentDashPack.clearLines();
    }
}