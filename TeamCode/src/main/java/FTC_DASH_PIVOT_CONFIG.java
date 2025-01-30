import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@TeleOp (name = "For FTC DASH ONLY")
public class FTC_DASH_PIVOT_CONFIG extends OpMode {
    private Servo clamp;
    private Servo rpivhigh;
    private Servo lpivhigh;
    static OuttakeLift outtakeLift;
    static double pivpos = 0;
    static int liftpos = 0;
    static double clamppos = 0;

    public void init() {
        clamp = hardwareMap.get(Servo.class, "clamp");
        rpivhigh = hardwareMap.get(Servo.class, "rpivhigh");
        lpivhigh = hardwareMap.get(Servo.class, "lpivhigh");
        rpivhigh.setDirection(Servo.Direction.FORWARD);
        lpivhigh.setDirection(Servo.Direction.REVERSE);
        clamp.setDirection(Servo.Direction.REVERSE);
    }

    public void loop() {
        if (gamepad1.y) {
            pivpos = 0.6;
        } else if (gamepad1.b) {
            pivpos  = 0;
        } else if (gamepad1.x) {
            pivpos = 1.6;
        } else if (gamepad1.dpad_down) {
            pivpos = 0;
        } else if (gamepad1.dpad_up) {
            pivpos = 1.2;
        }
        if (gamepad1.dpad_left){
            clamppos = 0.05;
        } else {
            clamppos = 0;
        }
        rpivhigh.setPosition(pivpos);
        lpivhigh.setPosition(pivpos);
        clamp.setPosition(clamppos);
        outtakeLift.LiftTarget(liftpos);
    }
}
