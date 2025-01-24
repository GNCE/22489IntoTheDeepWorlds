import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp (name = "For FTC DASH ONLY")
public class FTC_DASH_PIVOT_CONFIG extends OpMode {
    private Servo clamp;
    private Servo pivlow;
    private Servo rpivhigh;
    private Servo lpivhigh;
    private Servo revolute;
    static double pivpos = 0;
    static double lowpivpos = 0;
    static double revolutepos = 0;

    public void init() {
        clamp = hardwareMap.get(Servo.class, "clamp");
        rpivhigh = hardwareMap.get(Servo.class, "rpivhigh");
        pivlow = hardwareMap.get(Servo.class, "updownpiv");
        lpivhigh = hardwareMap.get(Servo.class, "lpivhigh");
        revolute = hardwareMap.get(Servo.class, "revolute");
        rpivhigh.setDirection(Servo.Direction.FORWARD);
        lpivhigh.setDirection(Servo.Direction.REVERSE);
        pivlow.setDirection(Servo.Direction.FORWARD);
        clamp.setDirection(Servo.Direction.REVERSE);
    }

    public void loop() {
        if (gamepad1.y) {
            pivpos = 0.6;
            lowpivpos = (0.25);
            revolutepos = (.5);
        } else if (gamepad1.b) {
            pivpos  = 0;
            lowpivpos = (0);
            revolutepos=(1.5);
        } else if (gamepad1.x) {
            pivpos = 1.6;
            lowpivpos = (0.25);
        } else if (gamepad1.dpad_down) {
            pivpos = 0;
            lowpivpos = (0.3);
        } else if (gamepad1.dpad_up) {
            pivpos = 1.2;
            lowpivpos = (.5);
        }
        if (gamepad1.dpad_left){
            clamp.setPosition(0.05);
        } else {
            clamp.setPosition(0);
        }
        rpivhigh.setPosition(pivpos);
        lpivhigh.setPosition(pivpos);
        pivlow.setPosition(lowpivpos);
        revolute.setPosition(revolutepos);
    }
}
