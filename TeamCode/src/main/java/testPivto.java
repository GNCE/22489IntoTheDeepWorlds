import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "motor test")
public class testPivto extends OpMode {
    private Servo clamp;
    private Servo pivlow;
    private Servo rpivhigh;
    private Servo lpivhigh;
    private Servo revolute;
    double pivpos = 0;

    public void init() {
        clamp = hardwareMap.get(Servo.class, "clamp");
        rpivhigh = hardwareMap.get(Servo.class, "rpivhigh");
        pivlow = hardwareMap.get(Servo.class, "pivlow");
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
            pivlow.setPosition(0.25);
        } else if (gamepad1.b) {
            pivpos  = 0;
            pivlow.setPosition(0);
        } else if (gamepad1.x) {
            pivpos = 1.6;
            pivlow.setPosition(0.25);
        } else if (gamepad1.dpad_down) {
            pivpos = 0;
            pivlow.setPosition(0.3);
        } else if (gamepad1.dpad_up) {
            pivpos = 1.2;
            pivlow.setPosition(.5);
        }
        if (gamepad1.dpad_left){
            clamp.setPosition(0.05);
        } else {
            clamp.setPosition(0);
        }
    }
}
