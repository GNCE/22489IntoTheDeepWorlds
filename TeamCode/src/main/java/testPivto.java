import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "pivto test")
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
        pivlow = hardwareMap.get(Servo.class, "updownpiv");
        lpivhigh = hardwareMap.get(Servo.class, "lpivhigh");
        revolute = hardwareMap.get(Servo.class, "spinpiv");
        rpivhigh.setDirection(Servo.Direction.FORWARD);
        lpivhigh.setDirection(Servo.Direction.REVERSE);
        pivlow.setDirection(Servo.Direction.FORWARD);
        clamp.setDirection(Servo.Direction.REVERSE);
    }

    public void loop() {
        if (gamepad1.y) {
            pivpos+=.01;


        } else if (gamepad1.b){
            pivpos-=0.01;
        }
        if (pivpos>1){
            pivpos=1;
        }else if (pivpos<0){
            pivpos=0;
        }
        rpivhigh.setPosition(pivpos);
        lpivhigh.setPosition(pivpos);
        telemetry.addData("pivpos",pivpos);
        telemetry.update();
    }
}
