import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
public class Outtake_Position_Config extends OpMode {
    private Servo clamp;
    private Servo Rdiffy;
    private Servo Ldiffy;
    private Servo rpivhigh;
    private Servo lpivhigh;
    double ArmPosition = 0;
    double updownpos = 0;
    double orientationpos = 0;
    double clawPos = 0;

    @Override
     public void init(){
         clamp = hardwareMap.get(Servo.class, "clamp");
         rpivhigh = hardwareMap.get(Servo.class, "rpivhigh");
         lpivhigh = hardwareMap.get(Servo.class, "lpivhigh");
         rpivhigh.setDirection(Servo.Direction.FORWARD);
         lpivhigh.setDirection(Servo.Direction.REVERSE);
         Rdiffy = hardwareMap.get(Servo.class,"Rdiffy");
         Ldiffy = hardwareMap.get(Servo.class,"Ldiffy");
         Rdiffy.setDirection(Servo.Direction.FORWARD);
         Ldiffy.setDirection(Servo.Direction.REVERSE);
     }
     @Override
    public void loop(){
        if (gamepad1.a){
            ArmPosition += 0.01;
        } else if (gamepad1.b){
            ArmPosition -= 0.01;
        }
        if (gamepad1.x){
            updownpos += 0.01;
        }else if(gamepad1.y){
            updownpos -= 0.01;
        }

     }
}
