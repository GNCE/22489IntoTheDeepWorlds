import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Outtake Arm Test")
@Config
public class OuttakeArmTest extends OpMode {
    private Servo rpivhigh;
    private Servo lpivhigh;
    public static double ARM_CUR_POS = 0.0;


    @Override
    public void init(){
        rpivhigh = hardwareMap.get(Servo.class, "outtakeRightArm");
        lpivhigh = hardwareMap.get(Servo.class, "outtakeLeftArm");
        rpivhigh.setDirection(Servo.Direction.REVERSE);
        lpivhigh.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void loop(){

        lpivhigh.setPosition(ARM_CUR_POS);
        rpivhigh.setPosition(ARM_CUR_POS);
    }
}
