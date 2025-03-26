import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Outtake Claw Test")
@Config
public class OuttakeClawTest extends OpMode {
    private Servo Rdiffy;
    private Servo Ldiffy;

    public static double DEFAULT_LDIFFY_POS = 0.527;
    public static double DEFAULT_RDIFFY_POS = 0.503;
    public static double LdiffyPos = DEFAULT_LDIFFY_POS;
    public static double RdiffyPos = DEFAULT_RDIFFY_POS;

    public static double targetUpDown = 0;
    public static double targetSpin = 0;

    @Override
    public void init(){
        Rdiffy = hardwareMap.get(Servo.class,"Rdiffy");
        Ldiffy = hardwareMap.get(Servo.class,"Ldiffy");
        Rdiffy.setDirection(Servo.Direction.REVERSE);
        Ldiffy.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void loop(){
        LdiffyPos = DEFAULT_LDIFFY_POS + targetUpDown/(300) + targetSpin*((double) 18/52) /(300);
        RdiffyPos = DEFAULT_RDIFFY_POS + targetUpDown/(300) - targetSpin*((double) 18/52) /(300);

        Ldiffy.setPosition(LdiffyPos);
        Rdiffy.setPosition(RdiffyPos);
    }
}
