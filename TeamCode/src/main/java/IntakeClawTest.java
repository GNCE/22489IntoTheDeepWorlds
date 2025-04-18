import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Intake Claw Test")
@Config
public class IntakeClawTest extends OpMode {
    private Servo Rdiffy;
    private Servo Ldiffy;

    private Servo RightArmPivot;
    private Servo LeftArmPivot;


    public static double DEFAULT_LDIFFY_POS = 0.527;
    public static double DEFAULT_RDIFFY_POS = 0.503;
    public static double LdiffyPos = DEFAULT_LDIFFY_POS;
    public static double RdiffyPos = DEFAULT_RDIFFY_POS;

    public static double targetUpDown = 0;
    public static double targetSpin = 0;
    private Servo IntakeClamp;

    @Override
    public void init(){
        Rdiffy = hardwareMap.get(Servo.class,"IntakeRDiffy");
        Ldiffy = hardwareMap.get(Servo.class,"IntakeLDiffy");
        Rdiffy.setDirection(Servo.Direction.REVERSE);
        Ldiffy.setDirection(Servo.Direction.FORWARD);

        IntakeClamp = hardwareMap.get(Servo.class, "intakeClamp");
        IntakeClamp.setDirection(Servo.Direction.REVERSE);

        RightArmPivot = hardwareMap.get(Servo.class, "rightArmPivot");
        LeftArmPivot = hardwareMap.get(Servo.class, "leftArmPivot");
        RightArmPivot.setDirection(Servo.Direction.FORWARD);
        LeftArmPivot.setDirection(Servo.Direction.REVERSE);
    }

    public static double ArmPosition = 0.05;
    public static double ClawPosition = 0;


    @Override
    public void loop(){
        LdiffyPos = DEFAULT_LDIFFY_POS + targetUpDown/(360*5) + targetSpin*((double) 18/52) /(360*5);
        RdiffyPos = DEFAULT_RDIFFY_POS + targetUpDown/(360*5) - targetSpin*((double) 18/52) /(360*5);
        IntakeClamp.setPosition(ClawPosition);
        Ldiffy.setPosition(LdiffyPos);
        Rdiffy.setPosition(RdiffyPos);
        LeftArmPivot.setPosition(ArmPosition);
        RightArmPivot.setPosition(ArmPosition);
    }
}
