package subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import commands.CommandCore;
import commands.InstantAction;

@Config
public class IntakeDifferentialSubsys extends SubsysCore {
    private static Servo IntakeLDiffy, IntakeRDiffy;
    private static UnifiedTelemetry tel = new UnifiedTelemetry();

    public static double DEFAULT_LDIFFY_POS = 0.399;
    public static double DEFAULT_RDIFFY_POS = 0.369;
    private static double LdiffyPos = DEFAULT_LDIFFY_POS;
    private static double RdiffyPos = DEFAULT_RDIFFY_POS;
    private static double verticalPivot = 0;
    private static double orientationPivot = 0;

    @Override
    public void init() {
        IntakeRDiffy = hardwareMap.get(Servo.class,"IntakeRDiffy");
        IntakeLDiffy = hardwareMap.get(Servo.class,"IntakeLDiffy");
        IntakeRDiffy.setDirection(Servo.Direction.FORWARD);
        IntakeLDiffy.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop(){
        double ServoRange = 360*5;
        LdiffyPos = DEFAULT_LDIFFY_POS + verticalPivot /ServoRange + orientationPivot *((double) 18/52)/ServoRange;
        RdiffyPos = DEFAULT_RDIFFY_POS + verticalPivot /ServoRange - orientationPivot *((double) 18/52)/ServoRange;

        tel.addLine("Intake Differential Control");
        tel.addData("Left Servo Position", LdiffyPos);
        tel.addData("Right Servo Position", RdiffyPos);
    }
}
