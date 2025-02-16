import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "extendo testy turney funny")

@Config
public class extendoTurney extends OpMode {
    private Servo reintake;
    private Servo leintake;


    public static boolean useBoth = false;
    public static double extendoPos = 0;

    @Override
    public void init(){
        reintake = hardwareMap.get(Servo.class,"reintake");
        leintake = hardwareMap.get(Servo.class, "leintake");
        reintake.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop(){
        if(useBoth){
            reintake.setPosition(extendoPos);
        }
        leintake.setPosition(extendoPos);
    }
}
