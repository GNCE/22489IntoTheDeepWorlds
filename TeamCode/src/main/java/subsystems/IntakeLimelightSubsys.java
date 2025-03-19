package subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class IntakeLimelightSubsys extends SubsysCore {
    public static Limelight3A ll;
    public static UnifiedTelemetry tel = new UnifiedTelemetry();
    public static LLResult llResult;
    private static double tx, ty, ta, angle;
    public boolean isRunning(){ return ll.isRunning(); }
    public boolean isDataFresh(){ return llResult.getStaleness() < 30; }

    public void turnOn(){ if(!isRunning()) ll.start(); }
    public void turnOff(){ if(isRunning()) ll.stop(); }
    public boolean isResultValid(){
        if(!isRunning()) return false;
        return isDataFresh() && (llResult.isValid() || getTa() > 6);
    }

    public static double getTa() { return ta; }
    public static double getTx(){ return tx; }
    public static double getTy(){ return ty; }
    public static double getAngle(){ return angle; }

    @Override
    void init(){
        ll = hardwareMap.get(Limelight3A.class, "limelight");
        ll.setPollRateHz(100);
    }

    @Override
    void loop(){
        if(isRunning()){
            llResult = ll.getLatestResult();
            tx = llResult.getTx();
            ty = llResult.getTy();
            ta = llResult.getTa();
            angle = llResult.getPythonOutput()[0];
        }

        tel.addLine("Limelight Data");
        tel.addData("Running", isRunning());
        if(isRunning()){
            tel.addData("Data Valid", isResultValid());
            tel.addData("Data Fresh", isDataFresh());
            tel.addData("Detected X", getTx());
            tel.addData("Detected Y", getTy());
            tel.addData("Detected Area", getTa());
            tel.addData("Detected Angle", getAngle());
        }
    }
}