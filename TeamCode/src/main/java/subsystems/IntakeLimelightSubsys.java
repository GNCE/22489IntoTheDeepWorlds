package subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class IntakeLimelightSubsys extends SubsysCore {
    public static Limelight3A ll;
    public static UnifiedTelemetry tel = new UnifiedTelemetry();
    public static LLResult llResult;
    private static double tx, ty, ta, angle;
    private static double[] pythonOutput;
    private static int pipelineNumber;
    public boolean isRunning(){ return ll.isRunning(); }
    public boolean isDataFresh(){ return llResult.getStaleness() < 30; }

    public void turnOn(){ if(!isRunning()) ll.start(); }
    public void turnOff(){ if(isRunning()) ll.stop(); }
    public boolean isResultValid(){
        if(!isRunning()) return false;
        return isDataFresh() && (llResult.isValid() || getTa() > 6);
    }

    public double getTa() { return ta; }
    public double getTx(){ return tx; }
    public double getTy(){ return ty; }
    public double getAngle(){ return angle; }
    public void setPipelineNumber(int pipelineNum){
        if(pipelineNum > 6 || pipelineNum < 4) return;
        pipelineNumber = pipelineNum;
    }
    public void captureSnapshot(String key){
        ll.captureSnapshot(key);
    }

    public int getPipelineNumber(){ return pipelineNumber; }

    @Override
    void init(){
        ll = hardwareMap.get(Limelight3A.class, "limelight");
        ll.setPollRateHz(100);
    }

    @Override
    void loop(){
        if(isRunning()){
            ll.pipelineSwitch(pipelineNumber);
            llResult = ll.getLatestResult();
            tx = llResult.getTx();
            ty = llResult.getTy();
            ta = llResult.getTa();

            pythonOutput = llResult.getPythonOutput();
            if(pythonOutput != null){
                if(pythonOutput.length > 0) angle = pythonOutput[0];
            }
        }

        tel.addLine("Limelight Data");
        tel.addData("Running", isRunning());
        if(isRunning()){
            tel.addData("Pipeline:", getPipelineNumber());
            tel.addData("Data Valid", isResultValid());
            tel.addData("Data Fresh", isDataFresh());
            tel.addData("Detected X", getTx());
            tel.addData("Detected Y", getTy());
            tel.addData("Detected Area", getTa());
            tel.addData("Detected Angle", getAngle());
        }
    }
}