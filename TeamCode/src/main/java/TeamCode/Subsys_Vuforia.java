package TeamCode;
import android.hardware.Camera;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;



public class Subsys_Vuforia {
    HardwareMap hardwareMap;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;
    private VuforiaLocalizer vuforia = null;
    public static Camera cam = null;
    private static final String VUFORIA_KEY =
            "Ado05xz/////AAABmY8uetMq2krthNRU8hk1XbAPBHbJ/EVJizmHI/8Kz+HV4an+0zONWUZOd9XOiJIebM2WA7z/Wzffa9W87IrMnmb4pKEkY5dYbzjEdsDy28aKcZSkAu7jpO610LnMv+tWDKK3Chj+apf7OinQiaMnm9xSdjIOTxe6kegt5kHTY6inImWrZuHXe6trOfv48elrDyhrTDNELqZwjjG1LFZkGzgyKCQ9wvWcO0JXec+R5iQg+RMc92eqhCMv/6558QRae364puvHtp0OfszOivgelgFk901BvjQzTFzYnh80+tFWbiNNGfc6jzyz09xcWBR9B9xOsIPHcNPgsF9akWHjrEaDCtj/XdsrlqOf93xq31fl ";
    Subsys_Vuforia(){}

    public void runOpMode(HardwareMap hM){
        this.hardwareMap = hM;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersV = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parametersV.vuforiaLicenseKey = VUFORIA_KEY;
        parametersV.cameraDirection   = CAMERA_CHOICE;


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parametersV);
        CameraDevice.getInstance().setFlashTorchMode(true);//turn on flashlight
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.



    }

    public void detect(){
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        targetsSkyStone.activate();
        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        while (!((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {

        }
    }
}
