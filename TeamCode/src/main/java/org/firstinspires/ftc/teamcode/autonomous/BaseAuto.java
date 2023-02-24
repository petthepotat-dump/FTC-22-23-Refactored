package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.wrappers.Controller;
import org.firstinspires.ftc.teamcode.wrappers.DetectConeDisplay;
import org.firstinspires.ftc.teamcode.wrappers.DetectPoleDisplay;
import org.firstinspires.ftc.teamcode.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.wrappers.Position;
import org.firstinspires.ftc.teamcode.wrappers.Vision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class BaseAuto extends LinearOpMode {
    public MecanumChassis robot;
    public Position pos;
    public Controller control;
    public Vision sleeveDetection;
    public DetectPoleDisplay poleDetection;
    public DetectConeDisplay detection;
    public WebcamName webcamName;
    public OpenCvCamera camera;
    public String route;
    public FtcDashboard dashboard = FtcDashboard.getInstance();

    public void init(HardwareMap map){
        robot = new MecanumChassis(map);
        pos = new Position(robot);
        control = new Controller(robot, pos);
        int cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamName = hardwareMap.get(WebcamName.class, "Camera");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        sleeveDetection = new Vision();
        poleDetection = new DetectPoleDisplay();
        detection = new DetectConeDisplay(true); // TODO depends on team
        camera.setPipeline(sleeveDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
                dashboard.startCameraStream(camera, 30);
            }
            @Override
            public void onError(int errorCode) {
            }
        });
        while (!isStarted()) {
            telemetry.addData("route: ", sleeveDetection.route);
            telemetry.update();
        }
    }

    @Override
    public abstract void runOpMode() throws InterruptedException;

}
