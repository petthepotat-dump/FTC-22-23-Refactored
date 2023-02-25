package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    public DetectConeDisplay coneDetection;
    public WebcamName webcamName;
    public OpenCvCamera camera;
    public String route;
    public FtcDashboard dashboard = FtcDashboard.getInstance();

    public void init(HardwareMap map, boolean blueTeam){
        robot = new MecanumChassis(map);
        pos = new Position(robot);
        control = new Controller(robot, pos);
        int cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamName = hardwareMap.get(WebcamName.class, "Camera");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        sleeveDetection = new Vision();
        poleDetection = new DetectPoleDisplay();
        coneDetection = new DetectConeDisplay(blueTeam); // TODO depends on team
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
    public void closeIntake() {
        robot.intake.setPosition(0.78);
    }
    public void openIntake() {
        robot.intake.setPosition(0.55);
    }
    public void goTo(double x, double y, double angle, double speed, double angleSpeed, double distanceDeadzone, double angleDeadzone, boolean velocityControl) {
        control.goTo(x, y, angle, speed, angleSpeed, distanceDeadzone, angleDeadzone, velocityControl);
        while (!control.finished) {
            if (isStopRequested()) control.stop();
            telemetry.addData("Angle: ", pos.angle);
            telemetry.addData("X: ", pos.x);
            telemetry.addData("Y: ", pos.y);
            telemetry.update();
            sleep(10);
        }
    }
    public void goToRel(double x, double y, double angle, double speed, double angleSpeed, double distanceDeadzone, double angleDeadzone, boolean velocityControl) {
        goTo(pos.x+x, pos.y+y, pos.angle+angle, speed, angleSpeed, distanceDeadzone, angleDeadzone, velocityControl);
    }
    public void setArmPositionWait(int pos, double speed) {
        setArmPosition(pos, speed);
        while (!isStopRequested() && robot.leftArm.isBusy()) sleep(10);
    }
    public void setArmPositionTiming(int pos, double speed, int delay) {
        sleep(delay);
        setArmPosition(pos, speed);
    }
    public void setArmPosition(int pos, double speed){
        robot.leftArm.setTargetPosition(pos);
        robot.rightArm.setTargetPosition(pos);
        robot.leftArm.setPower(speed);
        robot.rightArm.setPower(speed);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void move(double fr, double fl, double br, double bl) {
        robot.fr.setPower(fr);
        robot.fl.setPower(fl);
        robot.br.setPower(br);
        robot.bl.setPower(bl);
    }
    public void goToCone() {
        camera.setPipeline(coneDetection);
        while (coneDetection.x<118 || coneDetection.x>128 || coneDetection.width<150) {
            double error = 123-coneDetection.x, distanceError = coneDetection.width-150;
            double power = error/350, distancePower = distanceError/300;
            move(-power+distancePower, power+distancePower, power+distancePower, -power+distancePower);
            telemetry.addData("error", error);
            telemetry.addData("distance_error", distanceError);
            telemetry.update();
            sleep(100);
        }
    }
    public void goToPole(boolean cone) {
        camera.setPipeline(poleDetection);
        poleDetection.poleHasCone(cone);
        if (cone) {
            while (poleDetection.x<118 || poleDetection.x>128 || poleDetection.width<150) {
                double error = 123-poleDetection.x, distanceError = poleDetection.width-150;
                double power = error/350;
                double distancePower = clamp(distanceError/300, 0.07, 0.4);
                move(-power+distancePower, power+distancePower, power+distancePower, -power+distancePower);
                telemetry.addData("error", error);
                telemetry.addData("distance_error", distanceError);
                telemetry.update();
                sleep(100);
            }
        } else {
            while (poleDetection.x<97 || poleDetection.x>107 || poleDetection.width<65) {
                double error = 102-poleDetection.x, distanceError = poleDetection.width-65;
                double power = error/350;
                double distancePower = clamp(distanceError/300, 0.07, 0.4);
                move(-power+distancePower, power+distancePower, power+distancePower, -power+distancePower);
                telemetry.addData("error", error);
                telemetry.addData("distance_error", distanceError);
                telemetry.update();
                sleep(100);
            }
        }
    }
    public static double clamp(double val, double min, double max){
        return Math.max(min, Math.min(max, val));
    }
    @Override
    public abstract void runOpMode() throws InterruptedException;

}
