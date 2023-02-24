package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.wrappers.Controller;
import org.firstinspires.ftc.teamcode.wrappers.DetectPoleDisplay;
import org.firstinspires.ftc.teamcode.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.wrappers.Position;
import org.firstinspires.ftc.teamcode.wrappers.Vision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="teleop")

public class teleop extends LinearOpMode {
    private MecanumChassis robot;
    private Position pos;
    private DetectPoleDisplay poleDetection;
    private WebcamName webcamName;
    private OpenCvCamera camera;
    private double clamp(double min, double max, double val) {
        if (val<min) return min;
        else if (val>max) return max;
        else return val;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        int currentArmPosition=0;
        robot = new MecanumChassis(hardwareMap);
        pos = new Position(robot);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamName = hardwareMap.get(WebcamName.class, "Camera");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        poleDetection = new DetectPoleDisplay();
        camera.setPipeline(poleDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });
        waitForStart();
        setArmPosition(20, 0.3);
        while (opModeIsActive()) {
            telemetry.addData("Servo Position ", robot.intake.getPosition());
            telemetry.addData("Arm Position ", robot.leftArm.getCurrentPosition());
            telemetry.addData("Position Data", String.format("%.2f %.2f %.2f",pos.x,pos.y,pos.angle));
            telemetry.update();
            double lx = gamepad1.left_stick_x, ly = gamepad1.left_stick_y, rx = gamepad1.right_stick_x, ry = gamepad2.right_stick_y;
            double dn = 0.6/Math.max(Math.abs(lx)+0.7*Math.abs(rx)+Math.abs(ly),1);
            robot.fr.setPower((ly+lx+0.7*rx)*dn);
            robot.fl.setPower((ly-lx-0.7*rx)*dn);
            robot.br.setPower((ly-lx+0.7*rx)*dn);
            robot.bl.setPower((ly+lx-0.7*rx)*dn);
            if (gamepad2.dpad_up) {
                currentArmPosition = 520;
                setArmPosition(currentArmPosition, 0.3);
            } else if (gamepad2.b) {
                currentArmPosition = 365;
                setArmPosition(currentArmPosition, 0.3);
            } else if (gamepad2.y) {
                currentArmPosition = 260;
                setArmPosition(currentArmPosition, 0.3);
            } else if (gamepad2.x) {
                currentArmPosition = 70;
                setArmPosition(currentArmPosition, 0.3);
            } else if (gamepad2.dpad_down) {
                currentArmPosition = 20;
                setArmPosition(currentArmPosition, 0.2);
            } else if (gamepad2.dpad_left) {
                currentArmPosition = 0;
                robot.leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else if (Math.abs(ry)>0.1) {
                currentArmPosition=(int)clamp(-200,520,robot.leftArm.getCurrentPosition()-35*ry);
                setArmPosition(currentArmPosition,Math.abs(ry)*(ry>0?0.15:0.25));
            }
            if (gamepad2.left_bumper) robot.intake.setPosition(0.55);
            else robot.intake.setPosition(0.8);
        }
    }
    private void setArmPosition(int pos, double speed) {
        robot.leftArm.setTargetPosition(pos);
        robot.rightArm.setTargetPosition(pos);
        robot.leftArm.setPower(speed);
        robot.rightArm.setPower(speed);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
//    private void goToPole() {
//        while (!isStopRequested() && (!(Math.abs(poleDetection.widthError) < 4 && Math.abs(poleDetection.error) < 5))) {
//            robot.fl.setPower(-poleDetection.error * 0.002+poleDetection.widthError * 0.01);
//            robot.fr.setPower(poleDetection.error * 0.002+poleDetection.widthError * 0.01);
//            robot.bl.setPower(-poleDetection.error * 0.002+poleDetection.widthError * 0.01);
//            robot.br.setPower(poleDetection.error * 0.002+poleDetection.widthError * 0.01);
//            telemetry.addData("error: ", poleDetection.error);
//            telemetry.addData("widthError: ", poleDetection.widthError);
//            telemetry.update();
//            sleep(10);
//        }
//    }
}
