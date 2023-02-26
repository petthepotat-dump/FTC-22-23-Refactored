package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.wrappers.Controller;
import org.firstinspires.ftc.teamcode.wrappers.DetectPoleDisplay;
import org.firstinspires.ftc.teamcode.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.wrappers.Position;
import org.firstinspires.ftc.teamcode.wrappers.Utils;
import org.firstinspires.ftc.teamcode.wrappers.Vision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="Auton Left")
public class AutonLeft extends BaseAuto {

    // dashboard variables
    public volatile static double POS1X = 0.0, POS1Y = 1.15;
    public volatile static int MOVE1P = 400, ARM1P = 600;
    public volatile static long PARKINGTIME = 4 * 10000;

    public volatile static double POS2X = -0.66, POS2Y = POS1Y,
                PIVOT1X = -0.5, PIVOT1Y = POS2Y;

    public volatile static int STAGES = 10;


    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap, true);

//        route = sleeveDetection.route;
        waitForStart();

        Utils.TimerThread timer = new Utils.TimerThread(PARKINGTIME);

        closeIntake();
        camera.setPipeline(poleDetection);

        // ----------------- Autonomous ----------------- //
        // move to center position
        setArmPositionTiming(150, 0.2, 900); // arm1p

        if (STAGES < 1) return;
        setArmPositionTiming(520, 0.05, MOVE1P);
        goTo(POS1X,  POS1Y-0.2, 30, 2.0, 200, 0.04, 2, true);


        goToPole(false);

        setArmPositionWait(400, 0.2);
        openIntake();
        telemetry.addData("Stage", "CYCLE!");
        telemetry.update();
        if (STAGES < 2) return;
        // next


        // begin cycle when arrive at side pylon stack
//        Utils.PylonStackTracker ptracker = new Utils.PylonStackTracker();
        // intiial == at left side (arm down h = 20)
        int heightOfStack = 20*5;
        while(heightOfStack > 0) {
            // TODO I THINK WE NEED THIS
            telemetry.addData("height of stack", heightOfStack/20);
            telemetry.update();
            goTo(POS1X,POS1Y, -30, 0.4, 100, 0.04, 2, true);
            // then move towards left pylon stack
            setArmPosition(200,0.2);
            goTo(POS2X+0.35, POS2Y+0.1, -90, 1.0, 100/*this was 300 and i think that breaks it*/, 0.04, 2, true);
            goToCone(90);
            setArmPositionTiming(heightOfStack, 0.2, 300);
            goTo(pos.x-0.11, pos.y, -90, 0.4, 1, 0.03, 5, true);

            // ------------- //
            // move the robot cycle-wise as many times as possible
//            setArmPosition(ptracker.getPylonStackHeight(), 0.4);
            closeIntake();
            setArmPositionWait(100, 0.3);
            // move back a bit + spin around -- then move diagonal towards junction
            goTo(POS2X+0.35, POS2Y+0.1, -90, 1.0, 100/*this was 300 and i think that breaks it*/, 0.04, 2, true);

            setArmPositionTiming(520, 0.3, 400);
//            goTo(PIVOT1X, PIVOT1Y, 50, 1.4, 300, 0.04, 2, true);
            goTo(POS1X,POS1Y, 30, 0.4, 100, 0.04, 2, true);

            if (heightOfStack < 3*20) {
                goToPole(true);
            } else {
                goToPole(false);
            }
            // yeet cone down
            setArmPositionWait(400, 0.4);
            openIntake();
            heightOfStack -= 20;
         }

        if(STAGES < 3) return;
        // next

        // endgame movement
//        telemetry.addData("route: ", route);
//        telemetry.update();
//
//        // removed endgame movement
//        route = "NONE";
//
//        switch (route) {
//            case "LEFT":
//                goTo(-0.55,1.17,0,1.2, 200,0.04,2,true);
//                setArmPositionTiming(0, 0.2, 0);
//                goTo(-0.55,0.85,0,1.2, 200,0.04,2,true);
//                break;
//            case "CENTER":
//                goTo(0,1.35,0,1.8,200,0.04,15,true);
//                setArmPositionTiming(0, 0.2, 0);
//                goTo(0,0.9,0,1.2,150,0.04,2,true);
//                break;
//            case "RIGHT":
//                setArmPositionTiming(520, 0.2, 0);
//                goTo(0.6,1.4,0,1.8,200,0.04,15,true);
//                setArmPositionTiming(0, 0.2, 0);
//                goTo(0.6,0.9,0,1.2,150,0.04,2,true);
//                break;
//            default:
//                telemetry.addData("OH SHIT!","WE FUCKED UP!");
//                telemetry.update();
//                goTo(-0.55,1.17,0,1.2, 200,0.04,2,true);
//                setArmPositionTiming(0, 0.2, 0);
//                goTo(-0.55,0.85,0,1.2, 200,0.04,2,true);
//                break;
//        }
    }
}
