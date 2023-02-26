package org.firstinspires.ftc.teamcode.autonomous;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auton")
public class auton extends BaseAuto {
    private String route = "NONE";
    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap, true);
        if (isStopRequested()) return;
        while (!isStarted()) {
            route = sleeveDetection.route;
            FtcDashboard.getInstance().getTelemetry().addData("Route", route);
            FtcDashboard.getInstance().getTelemetry().update();
            sleep(50);
            if (isStopRequested()) return;
        }
        waitForStart();
        route = sleeveDetection.route;
        closeIntake();
        sleep(1000);
        control.goTo(-0.04, 1.14, 0 , 2.9, 120, 0.1, 6, false);
        sleep(700);
        setArmPosition(520, 0.4);
        control.updatePos(0.1, 1.27, 40, true);
        while (!control.finished) sleep(80);
//        goTo(0.07, 1.26, 45, 2.6, 90, 0.2, 8, false);
        for (int i=0; i<2; i++) {
            goToPole(false);
            pos.updatePos(0.12, 1.39);
            setArmPositionWait(420, 0.4);
            openIntake();
            setArmPosition(520, 0.4);
            control.goTo(-0.05, 1.36, -90, 1.4, 240, 0.2, 6, false);
//        sleep(1600);
//        control.updatePos(-0.4, 1.1, -90);
            sleep(200);
            setArmPosition(200, 0.3);
            control.updatePos(-0.28, 1.27, -90, true);
            while (!control.finished) sleep(80);
            goToCone(90-i*18);
            pos.updatePos(-0.52, 1.38);
//        pos.updatePos(-0.4, 1.1);
//        goTo(-0.48, 1.1, -90, 1.1, 10, 0.02, 5, true);
            closeIntake();
            sleep(400);
            setArmPositionWait(200, 0.5);
            setArmPosition(520, 0.15);
            control.goTo(-0.03, 1.2, 0, 2.5, 240, 0.05, 5, false);
            sleep(500);
            control.updatePos(0.03, 1.32, 40, true);
            while(!control.finished) sleep(80);
        }
        goToPole(false);
        pos.updatePos(0.12, 1.39);
        setArmPositionWait(420, 0.4);
        openIntake();
        setArmPosition(520, 0.4);
//
        goTo(0.02, 1.23, 90, 1.7, 250, 0.3, 10, true);
//        while (!control.finished) sleep(50);
        switch (route) {
            case "LEFT":
                sleep(200);
                setArmPosition(0, 0.4);
                goTo(-0.58, 1.23, 90,2.4, 170, 0.3, 10,true);
//                while (!control.finished) sleep(50);
                break;
            case "RIGHT":
                sleep(350);
                setArmPosition(0, 0.4);
                goTo(0.58, 1.23, 90,2.4, 170, 0.3, 10,true);
//                while (!control.finished) sleep(50);
                break;
            default:
                setArmPositionWait(0, 0.4);
        }


        // target
//        goTo()

//        pos.updatePos(0.2, 1.2);
    }
}
