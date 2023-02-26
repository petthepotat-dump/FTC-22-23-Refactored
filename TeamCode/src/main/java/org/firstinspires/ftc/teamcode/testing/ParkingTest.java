package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAuto;
import org.firstinspires.ftc.teamcode.wrappers.Vision;

@Config
@Autonomous(name="Parking Test")
public class ParkingTest extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        // red / blue team
        init(hardwareMap, false);

        // get vision
        Vision parkingDetect = new Vision();

        waitForStart();
        // move to wards detected position

        //  simulaete
        goTo(0.02, 1.23, 0, 1.7, 180, 0.04, 9, true);
        // parking go
        switch(parkingDetect.route){
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
                while (!control.finished) sleep(50);
                break;
            default:
                setArmPositionWait(0, 0.4);

        }

    }
}
