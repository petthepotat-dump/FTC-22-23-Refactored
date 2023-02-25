package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auton")
public class auton extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap, true);
        waitForStart();
        closeIntake();
        setArmPositionTiming(520, 0.2, 1000);
        goTo(0, 1.12, 45, 2.3, 90, 0.12, 10, true);
        goToPole(false);
        setArmPositionWait(420, 0.3);
        openIntake();

//        pos.updatePos(0.2, 1.2);
    }
}
