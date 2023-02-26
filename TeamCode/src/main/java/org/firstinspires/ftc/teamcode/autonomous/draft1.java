package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.wrappers.Utils;

@Autonomous(name="Draft1")
public class draft1 extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        // ===== step 0 - init + wait for start =====
        init(hardwareMap, true);
        waitForStart();
        Utils.Timer timer = new Utils.Timer();

        // ===== step 1 - place 1st cone =====
        closeIntake();
        setArmPositionTiming(520, 0.2, 1000);
        goTo(0, 1.12, 45, 2.3, 90, 0.12, 10, true);
        goToPole(false);
        setArmPositionWait(420, 0.3);
        openIntake();

//        pos.updatePos(0.2, 1.2);
        //  ===== cycle =====
        // starting at (0, 1.2)
        // ======
        // stack is the 5 * 5 + extra to grip right place
        int cycleTimeLeft = 30000 - timer.timePassedAbs();
        int stack = 28, cone = 5, count = 5;
        Utils.TimerThread timerThread = new Utils.TimerThread(10000);
        while (count > 0 && !timerThread.done){
            // === go cycle speed -- go towards cone stack
            goTo(-0.4, 1.0, -90, 2.3, 90, 0.12, 9, true);
            goToCone(90);

            // approaching stack
            goToRel(-0.3, -0.1, pos.angle - -90, 0.5, 0, 0.12, 9, true);
            openIntake();
            setArmPosition(stack - cone * count, 0.1);
            goTo(-0.4, 1.2, -90, 0.5, 0, 0.12, 9, true);
            closeIntake();
            setArmPosition(stack + 50, 0.2);
            goToRel(0.2, 0, 0, 0.5, 0, 0.12, 9, true);

            // move and arm up
            goTo(0, 1.2, 0, 0.5, 0, 0.12, 9, true);
            setArmPositionTiming(520, 0.2, 1000);

            // place
            goToPole(true);
        }

    }
}
