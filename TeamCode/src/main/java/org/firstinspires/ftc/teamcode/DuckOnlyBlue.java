package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="DuckOnlyBlue")
public class DuckOnlyBlue extends LinearOpMode {

    donkeybot robot = new donkeybot();


    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap,this);

        waitForStart();

        robot.encoderDrive(0.3, 4, 4, 5, this);

        robot.spin.setPower(1);
        sleep(4000);
        robot.spin.setPower(0);

    }



}
