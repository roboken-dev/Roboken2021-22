package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="DuckParkStorageBlue")
public class DuckParkStorageBlue extends LinearOpMode {

    donkeybot robot = new donkeybot();


    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap,this);

        waitForStart();

        robot.encoderDrive(0.3, 4, 4, 5, this);

        robot.spin.setPower(1);
        sleep(4000);
        robot.spin.setPower(0);

        robot.turnLeftAngle(0.4,50,this);
        robot.encoderDrive(0.3,22, 27, 11, this);
    }



}
