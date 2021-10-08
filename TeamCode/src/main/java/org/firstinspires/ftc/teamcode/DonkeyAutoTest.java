package org.firstinspires.ftc.teamcode;
        import android.graphics.Color;

        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.hardware.DcMotor;

        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="DonkeyAutoTest")

public class DonkeyAutoTest extends LinearOpMode {

    donkeybot robot = new donkeybot();


    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap,this);

        waitForStart();

        robot.encoderDrive(0.3, 12, 12, 10, this);

        robot.turnLeftAngle(0.05,90,this);

    }



}



