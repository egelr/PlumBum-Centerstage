package org.firstinspires.ftc.teamcode;
//A class for checking lift positions and fixing them
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.variables;

@TeleOp(name = "Lift")
public class Lift extends LinearOpMode {
    private Motor LeftLiftMotor, RightLiftMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        Motor LeftLiftMotor = new Motor(hardwareMap, "LeftLiftMotor");
        Motor RightLiftMotor = new Motor(hardwareMap, "RightLiftMotor");
        LeftLiftMotor.resetEncoder();
        RightLiftMotor.resetEncoder();
        LeftLiftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        RightLiftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        RightLiftMotor.setRunMode(Motor.RunMode.RawPower);
        LeftLiftMotor.setRunMode(Motor.RunMode.RawPower);
        RightLiftMotor.setInverted(true);

        ElapsedTime timer = new ElapsedTime();
        waitForStart();

        while (!isStopRequested())
        {
            telemetry.addData("left: ", LeftLiftMotor.getCurrentPosition());
            telemetry.addData("right: ", RightLiftMotor.getCurrentPosition());
            telemetry.update();
            if (gamepad1.right_bumper)
            {
                LeftLiftMotor.set(1);
                RightLiftMotor.set(1);
            }
            if (gamepad1.left_bumper)
            {
                LeftLiftMotor.set(-1);
                RightLiftMotor.set(-1);
            }
            if (gamepad1.dpad_up)
            {
                LeftLiftMotor.set(-0.5);
            }
            if (gamepad1.dpad_down)
            {
                LeftLiftMotor.set(0.5);
            }
            if (gamepad1.triangle)
            {
                RightLiftMotor.set(-0.5);
            }
            if (gamepad1.cross)
            {
                RightLiftMotor.set(0.5);
            }
            LeftLiftMotor.set(0);
            RightLiftMotor.set(0);
        }
    }

}
