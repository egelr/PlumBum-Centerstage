package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.variables;

@TeleOp(name = "OpMode")
public class TeleOP extends LinearOpMode {
    //Arm Motors and Claw servos
    private DcMotor armLiftMotor;
    private DcMotor armSlideMotor;
    static final double MOTOR_TICK_COUNT = 8192;
    SimpleServo clawAngleServo;
    SimpleServo clawRightServo;
    SimpleServo clawLeftServo;
    //Drivetrain Motors speed origin
    public double drive_speed = 0.85;
    //Arm Motors speed
    double sp = 0.05;
    double sp2 = 0.05;
    //Claw positions (Park - basic/start), (Grip - pixel grabbing/parallel with the ground), (Board - pixel depositing/parallel with the Backboard)
    double clawDegreesPark;

    double clawDegreesGrip;

    double clawDegreesBoard;
    //Drivetrain motors
    private Motor fL, fR, bL, bR;
    boolean dpadLeftPressed = false;
    int arm_extended = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        //Creating the Drivetrain Motors
        fL = new Motor(hardwareMap, "dtFrontLeftMotor", Motor.GoBILDA.RPM_312);
        fR = new Motor(hardwareMap, "dtFrontRightMotor", Motor.GoBILDA.RPM_312);
        bL = new Motor(hardwareMap, "dtBackLeftMotor", Motor.GoBILDA.RPM_312);
        bR = new Motor(hardwareMap, "dtBackRightMotor", Motor.GoBILDA.RPM_312);
        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //Creating the Mecanum Drivetrain
        MecanumDrive drive = new MecanumDrive(fL, fR, bL, bR);
        //Creating the Arm Motors
        Motor m_motor = new Motor(hardwareMap, "armLiftMotor", 8192, 60);
        Motor m_motor_2 = new Motor(hardwareMap, "armSlideMotor", 1425.2, 117);
        telemetry.addData("Hardware: ", "Initialized");
        telemetry.update();
        m_motor.setRunMode(Motor.RunMode.PositionControl);
        m_motor_2.setRunMode(Motor.RunMode.PositionControl);
        m_motor.setPositionCoefficient(0.01);
        //double kP = m_motor.getPositionCoefficient();
        m_motor_2.setPositionCoefficient(0.05);
        //double kP_2 = m_motor_2.getPositionCoefficient();
        //Creating pos and pos2 by current Motor Positions
        m_motor_2.resetEncoder();
        int pos = m_motor.getCurrentPosition();
        int pos2 = m_motor_2.getCurrentPosition();
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_motor_2.setInverted(true);
        m_motor_2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_motor.setPositionTolerance(100);
        m_motor_2.setPositionTolerance(50);

        //Creating the Claw Servos
        clawAngleServo = new SimpleServo(
                hardwareMap, "clawAngleServo", 0, 180,
                AngleUnit.DEGREES
        );

        clawRightServo = new SimpleServo(
                hardwareMap, "clawRightServo", 10, 120,
                AngleUnit.DEGREES
        );

        clawLeftServo = new SimpleServo(
                hardwareMap, "clawLeftServo", 10, 120,
                AngleUnit.DEGREES
        );
        //Defining CLaw positions
        clawDegreesPark = clawAngleServo.getAngle();
        clawDegreesGrip = clawDegreesPark + variables.ClawAngleGrab;
        clawDegreesBoard = clawDegreesPark + variables.ClawAngleDeposit;

        telemetry.addData("ClawAngle: ", clawAngleServo.getAngle());
        telemetry.addData("ClawRight: ", clawRightServo.getAngle());
        telemetry.addData("ClawLeft: ", clawLeftServo.getAngle());
        telemetry.addData("motor: ", m_motor.getCurrentPosition());
        telemetry.addData("motor2: ", m_motor_2.getCurrentPosition());
        telemetry.update();
        //clawAngleServo.setInverted(true);
        ElapsedTime timer = new ElapsedTime();
        waitForStart();


        //Initializing TeleOP
        while (!isStopRequested()) {
            //Drivetrain controls
            drive.driveRobotCentric(
                    gamepad1.left_stick_x * drive_speed,
                    -gamepad1.left_stick_y * drive_speed,
                    gamepad1.right_stick_x * drive_speed,
                    false

            );
            //Drivetrain Motors speed Change
            if (gamepad1.right_trigger > 0.5) {
                drive_speed = 0.35;
            } else {
                drive_speed = 0.85;
            }
            //PARK (Robot starting/basic compact Position)
            if (gamepad1.triangle) {
                m_motor.setTargetPosition(pos);
                clawAngleServo.turnToAngle(clawDegreesPark);
                timer.reset();
                while (!m_motor.atTargetPosition() && timer.seconds() < variables.timer_motor) {
                    m_motor.set(variables.speed_extender);
                }
                m_motor.stopMotor();
                m_motor_2.setTargetPosition(pos2);

                timer.reset();
                while (!m_motor_2.atTargetPosition() && timer.seconds() < variables.timer_motor_2) {
                    m_motor_2.set(variables.speed_extender);
                }
                m_motor_2.stopMotor();
                telemetry.addData("Status: ", "Park");
                telemetry.update();

            }

            //PIXEL GRABBING (Picking up pixels, extended Arm Position)

            if (gamepad1.circle) {

                m_motor.setTargetPosition(pos);
                timer.reset();
                while (!m_motor.atTargetPosition() && timer.seconds() < variables.timer_motor ){
                    m_motor.set(variables.speed_extender);
                }
                m_motor.stopMotor();
                m_motor_2.setTargetPosition(pos2 + variables.reachingHigh);

                timer.reset();
                while (!m_motor_2.atTargetPosition() && timer.seconds() < variables.timer_motor_2 ){
                    m_motor_2.set(variables.speed_extender);
                }
                m_motor_2.stopMotor();
                clawAngleServo.turnToAngle(clawDegreesGrip);
                telemetry.addData("Status: ", "Grab");
                telemetry.update();

            }

            //DASHBOARD LOW (Lower Backboard pixel depositing Position)
            if (gamepad1.cross) {
                clawAngleServo.turnToAngle(clawDegreesBoard);
                m_motor.setTargetPosition(pos - variables.nearBoard);
                timer.reset();
                while (!m_motor.atTargetPosition() && timer.seconds() < variables.timer_motor ){
                    m_motor.set(variables.speed_arm);
                }
                m_motor.stopMotor();
                m_motor_2.setTargetPosition(pos2);
                timer.reset();
                while (!m_motor_2.atTargetPosition() && timer.seconds() < variables.timer_motor_2 ){
                    m_motor_2.set(variables.speed_extender);
                }
                m_motor_2.stopMotor();

                telemetry.addData("Status: ", "Dashboard low");
                telemetry.update();
            }

            //DASHBOARD HIGH (Upper Backboard pixel depositing Position)
            if (gamepad1.square) {

                m_motor.setTargetPosition(pos - variables.nearBoard);
                //m_motor.set(sp);
                timer.reset();
                while (!m_motor.atTargetPosition() && timer.seconds() < variables.timer_motor ){
                    m_motor.set(variables.speed_extender);
                }
                m_motor.stopMotor();
                m_motor_2.setTargetPosition(pos2 + variables.reachingHigh_Board);
                //m_motor_2.set(speed_extender);
                timer.reset();
                while (!m_motor_2.atTargetPosition() && timer.seconds() < variables.timer_motor_2 ){
                    m_motor_2.set(variables.speed_extender);
                }
                m_motor_2.stopMotor();
                clawAngleServo.turnToAngle(clawDegreesBoard);
                telemetry.addData("Status: ", "Dashboard high");
                telemetry.update();
            }
            //RECOVERY OUT (Recovery mode for extending the Arm)
            if (gamepad1.share) {
                pos2 = pos2 + 50;
                m_motor_2.setTargetPosition(pos2);

                timer.reset();
                while (!m_motor_2.atTargetPosition() && timer.seconds() < variables.timer_motor_2) {
                    m_motor_2.set(variables.speed_extender);
                }
                m_motor_2.stopMotor();
                telemetry.addData("Status: ", "Park");
                telemetry.update();

            }
            //RECOVERY IN (Recovery mode for reducing the length of the extension Arm)
            if (gamepad1.options) {
                pos2 = pos2 - 50;
                m_motor_2.setTargetPosition(pos2);

                timer.reset();
                while (!m_motor_2.atTargetPosition() && timer.seconds() < variables.timer_motor_2) {
                    m_motor_2.set(variables.speed_extender);
                }
                m_motor_2.stopMotor();
                telemetry.addData("Status: ", "Park");
                telemetry.update();

            }


            //Closed Claw Position
            if (gamepad1.dpad_up) {
                clawRightServo.turnToAngle(variables.gripDegrees1);
                clawLeftServo.turnToAngle(variables.gripDegrees);
                telemetry.addData("Hardware: ", clawRightServo.getAngle());
                telemetry.addData("Hardware: ", clawLeftServo.getAngle());
                telemetry.update();
            }

            //Opened Claw Position
            if (gamepad1.dpad_down) {
                clawRightServo.turnToAngle(variables.gripDegrees);
                clawLeftServo.turnToAngle(variables.gripDegrees1);
                telemetry.addData("Hardware: ", clawRightServo.getAngle());
                telemetry.addData("Hardware: ", clawLeftServo.getAngle());
                telemetry.update();
            }


            //Opening and Closing the Left and Right arms of the Claw separately
            if ((gamepad1.left_trigger > 0.5) && gamepad1.dpad_left) {

                clawLeftServo.turnToAngle(variables.gripDegrees1);
                telemetry.addData("Hardware: ", clawLeftServo.getAngle());
                telemetry.addData("dpad_left pressed: ", gamepad1.dpad_left);
                telemetry.update();
            }
            if ((gamepad1.left_trigger > 0.5) && gamepad1.dpad_right){
                clawRightServo.turnToAngle(variables.gripDegrees);
                telemetry.addData("Hardware: ", clawLeftServo.getAngle());
                telemetry.addData("dpad_left pressed: ", gamepad1.dpad_right);
                telemetry.update();
            }

            if ((gamepad1.left_trigger < 0.5) && gamepad1.dpad_left){
                clawLeftServo.turnToAngle(variables.gripDegrees);
                telemetry.addData("Hardware: ", clawLeftServo.getAngle());
                telemetry.addData("dpad_left pressed: ", gamepad1.dpad_left);
                telemetry.update();
            }
            if ((gamepad1.left_trigger < 0.5) && gamepad1.dpad_right){
                clawRightServo.turnToAngle(variables.gripDegrees1);
                telemetry.addData("Hardware: ", clawLeftServo.getAngle());
                telemetry.addData("dpad_left pressed: ", gamepad1.dpad_right);
                telemetry.update();
            }

        }



    }

}