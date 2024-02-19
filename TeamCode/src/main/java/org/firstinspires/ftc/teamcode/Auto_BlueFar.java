package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.variables;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@Autonomous
public class Auto_BlueFar extends LinearOpMode {
    SimpleServo clawAngleServo;
    SimpleServo clawRightServo;
    SimpleServo clawLeftServo;
    double cX = 0;
    double cY = 0;
    double width = 0;

    int position = 0;


    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels
    public ElapsedTime timer;
    public Motor m_motor;
    public int pos;


    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "webcam1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

    class YellowBlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);
            //BLUE
            Scalar lowerBlue = new Scalar(25,50,50);
            Scalar upperBlue = new Scalar(32,255,255);


            Mat blueMask = new Mat();
            Core.inRange(hsvFrame, lowerBlue, upperBlue, blueMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_CLOSE, kernel);

            return blueMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }

        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }

    private static double getDistance(double width) {
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    } @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        clawRightServo = new SimpleServo(
                hardwareMap, "clawRightServo", 10, 120,
                AngleUnit.DEGREES
        );

        clawLeftServo = new SimpleServo(
                hardwareMap, "clawLeftServo", 10, 120,
                AngleUnit.DEGREES
        );
        clawAngleServo = new SimpleServo(
                hardwareMap, "clawAngleServo", 0, 180,
                AngleUnit.DEGREES
        );
        m_motor = new Motor(hardwareMap, "armLiftMotor", 8192, 60);
        m_motor.setRunMode(Motor.RunMode.PositionControl);
        m_motor.setPositionCoefficient(0.01);
        pos = m_motor.getCurrentPosition();
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_motor.setPositionTolerance(100);
        Pose2d startPose = new Pose2d(0, 0);
        timer = new ElapsedTime();

        drive.setPoseEstimate(startPose);

        Trajectory Center_1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(variables.CenterBack, 0), 0)
                .build();
        Trajectory Center_15 = drive.trajectoryBuilder(Center_1.end())
                .strafeRight(15)
                .build();
        Trajectory Center_16 = drive.trajectoryBuilder(Center_15.end())
                .forward(22)
                .build();
        Trajectory Center_17 = drive.trajectoryBuilder(Center_16.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .back(70)
                .build();
        Trajectory Center_2 = drive.trajectoryBuilder(Center_17.end(),true)
                .splineTo(new Vector2d(29, 80), Math.toRadians(90))
                .build();
        Trajectory Center_3 = drive.trajectoryBuilder(Center_2.end())
                .back (8)
                .build();
        Trajectory Center_4 = drive.trajectoryBuilder(Center_3.end())
                .strafeLeft(23)
                .build();
        Trajectory Center_5 = drive.trajectoryBuilder(Center_4.end())
                .back(10)
                .build();
        Trajectory Obs_11 = drive.trajectoryBuilder(startPose)
                .splineTo (new Vector2d(31, -5), Math.toRadians(90))
                .forward(5)
                .build();
        Trajectory Obs_115 = drive.trajectoryBuilder(Obs_11.end())
                .back(7)
                .build();
        Trajectory Obs_116 = drive.trajectoryBuilder(Obs_115.end())
                .strafeRight(23)
                .build();
        Trajectory Obs_117 = drive.trajectoryBuilder(Obs_116.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                .back(70)
                .build();
        Trajectory Obs_12 = drive.trajectoryBuilder(Obs_117.end(), true)
                .splineTo(new Vector2d(29, 80), Math.toRadians(90))
                .build();
        Trajectory Obs_13 = drive.trajectoryBuilder(Obs_12.end())
                .back (8)
                .build();
        Trajectory Obs_14 = drive.trajectoryBuilder(Obs_13.end())
                .strafeLeft(28)
                .build();
        Trajectory Obs_15 = drive.trajectoryBuilder(Obs_14.end())
                .back(10)
                .build();
        Trajectory Free_11 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(variables.RightBack, -14), 0)
                .build();
        Trajectory Free_115 = drive.trajectoryBuilder(Free_11.end())
                .strafeLeft(14)
                .build();
        Trajectory Free_116 = drive.trajectoryBuilder(Free_115.end())
                .forward(27)
                .build();
        Trajectory Free_117 = drive.trajectoryBuilder(Free_116.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .back(70)
                .build();
        Trajectory Free_12 = drive.trajectoryBuilder(Free_117.end(), true)
                .splineTo(new Vector2d(34, 80), Math.toRadians(90))
                .build();
        Trajectory Free_13 = drive.trajectoryBuilder(Free_12.end())
                .back (8)
                .build();
        Trajectory Free_14 = drive.trajectoryBuilder(Free_13.end())
                .strafeLeft(16)
                .build();
        Trajectory Free_15 = drive.trajectoryBuilder(Free_14.end())
                .back(10)
                .build();
        Trajectory NF_01 = drive.trajectoryBuilder(startPose)
                .forward(50)
                .build();
        Trajectory NF_02 = drive.trajectoryBuilder(NF_01.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .back(70)
                .build();
        Trajectory NF_11 = drive.trajectoryBuilder(NF_02.end(),true)
                .splineTo(new Vector2d(29, 80), Math.toRadians(90))
                .build();
        Trajectory NF_12 = drive.trajectoryBuilder(NF_11.end())
                .back (8)
                .build();
        Trajectory NF_13 = drive.trajectoryBuilder(NF_12.end())
                .strafeLeft(23)
                .build();
        Trajectory NF_14 = drive.trajectoryBuilder(NF_13.end())
                .back(10)
                .build();

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        waitForStart();

        if (isStopRequested())
            return;

        clawRightServo.turnToAngle(variables.gripDegrees1);
        clawLeftServo.turnToAngle(variables.gripDegrees);
        clawAngleServo.turnToAngle(variables.AutoCLawDown);
        timer.reset();
        while (timer.seconds() < 1);


        if ((int) cX > 200 && (int) cX < 400){
            position = variables.CENTRE;
            drive.followTrajectory(Center_1);
        }

        else if ((int) cX > 0 && (int) cX < 200){
            position = variables.LEFT;
            drive.followTrajectory(Obs_11);
        }
        else if ((int) cX > 400 && (int) cX < 700){
            position = variables.RIGHT;
            drive.followTrajectory(Free_11);


        }
        else {
            position = variables.NOTDETECTED;
            ClawPark();
            drive.followTrajectory(NF_01);
            drive.turn(Math.toRadians(-90));
            drive.followTrajectory(NF_02);
            drive.followTrajectory(NF_11);
        }
        controlHubCam.stopStreaming();


        if (position != variables.NOTDETECTED) {
            DropPixel(true, false);
            ClawPark();
        }

        if (position == variables.NOTDETECTED) {
            ArmUP();
            drive.followTrajectory(NF_12);
            DropPixel(true, true);
            ArmFromBoard();
            ArmPark();
            drive.followTrajectory(NF_13);
            drive.followTrajectory(NF_14);
        }

        if (position == variables.CENTRE){
            drive.followTrajectory(Center_15);
            drive.followTrajectory(Center_16);
            drive.turn(Math.toRadians(-90));
            drive.followTrajectory(Center_17);
            drive.followTrajectory(Center_2);
            ArmUP();
            drive.followTrajectory(Center_3);
            DropPixel(false, true);
            ArmFromBoard();
            ArmPark();
            drive.followTrajectory(Center_4);
            drive.followTrajectory(Center_5);

        }
        if (position == variables.RIGHT){
            drive.followTrajectory(Free_115);
            drive.followTrajectory(Free_116);
            drive.turn(Math.toRadians(-90));
            drive.followTrajectory(Free_117);
            drive.followTrajectory(Free_12);
            ArmUP();
            drive.followTrajectory(Free_13);
            DropPixel(false, true);
            ArmFromBoard();
            ArmPark();
            drive.followTrajectory(Free_14);
            drive.followTrajectory(Free_15);

        }
        if (position == variables.LEFT){
            drive.followTrajectory(Obs_115);
            drive.followTrajectory(Obs_116);
            drive.turn(Math.toRadians(180));
            drive.followTrajectory(Obs_117);
            drive.followTrajectory(Obs_12);
            ArmUP();
            drive.followTrajectory(Obs_13);
            DropPixel(false, true);
            ArmFromBoard();
            ArmPark();
            drive.followTrajectory(Obs_14);
            drive.followTrajectory(Obs_15);

        }


        // Release resources

    }



    void ClawDown()
    {
        clawAngleServo.turnToAngle(variables.AutoCLawDown);
    /*timer.reset();
    while (timer.seconds() < 1);
*/
    }
    void DropPixel(boolean Purple, boolean Yellow)
    {
        if (Purple) clawLeftServo.turnToAngle(variables.gripDegrees1);
        if (Yellow) clawRightServo.turnToAngle(variables.gripDegrees);
        timer.reset();
        while (timer.seconds() < 0.2) ;

    }
    void ClawPark()
    {

        clawLeftServo.turnToAngle(variables.gripDegrees);
        clawRightServo.turnToAngle(variables.gripDegrees1);
        clawAngleServo.turnToAngle(variables.AutoCLawPark);

    }
    void ArmUP()
    {
        clawAngleServo.turnToAngle(variables.ClawAngleDeposit);
        m_motor.setTargetPosition(pos - variables.nearBoard);
        timer.reset();
        while (!m_motor.atTargetPosition() && timer.seconds() < variables.timer_motor ){
            m_motor.set(variables.speed_arm);
        }
        m_motor.stopMotor();
    }
    void ArmFromBoard()
    {
        clawAngleServo.turnToAngle(variables.ClawAngleDeposit);
        m_motor.setTargetPosition(pos - variables.nearBoard+200);
        timer.reset();
        while (!m_motor.atTargetPosition() && timer.seconds() < variables.timer_motor ){
            m_motor.set(variables.speed_arm);
        }
        m_motor.stopMotor();
    }
    void ArmPark(){
        ClawPark();
        m_motor.setTargetPosition(pos);
        timer.reset();
        while (!m_motor.atTargetPosition() && timer.seconds() < variables.timer_motor ){
            m_motor.set(variables.speed_arm);
        }
        m_motor.stopMotor();

    }
}
