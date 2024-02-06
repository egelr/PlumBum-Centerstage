package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
public class Auto_RedFar extends LinearOpMode {
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
            Scalar lowerRed = new Scalar(100, 100, 100);
            Scalar upperRed = new Scalar(180, 255, 255);


            Mat redMask = new Mat();
            Core.inRange(hsvFrame, lowerRed, upperRed, redMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_CLOSE, kernel);

            return redMask;
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
    }
    @Override
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

        Motor m_motor = new Motor(hardwareMap, "armLiftMotor", 8192, 60);
        m_motor.setRunMode(Motor.RunMode.PositionControl);
        m_motor.setPositionCoefficient(0.01);
        int pos = m_motor.getCurrentPosition();
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_motor.setPositionTolerance(100);
        Pose2d startPose = new Pose2d(0, 0, 0);
        ElapsedTime timer = new ElapsedTime();

        drive.setPoseEstimate(startPose);

        Trajectory Center_1 = drive.trajectoryBuilder(startPose)
                .back(variables.CenterBack)
                .build();
        Trajectory Left_11 = drive.trajectoryBuilder(startPose)
                .back(variables.LeftBack)
                .build();
        Trajectory Left_12 = drive.trajectoryBuilder(Left_11.end())
                .strafeLeft(-1*(variables.LeftLeft))
                .build();
        Trajectory Left_13 = drive.trajectoryBuilder(Left_12.end().plus(new Pose2d(0,0,Math.toRadians(140))))
                .forward(variables.LeftForward)
                .build();

        Trajectory Right_11 = drive.trajectoryBuilder(startPose)
                .back(variables.RightBack)
                .build();
        Trajectory Right_12 = drive.trajectoryBuilder(Right_11.end())
                .strafeLeft(-1*(variables.RightLeft))
                .build();
        Trajectory NF_11 = drive.trajectoryBuilder(startPose)
                .back (variables.NFBackFar1)
                .build();
        Trajectory NF_12 = drive.trajectoryBuilder(NF_11.end().plus(new Pose2d(0,0,Math.toRadians(136))))
                .forward (variables.NFForwardFar)
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
        timer.reset();
        while (timer.seconds() < 2);

        telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
        telemetry.addData("Distance in Inch", (getDistance(width)));
        telemetry.update();

        if ((int) cX > 200 && (int) cX < 400){
            position = variables.CENTRE;
            drive.followTrajectory(Center_1);

        }

        else if ((int) cX > 0 && (int) cX < 200){
            position = variables.LEFT;
            drive.followTrajectory(Right_11);
            drive.followTrajectory(Right_12);

        }
        else if ((int) cX > 400 && (int) cX < 700){
            position = variables.RIGHT;
            drive.followTrajectory(Left_11);
            drive.followTrajectory(Left_12);
            drive.turn(Math.toRadians(-140));
            drive.followTrajectory(Left_13);


        }
        else {
            position = variables.NOTDETECTED;
            drive.followTrajectory(NF_11);
            drive.turn(Math.toRadians(136));
            drive.followTrajectory(NF_12);
        }
        controlHubCam.stopStreaming();
//Raise the arm
        m_motor.setTargetPosition(pos - 400);
        //m_motor.set(sp);
        timer.reset();
        while (!m_motor.atTargetPosition() && timer.seconds() < variables.timer_motor ){
            m_motor.set(variables.speed_extender);
        }
        m_motor.stopMotor();

//Drop the pixels with the Claw
        clawAngleServo.turnToAngle(variables.AutoCLawDown);
        timer.reset();
        while (timer.seconds() < 1);// drive.update();

        if (position == variables.NOTDETECTED){
            clawRightServo.turnToAngle(variables.gripDegrees);
        }
        clawLeftServo.turnToAngle(variables.gripDegrees1);
        timer.reset();
        while (timer.seconds() < 2);// drive.update();

        if (position == variables.NOTDETECTED){
            clawRightServo.turnToAngle(variables.gripDegrees1);
        }
        clawLeftServo.turnToAngle(variables.gripDegrees);

        timer.reset();
        while (timer.seconds() < 1);// drive.update();
        clawAngleServo.turnToAngle(variables.AutoCLawPark);
        while (timer.seconds() < 2);
        m_motor.setTargetPosition(pos);
        //m_motor.set(sp);
        timer.reset();
        while (!m_motor.atTargetPosition() && timer.seconds() < variables.timer_motor ){
            m_motor.set(variables.speed_extender);
        }
        m_motor.stopMotor();
        telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
        telemetry.addData("Distance in Inch", (getDistance(width)));
        telemetry.update();


        // Release resources

    }

}


