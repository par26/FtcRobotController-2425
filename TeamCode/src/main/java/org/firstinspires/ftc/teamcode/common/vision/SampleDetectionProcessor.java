package org.firstinspires.ftc.teamcode.common.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;



import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class SampleDetectionProcessor implements VisionProcessor {
    // Working image buffers
    private Mat ycrcbMat = new Mat();
    private Mat crMat = new Mat();
    private Mat cbMat = new Mat();

    private Mat blueThresholdMat = new Mat();
    private Mat redThresholdMat = new Mat();
    private Mat yellowThresholdMat = new Mat();

    private Mat morphedBlueThreshold = new Mat();
    private Mat morphedRedThreshold = new Mat();
    private Mat morphedYellowThreshold = new Mat();

    private Mat contoursOnPlainImageMat = new Mat();

    private Telemetry telemetry;

    // Configuration parameters
    public double minArea = 40;
    public double aspectRatioThresh = 0.5;

    // Threshold values
    public int YELLOW_MASK_THRESHOLD = 57;
    public int BLUE_MASK_THRESHOLD = 150;
    public int RED_MASK_THRESHOLD = 198;

    // Morphological elements
    private Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));
    private Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));

    // Colors
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar YELLOW = new Scalar(255, 255, 0);
    static final Scalar HIGHLIGHT_COLOR = new Scalar(0, 255, 0);

    static final int CONTOUR_LINE_THICKNESS = 2;
    static final int HIGHLIGHT_THICKNESS = 3;

    // Object tracking
    private AnalyzedStone closestStone = null;
    private double closestDistance = Double.MAX_VALUE;

    // Object dimensions
    public double objectWidth = 3.5;
    public double objectHeight = 1.5;

    // Camera parameters
    private Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
    private MatOfDouble distCoeffs = new MatOfDouble();

    private ArrayList<AnalyzedStone> internalStoneList = new ArrayList<>();
    private volatile ArrayList<AnalyzedStone> clientStoneList = new ArrayList<>();

    static class AnalyzedStone {
        double angle;
        String color;
        Mat rvec;
        Mat tvec;
        double dist;
    }

    public SampleDetectionProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;


    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

        // Initialize camera parameters
        double fx = 800;
        double fy = 800;
        double cx = 320;
        double cy = 240;

        cameraMatrix.put(0, 0,
                fx, 0, cx,
                0, fy, cy,
                0, 0, 1);

        distCoeffs = new MatOfDouble(0, 0, 0, 0, 0);

        cleanup();
    }

    @Override
    public Mat processFrame(Mat input, long frameTime) {
        internalStoneList.clear();
        closestDistance = Double.MAX_VALUE;
        closestStone = null;

        // Process frame
        findContours(input);

        if (closestStone != null) {
            telemetry.addLine("Closest Dist: " + closestStone.dist);
            drawAxis(input, closestStone.rvec, closestStone.tvec, cameraMatrix, distCoeffs);
        }

        clientStoneList = new ArrayList<>(internalStoneList);
        telemetry.update();

        return input;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Implement any additional canvas drawing if needed
        if (closestStone != null) {
            // Create paint object for drawing
            Paint paint = new Paint();
            paint.setColor(android.graphics.Color.GREEN);
            paint.setStyle(Paint.Style.STROKE);
            paint.setStrokeWidth(4);

            // Draw text with distance information
            String distanceText = String.format("Distance: %.2f units", closestStone.dist);
            paint.setTextSize(36);
            canvas.drawText(distanceText, 50, 50, paint);

            // Draw color information
            String colorText = "Color: " + closestStone.color;
            canvas.drawText(colorText, 50, 100, paint);
        }
    }

    // [Include all other helper methods from the original class...]
    void findContours(Mat input) {
        // Convert the input image to YCrCb color space
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

        // Extract channels
        Core.extractChannel(ycrcbMat, cbMat, 2);
        Core.extractChannel(ycrcbMat, crMat, 1);

        // Threshold channels
        Imgproc.threshold(cbMat, blueThresholdMat, BLUE_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(crMat, redThresholdMat, RED_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(cbMat, yellowThresholdMat, YELLOW_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);

        // Apply morphology
        morphMask(blueThresholdMat, morphedBlueThreshold);
        morphMask(redThresholdMat, morphedRedThreshold);
        morphMask(yellowThresholdMat, morphedYellowThreshold);

        // Find contours
        ArrayList<MatOfPoint> blueContoursList = new ArrayList<>();
        ArrayList<MatOfPoint> redContoursList = new ArrayList<>();
        ArrayList<MatOfPoint> yellowContoursList = new ArrayList<>();

        Imgproc.findContours(morphedBlueThreshold, blueContoursList, new Mat(),
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.findContours(morphedRedThreshold, redContoursList, new Mat(),
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.findContours(morphedYellowThreshold, yellowContoursList, new Mat(),
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // Analyze contours
        for(MatOfPoint contour : blueContoursList) {
            analyzeContour(contour, input, "Blue");
        }
        for(MatOfPoint contour : redContoursList) {
            analyzeContour(contour, input, "Red");
        }
        for(MatOfPoint contour : yellowContoursList) {
            analyzeContour(contour, input, "Yellow");
        }
    }

    void morphMask(Mat input, Mat output) {
        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);
        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    public void cleanup() {
        ycrcbMat.release();
        crMat.release();
        cbMat.release();
        blueThresholdMat.release();
        redThresholdMat.release();
        yellowThresholdMat.release();
        morphedBlueThreshold.release();
        morphedRedThreshold.release();
        morphedYellowThreshold.release();
        contoursOnPlainImageMat.release();

        // Release Mats in stored AnalyzedStone objects
        for (AnalyzedStone stone : internalStoneList) {
            if (stone.rvec != null) stone.rvec.release();
            if (stone.tvec != null) stone.tvec.release();
        }
    }


    void analyzeContour(MatOfPoint contour, Mat input, String color) {
        Point[] points = contour.toArray();
        MatOfPoint2f contour2f = new MatOfPoint2f(points);

        RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
        if((rotatedRectFitToContour.size.width * rotatedRectFitToContour.size.height) > minArea &&
                Math.abs(findAspectRatio(rotatedRectFitToContour.size) - (8.9/3.8)) > aspectRatioThresh) {
            drawRotatedRect(rotatedRectFitToContour, input, color);
        }

        double rotRectAngle = rotatedRectFitToContour.angle;
        if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height) {
            rotRectAngle += 90;
        }

        double angle = -(rotRectAngle - 180);
        drawTagText(rotatedRectFitToContour, Integer.toString((int) Math.round(angle)) + " deg", input, color);

        MatOfPoint3f objectPoints = new MatOfPoint3f(
                new Point3(-objectWidth / 2, -objectHeight / 2, 0),
                new Point3(objectWidth / 2, -objectHeight / 2, 0),
                new Point3(objectWidth / 2, objectHeight / 2, 0),
                new Point3(-objectWidth / 2, objectHeight / 2, 0)
        );

        Point[] rectPoints = new Point[4];
        rotatedRectFitToContour.points(rectPoints);
        Point[] orderedRectPoints = orderPoints(rectPoints);
        MatOfPoint2f imagePoints = new MatOfPoint2f(orderedRectPoints);

        Mat rvec = new Mat();
        Mat tvec = new Mat();

        boolean success = Calib3d.solvePnP(
                objectPoints,
                imagePoints,
                cameraMatrix,
                distCoeffs,
                rvec,
                tvec
        );

        if (success) {
            drawAxis(input, rvec, tvec, cameraMatrix, distCoeffs);

            AnalyzedStone analyzedStone = new AnalyzedStone();
            analyzedStone.angle = rotRectAngle;
            analyzedStone.color = color;
            analyzedStone.rvec = rvec;
            analyzedStone.tvec = tvec;
            analyzedStone.dist = findDistance(analyzedStone);
            internalStoneList.add(analyzedStone);

            if (analyzedStone.dist < closestDistance) {
                closestDistance = analyzedStone.dist;
                closestStone = analyzedStone;
            }
        }
    }

    void drawAxis(Mat img, Mat rvec, Mat tvec, Mat cameraMatrix, MatOfDouble distCoeffs) {
        double axisLength = 5.0;
        MatOfPoint3f axisPoints = new MatOfPoint3f(
                new Point3(0, 0, 0),
                new Point3(axisLength, 0, 0),
                new Point3(0, axisLength, 0),
                new Point3(0, 0, -axisLength)
        );

        MatOfPoint2f imagePoints = new MatOfPoint2f();
        Calib3d.projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

        Point[] imgPts = imagePoints.toArray();
        Imgproc.line(img, imgPts[0], imgPts[1], new Scalar(0, 0, 255), 2);
        Imgproc.line(img, imgPts[0], imgPts[2], new Scalar(0, 255, 0), 2);
        Imgproc.line(img, imgPts[0], imgPts[3], new Scalar(255, 0, 0), 2);
    }

    static Point[] orderPoints(Point[] pts) {
        Point[] orderedPts = new Point[4];
        double[] sum = new double[4];
        double[] diff = new double[4];

        for (int i = 0; i < 4; i++) {
            sum[i] = pts[i].x + pts[i].y;
            diff[i] = pts[i].y - pts[i].x;
        }

        orderedPts[0] = pts[indexOfMin(sum)];    // Top-left
        orderedPts[2] = pts[indexOfMax(sum)];    // Bottom-right
        orderedPts[1] = pts[indexOfMin(diff)];   // Top-right
        orderedPts[3] = pts[indexOfMax(diff)];   // Bottom-left

        return orderedPts;
    }

    static int indexOfMin(double[] array) {
        int index = 0;
        double min = array[0];
        for (int i = 1; i < array.length; i++) {
            if (array[i] < min) {
                min = array[i];
                index = i;
            }
        }
        return index;
    }

    static int indexOfMax(double[] array) {
        int index = 0;
        double max = array[0];
        for (int i = 1; i < array.length; i++) {
            if (array[i] > max) {
                max = array[i];
                index = i;
            }
        }
        return index;
    }

    static void drawTagText(RotatedRect rect, String text, Mat mat, String color) {
        Scalar colorScalar = getColorScalar(color);
        Imgproc.putText(
                mat,
                text,
                new Point(rect.center.x - 50, rect.center.y + 25),
                Imgproc.FONT_HERSHEY_PLAIN,
                1,
                colorScalar,
                1);
    }

    static void drawRotatedRect(RotatedRect rect, Mat drawOn, String color) {
        Point[] points = new Point[4];
        rect.points(points);
        Scalar colorScalar = getColorScalar(color);
        for (int i = 0; i < 4; ++i) {
            Imgproc.line(drawOn, points[i], points[(i + 1) % 4], colorScalar, 2);
        }
    }

    static Scalar getColorScalar(String color) {
        switch (color) {
            case "Blue": return BLUE;
            case "Yellow": return YELLOW;
            default: return RED;
        }
    }

    double findAspectRatio(Size sample) {
        double height = sample.height;
        double width = sample.width;
        if(sample.height > sample.width) {
            height = sample.width;
            width = sample.height;
        }
        return (height / width);
    }

    double findDistance(AnalyzedStone stone) {
        return Math.sqrt(
                Math.pow(stone.tvec.get(0, 0)[0], 2) +
                        Math.pow(stone.tvec.get(1, 0)[0], 2) +
                        Math.pow(stone.tvec.get(2, 0)[0], 2)
        );
    }

    void getStoneCorners(AnalyzedStone stone, Point[] corners) {
        MatOfPoint3f objectPoints = new MatOfPoint3f(
                new Point3(-objectWidth / 2, -objectHeight / 2, 0),
                new Point3(objectWidth / 2, -objectHeight / 2, 0),
                new Point3(objectWidth / 2, objectHeight / 2, 0),
                new Point3(-objectWidth / 2, objectHeight / 2, 0)
        );

        MatOfPoint2f imagePoints = new MatOfPoint2f();
        Calib3d.projectPoints(objectPoints, stone.rvec, stone.tvec, cameraMatrix, distCoeffs, imagePoints);

        Point[] points = imagePoints.toArray();
        System.arraycopy(points, 0, corners, 0, 4);
    }

    void highlightClosestObject(Mat input) {
        for (AnalyzedStone stone : internalStoneList) {
            double distance = findDistance(stone);
            if (distance < closestDistance) {
                closestDistance = distance;
                closestStone = stone;
            }
        }

        if (closestStone != null) {
            Point[] rectPoints = new Point[4];
            getStoneCorners(closestStone, rectPoints);

            for (int i = 0; i < 4; ++i) {
                Imgproc.line(input,
                        rectPoints[i],
                        rectPoints[(i + 1) % 4],
                        HIGHLIGHT_COLOR,
                        HIGHLIGHT_THICKNESS);
            }

            String distanceText = String.format("%.2f units", closestDistance);
            Point textPoint = new Point(rectPoints[0].x, rectPoints[0].y - 10);
            Imgproc.putText(input,
                    distanceText,
                    textPoint,
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    HIGHLIGHT_COLOR,
                    2);
        }
    }



    public ArrayList<AnalyzedStone> getDetectedStones() {
        return clientStoneList;
    }
}