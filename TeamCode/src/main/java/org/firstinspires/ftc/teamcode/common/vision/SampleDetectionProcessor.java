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
    private final Mat ycrcbMat = new Mat();
    private final Mat crMat = new Mat();
    private final Mat cbMat = new Mat();

    private final Mat blueThresholdMat = new Mat();
    private final Mat redThresholdMat = new Mat();
    private final Mat yellowThresholdMat = new Mat();

    private final Mat morphedBlueThreshold = new Mat();
    private final Mat morphedRedThreshold = new Mat();
    private final Mat morphedYellowThreshold = new Mat();

    private final Mat contoursOnPlainImageMat = new Mat();

    private final Telemetry telemetry;
    private double output = 0.0;

    public double minArea = 40;
    public double aspectRatioThresh = 0.5;
    public double objectWidth = 3.5;  // cm
    public double objectHeight = 1.5;  // cm

    public static int YELLOW_MASK_THRESHOLD = 57;
    private static final int BLUE_MASK_THRESHOLD = 150;
    private static final int RED_MASK_THRESHOLD = 198;

    private final Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));
    private final Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));

    private static final Scalar RED = new Scalar(255, 0, 0);
    private static final Scalar BLUE = new Scalar(0, 0, 255);
    private static final Scalar YELLOW = new Scalar(255, 255, 0);
    private static final Scalar HIGHLIGHT_COLOR = new Scalar(0, 255, 0);

    private static final int CONTOUR_LINE_THICKNESS = 2;
    private static final int HIGHLIGHT_THICKNESS = 3;

    private AnalyzedStone closestStone = null;
    private double closestDistance = Double.MAX_VALUE;

    private final ArrayList<AnalyzedStone> internalStoneList = new ArrayList<>();
    private volatile ArrayList<AnalyzedStone> clientStoneList = new ArrayList<>();

    private final Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
    private MatOfDouble distCoeffs = new MatOfDouble();

    private static class AnalyzedStone {
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
        if (!cameraMatrix.empty()) {
            cameraMatrix.release();
        }

        double fx = 800;
        double fy = 800;
        double cx = width/2.0;
        double cy = height/2.0;

        cameraMatrix.put(0, 0,
                fx, 0, cx,
                0, fy, cy,
                0, 0, 1);

        distCoeffs = new MatOfDouble(0, 0, 0, 0, 0);
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        if (input == null || input.empty()) {
            return null;
        }

        internalStoneList.clear();
        findContours(input);

        if (closestStone != null) {
            telemetry.addLine("Closest Dist: " + closestStone.dist);
            drawAxis(input, closestStone.rvec, closestStone.tvec, cameraMatrix, distCoeffs);
            closestDistance = Double.MAX_VALUE;
        }

        clientStoneList = new ArrayList<>(internalStoneList);
        telemetry.update();

        return input;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        Paint rectPaint = new Paint();
        Paint textPaint = new Paint();
        Paint highlightPaint = new Paint();

        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 2);

        textPaint.setTextSize(scaleCanvasDensity * 20);
        textPaint.setColor(Color.WHITE);

        highlightPaint.setStyle(Paint.Style.STROKE);
        highlightPaint.setStrokeWidth(scaleCanvasDensity * 3);
        highlightPaint.setColor(Color.GREEN);

        for(AnalyzedStone stone : clientStoneList) {
            Point[] corners = new Point[4];
            getStoneCorners(stone, corners);

            switch(stone.color) {
                case "Blue":
                    rectPaint.setColor(Color.BLUE);
                    break;
                case "Yellow":
                    rectPaint.setColor(Color.YELLOW);
                    break;
                default:
                    rectPaint.setColor(Color.RED);
                    break;
            }

            for(int i = 0; i < 4; i++) {
                float startX = (float)corners[i].x * scaleBmpPxToCanvasPx;
                float startY = (float)corners[i].y * scaleBmpPxToCanvasPx;
                float endX = (float)corners[(i+1)%4].x * scaleBmpPxToCanvasPx;
                float endY = (float)corners[(i+1)%4].y * scaleBmpPxToCanvasPx;

                Paint paint = (stone == closestStone) ? highlightPaint : rectPaint;
                canvas.drawLine(startX, startY, endX, endY, paint);
            }

            String info = String.format("%.1f°, %.1fcm", stone.angle, stone.dist);
            canvas.drawText(info,
                    (float)corners[0].x * scaleBmpPxToCanvasPx,
                    (float)corners[0].y * scaleBmpPxToCanvasPx - 10,
                    textPaint);
        }

        if(closestStone != null) {
            drawAxis(canvas, closestStone.rvec, closestStone.tvec,
                    scaleBmpPxToCanvasPx, scaleCanvasDensity);
        }
    }

    private void findContours(Mat input) {
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(ycrcbMat, cbMat, 2);
        Core.extractChannel(ycrcbMat, crMat, 1);

        Imgproc.threshold(cbMat, blueThresholdMat, BLUE_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(crMat, redThresholdMat, RED_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(cbMat, yellowThresholdMat, YELLOW_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);

        morphMask(blueThresholdMat, morphedBlueThreshold);
        morphMask(redThresholdMat, morphedRedThreshold);
        morphMask(yellowThresholdMat, morphedYellowThreshold);

        ArrayList<MatOfPoint> blueContours = new ArrayList<>();
        ArrayList<MatOfPoint> redContours = new ArrayList<>();
        ArrayList<MatOfPoint> yellowContours = new ArrayList<>();

        Imgproc.findContours(morphedBlueThreshold, blueContours, new Mat(),
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.findContours(morphedRedThreshold, redContours, new Mat(),
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.findContours(morphedYellowThreshold, yellowContours, new Mat(),
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        for(MatOfPoint contour : blueContours) {
            analyzeContour(contour, input, "Blue");
            contour.release();
        }
        for(MatOfPoint contour : redContours) {
            analyzeContour(contour, input, "Red");
            contour.release();
        }
        for(MatOfPoint contour : yellowContours) {
            analyzeContour(contour, input, "Yellow");
            contour.release();
        }
    }

    private void morphMask(Mat input, Mat output) {
        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);
        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    private void analyzeContour(MatOfPoint contour, Mat input, String color) {
        Point[] points = contour.toArray();
        MatOfPoint2f contour2f = new MatOfPoint2f(points);

        RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
        if((rotatedRect.size.width * rotatedRect.size.height) > minArea &&
                Math.abs(findAspectRatio(rotatedRect.size) - (8.9/3.8)) > aspectRatioThresh) {
            drawRotatedRect(rotatedRect, input, color);
        }

        double rotRectAngle = rotatedRect.angle;
        if (rotatedRect.size.width < rotatedRect.size.height) {
            rotRectAngle += 90;
        }

        double angle = -(rotRectAngle - 180);
        drawTagText(rotatedRect, String.format("%.0f°", angle), input, color);

        MatOfPoint3f objectPoints = new MatOfPoint3f(
                new Point3(-objectWidth / 2, -objectHeight / 2, 0),
                new Point3(objectWidth / 2, -objectHeight / 2, 0),
                new Point3(objectWidth / 2, objectHeight / 2, 0),
                new Point3(-objectWidth / 2, objectHeight / 2, 0)
        );

        Point[] rectPoints = new Point[4];
        rotatedRect.points(rectPoints);
        Point[] orderedPoints = orderPoints(rectPoints);
        MatOfPoint2f imagePoints = new MatOfPoint2f(orderedPoints);

        Mat rvec = new Mat();
        Mat tvec = new Mat();

        if (Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec)) {
            drawAxis(input, rvec, tvec, cameraMatrix, distCoeffs);

            AnalyzedStone stone = new AnalyzedStone();
            stone.angle = rotRectAngle;
            stone.color = color;
            stone.rvec = rvec;
            stone.tvec = tvec;
            stone.dist = findDistance(stone);
            internalStoneList.add(stone);

            if (stone.dist < closestDistance) {
                closestDistance = stone.dist;
                closestStone = stone;
            }
        }

        contour2f.release();
        objectPoints.release();
        imagePoints.release();
    }

    private void getStoneCorners(AnalyzedStone stone, Point[] corners) {
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

        objectPoints.release();
        imagePoints.release();
    }

    private void drawAxis(Canvas canvas, Mat rvec, Mat tvec,
                          float scaleBmpPxToCanvasPx, float scaleCanvasDensity) {
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

        Paint axisPaint = new Paint();
        axisPaint.setStrokeWidth(scaleCanvasDensity * 2);

        // X axis (red)
        axisPaint.setColor(Color.RED);
        canvas.drawLine(
                (float)imgPts[0].x * scaleBmpPxToCanvasPx,
                (float)imgPts[0].y * scaleBmpPxToCanvasPx,
                (float)imgPts[1].x * scaleBmpPxToCanvasPx,
                (float)imgPts[1].y * scaleBmpPxToCanvasPx,
                axisPaint
        );

        // Y axis (green)
        axisPaint.setColor(Color.GREEN);
        canvas.drawLine(
                (float)imgPts[0].x * scaleBmpPxToCanvasPx,
                (float)imgPts[0].y * scaleBmpPxToCanvasPx,
                (float)imgPts[2].x * scaleBmpPxToCanvasPx,
                (float)imgPts[0].x * scaleBmpPxToCanvasPx,
                axisPaint
        );

        // Z axis (blue)
        axisPaint.setColor(Color.BLUE);
        canvas.drawLine(
                (float)imgPts[0].x * scaleBmpPxToCanvasPx,
                (float)imgPts[0].y * scaleBmpPxToCanvasPx,
                (float)imgPts[3].x * scaleBmpPxToCanvasPx,
                (float)imgPts[3].y * scaleBmpPxToCanvasPx,
                axisPaint
        );

        axisPoints.release();
        imagePoints.release();
    }

    private void drawAxis(Mat img, Mat rvec, Mat tvec, Mat cameraMatrix, MatOfDouble distCoeffs) {
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

        axisPoints.release();
        imagePoints.release();
    }

    private static Point[] orderPoints(Point[] pts) {
        if (pts == null || pts.length != 4) {
            return new Point[4];
        }

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

    private static int indexOfMin(double[] array) {
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

    private static int indexOfMax(double[] array) {
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

    private static void drawTagText(RotatedRect rect, String text, Mat mat, String color) {
        Scalar colorScalar = getColorScalar(color);

        Imgproc.putText(
                mat,
                text,
                new Point(rect.center.x - 50, rect.center.y + 25),
                Imgproc.FONT_HERSHEY_PLAIN,
                1,
                colorScalar,
                1
        );
    }

    private static void drawRotatedRect(RotatedRect rect, Mat drawOn, String color) {
        Point[] points = new Point[4];
        rect.points(points);
        Scalar colorScalar = getColorScalar(color);

        for (int i = 0; i < 4; i++) {
            Imgproc.line(drawOn, points[i], points[(i + 1) % 4], colorScalar, 2);
        }
    }

    private static Scalar getColorScalar(String color) {
        switch (color) {
            case "Blue":
                return BLUE;
            case "Yellow":
                return YELLOW;
            default:
                return RED;
        }
    }

    private double findAspectRatio(Size sample) {
        double height = sample.height;
        double width = sample.width;

        if (height > width) {
            height = width;
            width = sample.height;
        }

        return height / width;
    }

    private double findDistance(AnalyzedStone stone) {
        return Math.sqrt(
                Math.pow(stone.tvec.get(0, 0)[0], 2) +
                        Math.pow(stone.tvec.get(1, 0)[0], 2) +
                        Math.pow(stone.tvec.get(2, 0)[0], 2)
        );
    }

    public AnalyzedStone getClosestStone() {
        return closestStone;
    }

    public ArrayList<AnalyzedStone> getDetectedStones() {
        return clientStoneList;
    }

    public double getOutput() {
        return output;
    }
}