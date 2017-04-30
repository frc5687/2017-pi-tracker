package org.frc5687.pitracker2017;


import com.sun.javafx.font.directwrite.RECT;
import com.sun.prism.paint.Stop;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.net.*;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;
import java.util.Timer;

import static org.opencv.imgproc.Imgproc.circle;
import static org.opencv.imgproc.Imgproc.rectangle;
import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.videoio.Videoio.CAP_PROP_EXPOSURE;
import static org.opencv.videoio.Videoio.CV_CAP_PROP_FRAME_HEIGHT;
import static org.opencv.videoio.Videoio.CV_CAP_PROP_FRAME_WIDTH;

public class Main {
    private static double CAMERA_HORIZONTAL_FOV = 62.2;
    private static double CAMERA_VERTICAL_FOV = 48.8;
    private static double CAMERA_ANGLE = 10.00;
    private static double CAMERA_HEIGHT = 2.5;

    private static double FOCAL = 543.25;

    private static double TARGET_BOTTOM_HEIGHT = 10.75;
    private static double TARGET_HEIGHT = 5;
    private static double TARGET_WIDTH = 20;

    public static final int piPort = 27002;
    public static final int rioPort = 27001;

    static int camPort = 0;
    static Double exposure = null;
    static String team = "5687";
    static String address = null;
    static boolean logging = false;
    static int images = 0;
    static String file = null;

    static int minArea = 20;
    static int maxRatio = 6;

    static boolean debug = false;

    public static void main(String[] args) {
        int rX = 0;
        int rY = 0;
        double fX = 0;
        double fY = 0;

        long startMills = System.currentTimeMillis();

        // Parse the parameters...
        parseParameters(args);

        // Confirm the parameters
        System.out.println(String.format("Camera port set to %1$d", camPort));
        if (team!=null) {
            System.out.println(String.format("Team set to %1$s", team));
        }
        System.out.println(String.format("Logging set to %1$b", logging));
        System.out.println(String.format("Images set to %1$b", images));
        System.out.println(String.format("Debug set to %1$b", debug));

        if (address==null && team!=null) {
            address =  String.format("roboRIO-%1$s-FRC.local", team);
        }
        System.out.println(String.format("Address set to %1$s", address));

        // Connect to the rio via the RobotProxy...
        RobotProxy robot = new RobotProxy(address, piPort, rioPort);

        // Tell the rio we're here!!!
        // This tells the rio where to send heartbeats
        robot.Send(0, false, 0, 0);

        // Initialize OpenCV
        System.out.println("Loading OpenCV...");

        // Load the native library.
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        System.out.println("Initializing camera...");
        VideoCapture camera = new VideoCapture(0);
        try {
            Thread.sleep(1000);
        } catch (Exception e) {
        }

        // Open the camera
        camera.open(camPort); //Useless

        if (!camera.isOpened()) {
            System.out.println("Camera Error!");
        } else {
            System.out.println("Camera connected!");
        }

        if (!camera.set(CV_CAP_PROP_FRAME_WIDTH, 320)){
            System.out.println("Unable to set frame width!");
        }
        if (!camera.set(CV_CAP_PROP_FRAME_HEIGHT, 240)){
            System.out.println("Unable to set frame height!");
        }

        // And set the exposure low (to improve contrast of retro-reflective tape)
        // Note that we don't normally use this because of our shades.  LEaving it here in case it's needed in the future.
        if (exposure!=null) {
            camera.set(CAP_PROP_EXPOSURE, exposure.doubleValue());
        }


        long folderNumber = 1;
        String prefix = "./";

        // Setup the image logging folder
        if (images>0) {
            while (true) {
                prefix = "/home/pi/images/" + folderNumber;
                // prefix = "C:\\Users\\Ben Bernard\\Documents\\FRC\\FRC 2017\\workspace\\images\\Good\\";
                File imageDir = new File(prefix);
                if (!imageDir.exists()) {
                    imageDir.mkdir();
                    if (imageDir.exists()) {
                        prefix += "/";
                    } else {
                        prefix = "./";
                    }
                    break;
                }
                folderNumber++;
            }
        }


        int retry = 240;





        int lowerH = 53;
        int lowerL = 41;
        int lowerS = 165;

        int upperH = 77;
        int upperL = 135;
        int upperS = 255;
/*
        External:
        int lowerH = 45;
        int lowerL = 34;
        int lowerS = 140;

        int upperH = 74;
        int upperL = 255;
        int upperS = 255;




        int lowerH = 57;
        int lowerL = 44;
        int lowerS = 105;

        int upperH = 100;
        int upperL = 174;
        int upperS = 255;
*/



        boolean first=true;
        Mat frame = new Mat();
        Mat hls = null;
        Mat filtered = null;
        Mat cont = null;
        MatOfInt minCompressionParam = new MatOfInt(Imgcodecs.CV_IMWRITE_PNG_COMPRESSION, 3);
        MatOfInt compressionParam = new MatOfInt(Imgcodecs.CV_IMWRITE_PNG_COMPRESSION, 6);
        MatOfInt maxCompressionParam = new MatOfInt(Imgcodecs.CV_IMWRITE_PNG_COMPRESSION, 8);
        ContourComparator comparator = new ContourComparator();

        // time counting
        int count;

        Stopwatch loopTimer = new Stopwatch("Loop");
        Stopwatch captureTimer = new Stopwatch("Capture Frame");
        Stopwatch conversionTimer = new Stopwatch("Conversion");
        Stopwatch filterTimer = new Stopwatch("HLS Filter");
        Stopwatch contourTimer = new Stopwatch("Contour");
        Stopwatch targetTimer = new Stopwatch("Target");
        Stopwatch imageLoggingTimer = new Stopwatch("Image Logging");

        count = 0;

        while (true) {
            boolean logImage = images>0 && (count % images==0);
            loopTimer.start();

            StringBuilder log = new StringBuilder();
            long loopMillis = System.currentTimeMillis();
            long mills = loopMillis - startMills;
            long rioMillis = 0;
            rioMillis = robot.getRobotTimestamp();
            if (rioMillis==0 && !debug) {
                // Keep trying to contact the robot...
                robot.Send(0, false, 0, 0);
            }

            if (debug || file!=null || robot.isRinglighton()) {
                captureTimer.start();

                // Capture a frame and write to disk
                if (file==null) {
                    camera.read(frame);
                } else {
                    frame = Imgcodecs.imread(file);
                }

                captureTimer.stop();


                if (rX != frame.width()) {
                    rX = frame.width();
                    fX = getFocalLength(rX, CAMERA_HORIZONTAL_FOV);
                    first = true;
                }
                if (rY != frame.height()) {
                    rY = frame.height();
                    fY = getFocalLength(rY, CAMERA_VERTICAL_FOV);
                    first = true;  // reset first because the frame size changed and we need to reallocate mats
                }

                if (logImage) {
                    imageLoggingTimer.start();
                    Imgcodecs.imwrite(prefix + "a_bgr_" + mills + ".png", frame, minCompressionParam);
                    imageLoggingTimer.stop();
                }

                if (first) {
                    hls = new Mat(frame.size(), frame.type());
                    filtered = new Mat(frame.size(), frame.type());
                    cont = new Mat(frame.size(), frame.type());
                    first = false;
                }


                conversionTimer.start();
                // Convert to HLS color model
                Imgproc.cvtColor(frame, hls, Imgproc.COLOR_BGR2HLS);
                //if (images) {
                //    Imgcodecs.imwrite(prefix + "b_hls_" + mills + ".png", hls, minCompressionParam);
                //}

                conversionTimer.stop();

                filterTimer.start();

                // Filter using HLS lower and upper range
                Scalar lower = new Scalar(lowerH, lowerL, lowerS, 0);
                Scalar upper = new Scalar(upperH, upperL, upperS, 0);

                Core.inRange(hls, lower, upper, filtered);
                filterTimer.stop();

                if (logImage) {
                    imageLoggingTimer.start();
                    Imgcodecs.imwrite(prefix + "c_flt_" + mills + ".png", filtered, minCompressionParam);
                    imageLoggingTimer.stop();
                }

                contourTimer.start();

                // Find the contours...
                List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
                Imgproc.findContours(filtered, contours, cont, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                contourTimer.stop();

                targetTimer.start();
                int cmax = contours.size();
                Rect rectA = null;
                Rect rectB = null;
                int sizeA = 0;
                int sizeB = 0;

                for(int i = 0; i < cmax; i++) {
                    MatOfPoint contour = contours.get(i);
                    Rect rect = Imgproc.boundingRect(contour);

                    int width = rect.width;
                    int height = rect.height;

                    if (height > width * maxRatio) {
                        continue; // Too tall!
                    }
                    if (width > height * maxRatio) {
                        continue; // Too short
                    }

                    int size = width * height;


                    if (size < minArea) {
                        continue; // Too small!
                    }

                    if (rectA == null) {
                        rectA = rect;
                        sizeA = size;
                    } else if (size > sizeA) {
                        rectB = rectA;
                        sizeB = sizeA;
                        rectA = rect;
                        sizeA = size;
                    } else if (rectB == null) {
                        rectB = rect;
                        sizeB = size;
                    } else if (size > sizeB) {
                        rectB = rect;
                        sizeB = size;
                    }
                }

                targetTimer.stop();

                if (rectB!=null) {
                    // And find the bounding rectangle for the two largest...
                    Rect rect = findBoundingRect(rectA, rectB);

                    // Now find its center point
                    final double vcx = rect.x + (rect.width/2);
                    final double vcy = rect.y + (rect.height/2);

                    final double cx = vcx - (rX / 2);
                    final double cy = -1 * (vcy - (rY / 2));

                    if (logImage) {
                        imageLoggingTimer.start();

                        // Copy the frame to the cont mat
                        frame.copyTo(cont);


                        // Draw the two contours:
                        //for (MatOfPoint m : contours) {
                        //    Rect brect = Imgproc.boundingRect(m);
                        //    rectangle(cont, brect.tl(), brect.br(), new Scalar(128, 128, 0), 2, 8, 0);
                        //}
                        rectangle(cont, rectA.tl(), rectA.br(), new Scalar(0, 255, 0), 2, 8, 0);
                        rectangle(cont, rectB.tl(), rectB.br(), new Scalar(0, 255, 0), 2, 8, 0);

                        // Draw the bounding rectangle
                        rectangle(cont, rect.tl(), rect.br(), new Scalar(255, 255, 255), 4, 8, 0);

                        // Draw the pin in the center
                        circle(cont, new Point(vcx, vcy), 2, new Scalar(200, 255, 200), 1, 8, 0);

                        // Write it to the card
                        Imgcodecs.imwrite(prefix + "d_con_" + mills + ".png", cont, minCompressionParam);
                        imageLoggingTimer.stop();

                    }


                    final int width = rect.width;
                    final int height = rect.height;

                    // Find the lateral offset angle
                    final double offsetAngle = getAngle(cx, fX);


                    // Now find the vertical angle
                    final double verticalAngle = getAngle(cy, fY);

                    // From that, determine the distance to the target
                    final double distance = getDistance(verticalAngle);


                    // Send the results to the rio
                    robot.Send(rioMillis, true, offsetAngle, distance);


                    if (logging) {
                        System.out.println(String.format("cx=%1$f, cy=%2$f, offsetAngle=%3$f, distance=%4$f", cx, cy, offsetAngle, distance));
                    }

                    if (logImage) {
                        logFrame(prefix, mills, width, height, cx, cy, offsetAngle, distance);
                    }

                } else {
                    robot.Send(rioMillis, false, 0, 0);
                    if (logging) {
                        System.out.println(String.format("Target absent."));
                    }

                    if (logImage) {
                        imageLoggingTimer.start();
                        logFrame(prefix, mills, log.toString());
                        imageLoggingTimer.stop();
                    }

                }
            } else {
                robot.Send(rioMillis, false, 0, 0);
                if (logging) {
                    System.out.println(String.format("Not tracking."));
                }
            }

            try {
                // Make sure that this has taken AT LEAST 20 milliseconds.
                // If not, sleep until 20ms have passed
                long w = System.currentTimeMillis() - loopMillis;
                if (w < 20) {
                    Thread.sleep(20 - w);
                }
            } catch (Exception e) {
            }
            // long runMillis = System.currentTimeMillis() - loopMillis;
//            if (file!=null) { return; }

            count ++;
            loopTimer.stop();
            if (logging && count % 100 == 0) {
                System.out.println("Cycles Per Second: " + count / (loopTimer.elapsed() / 1000));
                System.out.println("Mean Cycle Time: " + (loopTimer.elapsed() / count));
                System.out.println("Mean Capture Time: " + (captureTimer.elapsed() / count));
                System.out.println("Mean Conversion Time: " + (conversionTimer.elapsed() / count));
                System.out.println("Mean Filtering Time: " + (filterTimer.elapsed() / count));
                System.out.println("Mean Contour Detection Time: " + (contourTimer.elapsed() / count));
                System.out.println("Mean Contour Filtering Time: " + (targetTimer.elapsed() / count));
                System.out.println("File Logging: " + (imageLoggingTimer.elapsed() / count));
            }

        }



    }

    /**
     * Parses the parameters of the pi tracker
     * @param args The list of arguments to parse
     */
    private static void parseParameters(String[] args) {
        for (String arg : args) {
            String[] a = arg.toLowerCase().split("=");
            if (a.length == 2) {
                switch (a[0]) {
                    case "cam":
                    case "camera":
                    case "c":
                        try {
                            camPort = Integer.parseInt(a[1]);
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        break;
                    case "exposure":
                    case "exp":
                    case "e":
                        try {
                            exposure = Double.parseDouble(a[1]);
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        break;
                    case "team":
                        try {
                            team = a[1];
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        break;
                    case "logging":
                    case "logs":
                    case "log":
                    case "l":
                        try {
                            logging = a[1].equals("on") || a[1].equals("yes") || a[1].equals("true");
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        break;
                    case "images":
                    case "i":
                        try {
                            images = Integer.parseInt(a[1]);;
                        } catch (Exception e) {
                            images = 0;
                            e.printStackTrace();
                        }
                        break;
                    case "debug":
                    case "d":
                        try {
                            debug = a[1].equals("on") || a[1].equals("yes") || a[1].equals("true");
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        break;
                    case "address":
                    case "addr":
                    case "a":
                        try {
                            address = a[1];
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        break;
                    case "file":
                    case "f":
                        try {
                            file = a[1];
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        break;
                    case "minarea":
                    case "ma":
                        try {
                            minArea = Integer.parseInt(a[1]);;
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        break;
                    case "maxratio":
                    case "mr":
                        try {
                            maxRatio = Integer.parseInt(a[1]);;
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        break;
                }
            }
        }
    }


    /**
     * Finds the rectangle that bounds two rectangles
     * @param a One rectangle to find the bounds of
     * @param b The other rectangle to find the bounds of
     * @return The bounding rectangle
     */
    private static Rect findBoundingRect(Rect a, Rect b) {
        int top = Math.min(a.y, b.y);
        int bottom =  Math.max(a.y + a.height, b.y + b.height);
        int left = Math.min(a.x, b.x);
        int right = Math.max(a.x + a.width, b.x + b.width);

        return new Rect(left, top, right - left, bottom - top);
    }


    private static void logFrame(String prefix, long mills, double width, double height, double cx, double cy, double offsetAngle, double distance) {
        try {
            //create a temporary file
            File logFile = new File(prefix + "e_log_" + mills + ".txt");

            BufferedWriter writer = new BufferedWriter(new FileWriter(logFile));
            writer.write("");
            writer.write("Mills: " + mills);
            writer.newLine();
            writer.write("TargetSighted: true");
            writer.newLine();
            writer.write("TargetSighting: Sighted");
            writer.newLine();
            writer.write("width: " + width);
            writer.newLine();
            writer.write("height: " + height);
            writer.newLine();
            writer.write("centerX: " + cx);
            writer.newLine();
            writer.write("centerY: " + cy);
            writer.newLine();
            writer.write("offsetAngle: " + offsetAngle);
            writer.newLine();
            writer.write("distance: " + distance);
            writer.newLine();
            //Close writer
            writer.close();
        } catch (Exception e) {

        }
    }

    private static void logFrame(String prefix, long mills, String log) {
        try {
            File logFile = new File(prefix + "e_log_" + mills + ".txt");

            BufferedWriter writer = new BufferedWriter(new FileWriter(logFile));
            writer.write("");
            writer.write("Mills: " + mills);
            writer.newLine();
            writer.write("TargetSighted: false");
            writer.newLine();
            writer.write("TargetSighting: Absent");
            writer.newLine();
            writer.write("Log: " + log.toString());
            writer.newLine();
            //Close writer
            writer.close();
        } catch (Exception e) {
        }

    }

    /**
     * Returns the computed focal length
     * @param imageSize Image size on the requested axis, in pixels.
     * @param fov Field of view on the requested axis, in degrees.
     * @return Focal length along the requested axis, in pixels.
     */
    private static double getFocalLength(double imageSize, double fov) {
        double fovRads = fov * Math.PI / 180;
        return imageSize / (2 * Math.tan(fovRads/2));
    }

    private static double getAngle(double center, double focal) {
        return Math.atan(center/focal) * 180 / Math.PI;
    }

    private static double getDistance(double verticalAngle) {
        return (TARGET_BOTTOM_HEIGHT + (TARGET_HEIGHT/2) - CAMERA_HEIGHT) / Math.tan((verticalAngle + CAMERA_ANGLE) * Math.PI / 180) ;
    }

    private static void SortContours(List<MatOfPoint> contours) {
        if (contours.size() <=2) { return; }
        boolean swapped = true;
        while (swapped) {
            swapped = false;
            for (int i = 1; i < contours.size(); i++) {
                if (contourArea(contours.get(i)) > contourArea(contours.get(i - 1))) {
                    MatOfPoint swap = contours.get(i);
                    contours.set(i, contours.get(i - 1));
                    contours.set(i - 1, swap);
                    swapped = true;
                }
            }
        }
    }

    private static InetAddress getFirstNonLoopbackAddress(boolean preferIpv4, boolean preferIPv6) throws SocketException {
        Enumeration en = NetworkInterface.getNetworkInterfaces();
        while (en.hasMoreElements()) {
            NetworkInterface i = (NetworkInterface) en.nextElement();
            for (Enumeration en2 = i.getInetAddresses(); en2.hasMoreElements(); ) {
                InetAddress addr = (InetAddress) en2.nextElement();
                if (!addr.isLoopbackAddress()) {
                    if (addr instanceof Inet4Address) {
                        if (preferIPv6) {
                            continue;
                        }
                        return addr;
                    }
                    if (addr instanceof Inet6Address) {
                        if (preferIpv4) {
                            continue;
                        }
                        return addr;
                    }
                }
            }
        }
        return null;
    }
}

