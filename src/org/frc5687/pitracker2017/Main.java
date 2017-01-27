package org.frc5687.pitracker2017;


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


public class Main {
    private static double CENTER_X = 319.5;
    private static double CENTER_Y = 239.5;
    private static double FOCAL = 543.25;

    private static double CAMERA_HEIGHT = 15;
    private static double TARGET_BOTTOM_HEIGHT = 83.5;
    private static double TARGET_HEIGHT = 14;
    private static double TARGET_WIDTH = 20;
    private static double CAMERA_ANGLE = 45.00;

    public static final int piPort = 27002;
    public static final int rioPort = 27001;

    public static void main(String[] args) {
        int camPort = 0;
        double exposure = .25;
        String team = null;
        String address = null;
        boolean logging = false;
        boolean images = false;
        int rX = 0;
        int rY = 0;

        long startMills = Instant.now().toEpochMilli();

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
                            images = a[1].equals("on") || a[1].equals("yes") || a[1].equals("true");
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
                }
            }
        }

        System.out.println(String.format("Camera port set to %1$d", camPort));
        if (team!=null) {
            System.out.println(String.format("Team set to %1$s", team));
        }
        System.out.println(String.format("Logging set to %1$b", logging));
        System.out.println(String.format("Images set to %1$b", images));

        if (address==null && team!=null) {
            address = String.format("roboRIO-%1$s-FRC.local", team);
        }
        System.out.println(String.format("Address set to %1$s", address));

        // Initialize the NetworkTable library with team information

        RobotProxy robot = new RobotProxy(address, piPort, rioPort);

        robot.Send(0, false, 0, 0);

        // Initialize OpenCV
        System.out.println("Loading OpenCV...");
        // Load the native library.
        System.loadLibrary("opencv_java310");

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

        // And set the exposure low (to improve contrast of retro-reflective tape)
        // camera.set(15, exposure);


        long folderNumber = 1;

        String prefix = "./";
        if (images) {
            while (true) {
                prefix = "/home/pi/images/" + folderNumber;
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
        int lowerL = 5;
        int lowerS = 190;

        int upperH = 74;
        int upperL = 135;
        int upperS = 255;
/*
        int lowerH = 57;
        int lowerL = 44;
        int lowerS = 105;

        int upperH = 100;
        int upperL = 174;
        int upperS = 255;
*/

        int minArea = 20;


        boolean first=true;
        Mat frame = new Mat();
        Mat hls = null;
        Mat filtered = null;
        Mat cont = null;
        MatOfInt minCompressionParam = new MatOfInt(Imgcodecs.CV_IMWRITE_PNG_COMPRESSION, 3);
        MatOfInt compressionParam = new MatOfInt(Imgcodecs.CV_IMWRITE_PNG_COMPRESSION, 6);
        MatOfInt maxCompressionParam = new MatOfInt(Imgcodecs.CV_IMWRITE_PNG_COMPRESSION, 8);

        while (true) {
            StringBuilder log = new StringBuilder();
            long mills = Instant.now().toEpochMilli() - startMills;
            long rioMillis = 0;
            rioMillis = robot.getRobotTimestamp();

            images = robot.isRinglighton();

            // Capture a frame and write to disk
            camera.read(frame);
            rX = frame.width();
            rY = frame.height();

            if (images) {
                Imgcodecs.imwrite(prefix + "a_bgr_" + mills + ".png", frame, minCompressionParam);
            }

            if (first) {
                hls = new Mat(frame.size(), frame.type());
                filtered = new Mat(frame.size(), frame.type());
                cont = new Mat(frame.size(), frame.type());
                first = false;
            }

            // Convert to HLS color model
            Imgproc.cvtColor(frame, hls, Imgproc.COLOR_BGR2HLS);
            //if (images) {
            //    Imgcodecs.imwrite(prefix + "b_hls_" + mills + ".png", hls, minCompressionParam);
            //}

            // Filter using HLS lower and upper range
            Scalar lower = new Scalar(lowerH, lowerL, lowerS, 0);
            Scalar upper = new Scalar(upperH, upperL, upperS, 0);

            Core.inRange(hls, lower, upper, filtered);
            if (images) {
                Imgcodecs.imwrite(prefix + "c_flt_" + mills + ".png", filtered, minCompressionParam);
            }

            // Find the contours...
            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Imgproc.findContours(filtered, contours, cont, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Now find the biggest contour (if any)
            double maxArea = 0;
            MatOfPoint biggest = null;
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea && area > minArea) {
                    maxArea = area;
                    biggest = contour;
                }
            }

            if (biggest != null) {
                // If we have one, find the bounding rectangle
                final Rect rect = Imgproc.boundingRect(biggest);

                // And its center point
                final double cx = rect.x + (rect.width / 2) - (rX / 2);
                final double cy = -1 * (rect.y + (rect.height / 2) - (rY / 2));


                final int width = rect.width;
                final int height = rect.height;

                // Find the lateral offset angle
                final double offsetAngle = getAngle(cx);


                // Now find the vertical angle
                final double verticalAngle = getAngle(cy);

                // From that, determine the distance to the target
                final double distance = getDistance(verticalAngle);



                robot.Send(rioMillis, true, offsetAngle, distance);


                if (logging) {
                    System.out.println(String.format("cx=%1$f, cy=%2$f, offsetAngle=%3$f, distance=%4$f", cx, cy, offsetAngle, distance));
                }

                if (images) {
                    try {
                        //create a temporary file
                        File logFile=new File(prefix + "d_log_" + mills + ".txt");

                        BufferedWriter writer = new BufferedWriter(new FileWriter(logFile));
                        writer.write("");
                        writer.write("Mills: " + mills); writer.newLine();
                        writer.write("TargetSighted: true"); writer.newLine();
                        writer.write("TargetSighting: Sighted"); writer.newLine();
                        writer.write("width: " + width); writer.newLine();
                        writer.write("height: " + rect.height); writer.newLine();
                        writer.write("centerX: " + cx); writer.newLine();
                        writer.write("centerY: " + cy); writer.newLine();
                        writer.write("offsetAngle: " + offsetAngle); writer.newLine();
                        writer.write("distance: " + distance); writer.newLine();
                        //Close writer
                        writer.close();
                    } catch(Exception e) {

                    }
                }

            } else {
                robot.Send(rioMillis, false, 0, 0);
                if (logging) {
                    System.out.println(String.format("Target absent."));
                }

                if (images) {
                    try {
                        //create a temporary file
                        File logFile=new File(prefix + "d_log_" + mills + ".txt");

                        BufferedWriter writer = new BufferedWriter(new FileWriter(logFile));
                        writer.write("");
                        writer.write("Mills: " + mills); writer.newLine();
                        writer.write("TargetSighted: false"); writer.newLine();
                        writer.write("TargetSighting: Absent"); writer.newLine();
                        writer.write("Log: " + log.toString()); writer.newLine();
                        //Close writer
                        writer.close();
                    } catch(Exception e) {

                    }
                }

            }


            try {
                // Make sure that this has taken AT LEAST 20 milliseconds.
                // If not, sleep until 20ms have passed
                long w = (Instant.now().toEpochMilli() - mills);
                if (w < 20) {
                    Thread.sleep(20 - w);
                }
            } catch (Exception e) {
            }
        }

    }


    private static double getAngle(double centerX) {
        return Math.atan(centerX/FOCAL) * 180 / Math.PI;
    }

    private static double getDistance(double verticalAngle) {
        return (TARGET_BOTTOM_HEIGHT + (TARGET_HEIGHT/2) - CAMERA_HEIGHT) / Math.tan((verticalAngle + CAMERA_ANGLE) * Math.PI / 180) ;
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

