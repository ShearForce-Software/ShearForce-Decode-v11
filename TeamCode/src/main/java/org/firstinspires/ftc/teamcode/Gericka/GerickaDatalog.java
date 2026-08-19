package org.firstinspires.ftc.teamcode.Gericka;

import android.icu.text.SimpleDateFormat;

import java.io.FileWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Field;
import java.util.Date;
import java.util.Locale;

public class GerickaDatalog {
    private FileWriter writer;
    private StringBuffer lineBuffer;
    private long timeBase;

    public Field resetPositionCount = new Field();
    public Field imuHeading = new Field();
    public Field roadrunnerX = new Field();
    public Field roadrunnerY = new Field();
    public Field roadrunnerHeading = new Field();
    public Field distanceToTarget = new Field();
    public Field LaunchRampPosition = new Field();
    public Field shooterRPM = new Field();
    public Field shooterTargetRPM = new Field();
    public Field YawScalar = new Field();
    public Field turretTicks = new Field();
    public Field turrentAngle = new Field();

    public GerickaDatalog(String filenamePrefix) {
        File dir = new File("/sdcard/FIRST/Datalogs");
        if (!dir.exists()) dir.mkdirs();

        String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault()).format(new Date());
        File file = new File(dir, filenamePrefix + timestamp + ".csv");
        this.lineBuffer = new StringBuffer(256);
        this.timeBase = System.currentTimeMillis();

        try{
            this.writer = new FileWriter(file, false);
            writeHeader();
        } catch (IOException e) {
            throw new RuntimeException("Gericka Datalogger stream failed to open", e);
        }

        long maxAgeMs = 7200000;
        long currentTime = System.currentTimeMillis();

        if (file.exists() && file.isDirectory()) {
            File[] files = file.listFiles();
            if (files != null) {
                for (File file : files) {
                    if (file.isFile()) {
                        long fileAgeMs = currentTime - maxAgeMs;

                        if (fileAgeMs > maxAgeMs) {
                            file.delete();
                        }

                    }

                }

            }

        }

        // initialize all of the fields
        resetPositionCount.set(0);
        roadrunnerX.set(0.0);
        roadrunnerY.set(0.0);
        roadrunnerHeading.set(0.0);
        distanceToTarget.set(0.0);
        LaunchRampPosition.set(0.0);
        shooterRPM.set(0.0);
        shooterTargetRPM.set(0.0);
        YawScalar.set(0.0);
        turretTicks.set(0.0);
        turrentAngle.set(0.0);
        imuHeading.set(0.0);
    }
    private void writeHeader() throws IOException{
        writer.append("time(sec), resetCount, imuHeading(deg)" +
                ", roadrunner-x, roadrunner-y, roadrunner-heading(deg)" +
                ", distanceToTarget(in), LaunchRampPosition, shooterRPM, shooterTargetRPM" +
                ", YawScalar" +
                ", turretTicks, turrentAngle(deg) \n");
        writer.flush();
    }

    public void writeLine() {
        try{
            lineBuffer.setLength(0);
            lineBuffer.append((System.currentTimeMillis() - timeBase)/1000.0).append(",");
            lineBuffer.append(resetPositionCount.val).append(",");
            lineBuffer.append(imuHeading.val).append(",");
            lineBuffer.append(roadrunnerX.val).append(",");
            lineBuffer.append(roadrunnerY.val).append(",");
            lineBuffer.append(roadrunnerHeading.val).append(",");
            lineBuffer.append(distanceToTarget.val).append(",");
            lineBuffer.append(LaunchRampPosition.val).append(",");
            lineBuffer.append(shooterRPM.val).append(",");
            lineBuffer.append(shooterTargetRPM.val).append(",");
            lineBuffer.append(YawScalar.val).append(",");
            lineBuffer.append(turretTicks.val).append(",");
            lineBuffer.append(turrentAngle.val).append("\n");

            writer.append(lineBuffer.toString());
        } catch (IOException ignored) {}
    }

    public void close() {
        try{
            if (writer != null) {
              writer.flush();
              writer.close();
            }
        } catch (IOException ignored) {}
    }

    public static class Field {
        public String val = "0";
        public void set(Object obj){
            this.val = String.valueOf(obj);
        }
    }
}
