package org.firstinspires.ftc.teamcode.Gericka;

import java.io.FileWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Field;

public class GerickaDatalog {
    private FileWriter writer;
    private StringBuffer lineBuffer;
    private long timeBase;

    public Field imuHeading = new Field();

    public GerickaDatalog(String filenamePrefix) {
        File dir = new File("/sdcard/FIRST/java/src/Datalogs");
        if (!dir.exists()) dir.mkdirs();

        File file = new File(dir, filenamePrefix + ".csv");
        this.lineBuffer = new StringBuffer(256);
        this.timeBase = System.currentTimeMillis();

        try{
            this.writer = new FileWriter(file, false);
            writeHeader();
        } catch (IOException e) {
            throw new RuntimeException("Gericka Datalogger stream failed to open", e);
        }
    }
    private void writeHeader() throws IOException{
        writer.append("time, imuHeading");
        writer.flush();
    }

    public void writeLine() {
        try{
            lineBuffer.setLength(0);
            lineBuffer.append(System.currentTimeMillis() - timeBase).append(",");
            lineBuffer.append(imuHeading.val).append(",");
            //lineBuffer.append(imuHeading.val).append(",");

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
