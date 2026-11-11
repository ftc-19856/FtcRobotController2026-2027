package org.firstinspires.ftc.teamcode.opmode;

import android.content.Context;
import com.qualcomm.robotcore.util.WebHandlerManager;
import fi.iki.elonen.NanoHTTPD;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;

/** Stores scored trials on the Hub and serves a read-only export of all sessions. */
public final class ShotTuningLog {
    private static final Object FILE_LOCK = new Object();
    private static final String HEADER = "session,x_in,y_in,heading_deg,distance_in,turret_target_deg,"
            + "turret_actual_deg,speed_percent,target_ticks_s,actual_ticks_s,battery_v,hood,"
            + "pivot_forward_in,pivot_left_in,pollen,min_ticks_s,window_s,movement_in,"
            + "heading_change_deg,max_aim_error_deg,hits,misses,high,low,shot_sequence";
    private final File directory;
    private File session;

    ShotTuningLog(Context context) {
        directory = directory(context);
    }

    private static File directory(Context context) {
        return new File(context.getFilesDir(), "pollen-shot-logs");
    }

    void append(String row) throws IOException {
        synchronized (FILE_LOCK) {
            if (!directory.isDirectory() && !directory.mkdirs()) {
                throw new IOException("Could not create shot log folder");
            }
            if (session == null) session = File.createTempFile("shots-", ".csv", directory);
            try (FileWriter writer = new FileWriter(session, true)) {
                writer.write(session.getName() + "," + row + "\n");
            }
        }
    }

    static String quote(String field) {
        String text = field == null ? "" : field.replace('\r', ' ').replace('\n', ' ');
        return '"' + text.replace("\"", "\"\"") + '"';
    }

    @WebHandlerRegistrar
    public static void registerWebHandlers(Context context, WebHandlerManager manager) {
        manager.register("/shot-tuning.csv", request -> {
            synchronized (FILE_LOCK) {
                File[] logs = directory(context).listFiles((dir, name) ->
                        name.startsWith("shots-") && name.endsWith(".csv"));
                StringBuilder csv = new StringBuilder(HEADER).append('\n');
                if (logs != null) {
                    Arrays.sort(logs);
                    for (File log : logs) {
                        try (BufferedReader reader = new BufferedReader(new FileReader(log))) {
                            String row;
                            while ((row = reader.readLine()) != null) csv.append(row).append('\n');
                        }
                    }
                }
                NanoHTTPD.Response response = NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK,
                        "text/csv; charset=utf-8", csv.toString());
                response.addHeader("Cache-Control", "no-store");
                response.addHeader("Content-Disposition", "attachment; filename=pollen-shots.csv");
                return response;
            }
        });
    }
}
