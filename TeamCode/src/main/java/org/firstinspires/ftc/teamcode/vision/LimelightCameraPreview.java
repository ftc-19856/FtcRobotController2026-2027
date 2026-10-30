package org.firstinspires.ftc.teamcode.vision;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.WebHandlerManager;
import fi.iki.elonen.NanoHTTPD;
import java.io.BufferedInputStream;
import java.io.FilterInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Collections;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.Semaphore;
import java.util.concurrent.atomic.AtomicBoolean;
import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.json.JSONException;
import org.json.JSONObject;

/** On-demand Driver Station preview and a read-only continuous browser stream. */
public final class LimelightCameraPreview implements CameraStreamSource, AutoCloseable {
    private static volatile LimelightCameraPreview active;
    private static final String PAGE = "<!doctype html><meta name='viewport' content='width=device-width'>"
            + "<title>Limelight camera</title><style>body{background:#161616;color:#eee;font:16px sans-serif}"
            + "img{max-width:100%;max-height:70vh}pre{white-space:pre-wrap}</style>"
            + "<h2>Limelight camera</h2><img src='/limelight/stream' alt='Initialize a vision OpMode to view the camera'>"
            + "<pre id='state'>Waiting for camera...</pre><script>"
            + "async function poll(){try{let r=await fetch('/limelight/state',{cache:'no-store'});"
            + "let s=await r.json();document.getElementById('state').textContent=JSON.stringify(s,null,2)}"
            + "catch(e){document.getElementById('state').textContent='Initialize a vision OpMode to view the camera'}"
            + "setTimeout(poll,250)}poll()</script>";

    private final Limelight3A camera;
    private final ExecutorService worker = Executors.newSingleThreadExecutor();
    private final AtomicBoolean pending = new AtomicBoolean();
    private final Semaphore streamSlots = new Semaphore(2);
    private final Set<HttpURLConnection> connections = Collections.newSetFromMap(new ConcurrentHashMap<>());
    private volatile boolean closed;
    private volatile String status = "Ready: DS menu > Camera Stream";
    private volatile String trackingStatus = "";

    private LimelightCameraPreview(Limelight3A camera) { this.camera = camera; }

    public static LimelightCameraPreview start(Limelight3A camera) {
        if (active != null) active.close();
        LimelightCameraPreview preview = new LimelightCameraPreview(camera);
        active = preview;
        CameraStreamServer.getInstance().setSource(preview);
        return preview;
    }

    public void setTrackingStatus(String text) { trackingStatus = text; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("Camera preview: INIT > 3-dot menu > Camera Stream (tap to refresh)");
        telemetry.addData("Live video", "http://192.168.43.1:8080/limelight");
        telemetry.addData("Preview status", status);
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        if (closed || !pending.compareAndSet(false, true)) return;
        try {
            worker.execute(() -> {
                try (InputStream input = new BufferedInputStream(openCameraStream())) {
                    byte[] jpeg = MjpegFrameReader.readFrame(input, 2 * 1024 * 1024);
                    Bitmap bitmap = BitmapFactory.decodeByteArray(jpeg, 0, jpeg.length);
                    if (bitmap == null) throw new IOException("Camera JPEG could not be decoded");
                    if (closed) {
                        bitmap.recycle();
                    } else {
                        status = "Frame received";
                        continuation.dispatch(consumer -> {
                            try { consumer.accept(bitmap); }
                            finally { bitmap.recycle(); }
                        });
                    }
                } catch (IOException | RuntimeException e) {
                    if (!closed) status = "Preview unavailable: " + e.getMessage();
                } finally {
                    pending.set(false);
                }
            });
        } catch (RejectedExecutionException e) {
            pending.set(false);
        }
    }

    private InputStream openCameraStream() throws IOException {
        if (closed) throw new IOException("Preview stopped");
        HttpURLConnection connection = (HttpURLConnection) new URL(
                "http://" + RobotConfig.LIMELIGHT_IP + ":5800/").openConnection();
        connection.setConnectTimeout(1000);
        connection.setReadTimeout(2000);
        connection.setUseCaches(false);
        if (!streamSlots.tryAcquire()) throw new IOException("Close another camera preview first");
        connections.add(connection);
        try {
            InputStream input = connection.getInputStream();
            if (closed) throw new IOException("Preview stopped");
            return new FilterInputStream(input) {
                private boolean released;
                @Override public void close() throws IOException {
                    if (released) return;
                    released = true;
                    try { super.close(); }
                    finally {
                        connections.remove(connection);
                        connection.disconnect();
                        streamSlots.release();
                    }
                }
            };
        } catch (IOException e) {
            connections.remove(connection);
            connection.disconnect();
            streamSlots.release();
            throw e;
        }
    }

    private String stateJson() {
        JSONObject json = new JSONObject();
        try {
            json.put("tracking", trackingStatus);
            json.put("polling", camera.isRunning());
            json.put("connected", camera.isConnected());
            LLResult result = camera.getLatestResult();
            if (result != null) {
                json.put("pipeline", result.getPipelineIndex());
                json.put("valid", result.isValid());
                json.put("areaPercent", result.getTa());
                json.put("rawTxDegrees", result.getTx());
                json.put("rawTyDegrees", result.getTy());
                json.put("frameAgeMs", result.getStaleness());
            }
        } catch (JSONException ignored) { }
        return json.toString();
    }

    @WebHandlerRegistrar
    public static void registerWebHandlers(Context context, WebHandlerManager manager) {
        manager.register("/limelight", session -> response("text/html", PAGE));
        manager.register("/limelight/state", session -> {
            LimelightCameraPreview preview = active;
            return response("application/json", preview == null
                    ? "{\"status\":\"Initialize a vision OpMode\"}" : preview.stateJson());
        });
        manager.register("/limelight/stream", session -> {
            LimelightCameraPreview preview = active;
            if (preview == null) return unavailable("Initialize a vision OpMode first");
            try {
                InputStream input = preview.openCameraStream();
                NanoHTTPD.Response response = NanoHTTPD.newChunkedResponse(NanoHTTPD.Response.Status.OK,
                        "multipart/x-mixed-replace; boundary=boundarydonotcross", input);
                response.addHeader("Cache-Control", "no-store");
                return response;
            } catch (IOException e) {
                preview.status = "Preview unavailable: " + e.getMessage();
                return unavailable(preview.status);
            }
        });
    }

    private static NanoHTTPD.Response response(String type, String body) {
        NanoHTTPD.Response response = NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, type, body);
        response.addHeader("Cache-Control", "no-store");
        return response;
    }

    private static NanoHTTPD.Response unavailable(String text) {
        return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.SERVICE_UNAVAILABLE, "text/plain", text);
    }

    @Override public void close() {
        closed = true;
        worker.shutdownNow();
        for (HttpURLConnection connection : connections) connection.disconnect();
        connections.clear();
        if (active == this) {
            active = null;
            CameraStreamServer.getInstance().setSource(null);
        }
    }
}
