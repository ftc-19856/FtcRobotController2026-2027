package org.firstinspires.ftc.teamcode.vision;

import java.io.ByteArrayOutputStream;
import java.io.EOFException;
import java.io.IOException;
import java.io.InputStream;

/** Reads one bounded JPEG from a multipart camera stream. */
final class MjpegFrameReader {
    private MjpegFrameReader() { }

    static byte[] readFrame(InputStream input, int maxBytes) throws IOException {
        int previous = -1;
        for (int skipped = 0; skipped < 65536; skipped++) {
            int next = input.read();
            if (next < 0) throw new EOFException("Camera stream ended before an image");
            if (previous == 0xff && next == 0xd8) {
                ByteArrayOutputStream jpeg = new ByteArrayOutputStream(65536);
                jpeg.write(0xff);
                jpeg.write(0xd8);
                previous = next;
                while (jpeg.size() < maxBytes) {
                    next = input.read();
                    if (next < 0) throw new EOFException("Incomplete camera image");
                    jpeg.write(next);
                    if (previous == 0xff && next == 0xd9) return jpeg.toByteArray();
                    previous = next;
                }
                throw new IOException("Camera image exceeds size limit");
            }
            previous = next;
        }
        throw new IOException("No JPEG found in camera stream");
    }
}
