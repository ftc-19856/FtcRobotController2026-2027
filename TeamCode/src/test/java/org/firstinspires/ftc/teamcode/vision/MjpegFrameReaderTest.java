package org.firstinspires.ftc.teamcode.vision;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.EOFException;
import java.io.IOException;
import org.junit.Test;
import static org.junit.Assert.*;

public class MjpegFrameReaderTest {
    @Test public void extractsOnlyTheFirstJpegAfterMultipartHeaders() throws Exception {
        byte[] jpeg = {(byte) 0xff, (byte) 0xd8, 1, 2, (byte) 0xff, 0, 3, (byte) 0xff, (byte) 0xd9};
        ByteArrayOutputStream stream = new ByteArrayOutputStream();
        stream.write("--boundary\r\nContent-Type: image/jpeg\r\n\r\n".getBytes("US-ASCII"));
        stream.write(jpeg);
        stream.write("\r\n--boundary\r\n".getBytes("US-ASCII"));
        stream.write(jpeg);
        assertArrayEquals(jpeg, MjpegFrameReader.readFrame(new ByteArrayInputStream(stream.toByteArray()), 100));
    }

    @Test(expected = EOFException.class) public void rejectsIncompleteFrame() throws Exception {
        MjpegFrameReader.readFrame(new ByteArrayInputStream(new byte[] {(byte) 0xff, (byte) 0xd8, 1}), 100);
    }

    @Test(expected = IOException.class) public void boundsFrameMemory() throws Exception {
        MjpegFrameReader.readFrame(new ByteArrayInputStream(new byte[] {
                (byte) 0xff, (byte) 0xd8, 1, 2, 3, 4, (byte) 0xff, (byte) 0xd9}), 5);
    }

    @Test(expected = IOException.class) public void boundsSearchForMissingJpeg() throws Exception {
        MjpegFrameReader.readFrame(new ByteArrayInputStream(new byte[70000]), 100);
    }
}
