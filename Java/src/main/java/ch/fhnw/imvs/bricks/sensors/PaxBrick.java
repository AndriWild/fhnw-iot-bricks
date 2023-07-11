// Copyright (c) 2020 FHNW, Switzerland. All rights reserved.
// Licensed under MIT License, see LICENSE for details.

package ch.fhnw.imvs.bricks.sensors;

import ch.fhnw.imvs.bricks.core.Brick;
import ch.fhnw.imvs.bricks.core.Proxy;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public final class PaxBrick extends Brick {
    private PaxBrick(Proxy proxy, String brickID) {
        super(proxy, brickID);
    }

    private volatile int currentDist;

    public int getDistance() { // cm
        return currentDist;
    }

    @Override
    protected byte[] getTargetPayload(boolean mock) {
        byte[] payload = null;
        if (mock) {
            ByteBuffer buf = ByteBuffer.allocate(4);
            buf.order(ByteOrder.BIG_ENDIAN); // network byte order
            double mockBatt = Math.random() * 3.7;
            short mockDist = (short) (Math.random() * 350);
            buf.putShort((short) (mockBatt * 100));
            buf.putShort(mockDist);
            payload = buf.array();
        }
        return payload;
    }

    @Override
    protected void setCurrentPayload(byte[] payload) {
        ByteBuffer buf = ByteBuffer.wrap(payload);
        buf.order(ByteOrder.BIG_ENDIAN); // network byte order
        super.setBatteryVoltage(buf.getShort() / 100.0f);
        currentDist = buf.getShort();
    }

    public static PaxBrick connect(Proxy proxy, String brickID) {
        PaxBrick brick = new PaxBrick(proxy, brickID);
        brick.connect();
        return brick;
    }
}
