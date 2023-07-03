// Copyright (c) 2022 FHNW, Switzerland. All rights reserved.
// Licensed under MIT License, see LICENSE for details.

package ch.fhnw.imvs.bricks.actuators;

import ch.fhnw.imvs.bricks.core.Brick;
import ch.fhnw.imvs.bricks.core.Proxy;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public abstract class MotorBrick extends Brick {

  protected volatile int currentPosition = 0;
  protected volatile int targetPosition = 0;

  protected MotorBrick(Proxy proxy, String brickID) {
    super(proxy, brickID);
  }

  public int getPosition() {
    return currentPosition;
  }

  public abstract void setPosition(int position);

    @Override
  protected byte[] getTargetPayload(boolean mock) {
    ByteBuffer buf = ByteBuffer.allocate(mock ? 4 : 2);
    buf.order(ByteOrder.BIG_ENDIAN); // network byte order
    if (mock) {
      double mockBatt = Math.random() * 3.7;
      buf.putShort((short) (mockBatt * 100));
    }
    buf.putShort((short) targetPosition);
    return buf.array();
  }

  @Override
  protected void setCurrentPayload(byte[] payload) {
    ByteBuffer buf = ByteBuffer.wrap(payload);
    buf.order(ByteOrder.BIG_ENDIAN); // network byte order
    super.setBatteryVoltage(buf.getShort() / 100.0);
    currentPosition = buf.getShort();
  }
}