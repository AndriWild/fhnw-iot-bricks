// Copyright (c) 2022 FHNW, Switzerland. All rights reserved.
// Licensed under MIT License, see LICENSE for details.

package ch.fhnw.imvs.bricks.actuators;

import ch.fhnw.imvs.bricks.core.Proxy;

public final class ServoBrick extends MotorBrick {
    private ServoBrick(Proxy proxy, String brickID) {
        super(proxy, brickID);
    }


    public void setPosition(int position) { // degree
        if (position < 0 || position > 180) {
            throw new IllegalArgumentException();
        }
        if (targetPosition != position) {
            targetPosition = position;
            super.sync();
        }
    }

    public static ServoBrick connect(Proxy proxy, String brickID) {
        ServoBrick brick = new ServoBrick(proxy, brickID);
        brick.connect();
        return brick;
    }
}
