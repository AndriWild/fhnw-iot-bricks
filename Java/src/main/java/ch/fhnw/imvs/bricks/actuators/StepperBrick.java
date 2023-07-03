// Copyright (c) 2022 FHNW, Switzerland. All rights reserved.
// Licensed under MIT License, see LICENSE for details.

package ch.fhnw.imvs.bricks.actuators;

import ch.fhnw.imvs.bricks.core.Proxy;

public class StepperBrick extends MotorBrick {

  private StepperBrick(Proxy proxy, String brickID){
   super(proxy, brickID);
  }

  @Override
  public void setPosition(int position) {
    if (targetPosition != position) {
      targetPosition = position;
      super.sync();
    }
  }

  public static StepperBrick connect(Proxy proxy, String brickID) {
    StepperBrick brick = new StepperBrick(proxy, brickID);
    brick.connect();
    return brick;
  }
}