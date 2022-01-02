# FHNW IoT Bricks
> Work in progress. Interested? Contact thomas.amberg@fhnw.ch
## Building blocks for distributed IoT use cases
IoT Bricks enable IoT prototyping in a room, building or city.
## Simple, self-contained, connected
IoT Bricks come with connectivity and a simple Java SDK.
## Hardware example
<img src="IoTBrickTemperature.jpg"/>

[IoT Brick Temperature](https://www.thingiverse.com/thing:3638252) on Thingiverse.

## Software examples
### Config
```
final String BASE_URI = "brick.li"; // the registry has a base URI
final String BRICK_ID = "0000-0001"; // each brick has a unique ID
```
### Shared proxy
```
Proxy proxy = MqttProxy.fromConfig(BASE_URI);
// or proxy = MockProxy.fromConfig(BASE_URI);
```
### Door bell
```
ButtonBrick button = ButtonBrick.connect(proxy, BUTTON_BRICK_ID);
BuzzerBrick buzzer = BuzzerBrick.connect(proxy, BUZZER_BRICK_ID);

while (true) {
    boolean pressed = button.isPressed();
    buzzer.setEnabled(pressed);
    proxy.waitForUpdate();
}
```
### Light switch
```
ButtonBrick button = ButtonBrick.connect(proxy, BUTTON_BRICK_ID);
RelayBrick relay = RelayBrick.connect(proxy, RELAY_BRICK_ID);

int state = 0;
while (true) {
    boolean pressed = button.isPressed();
    if (state == 0 && pressed) {
        relay.setEnabled(true);
        state = 1;
    } else if (state == 1 && !pressed) {
        state = 2;
    } else if (state == 2 && pressed) {
        state = 3;
        relay.setEnabled(false);
    } else if (state == 3 && !pressed) {
        state = 0;
    }
    proxy.waitForUpdate();
}
```
### Monitoring system
```
HumiTempBrick sensor = HumiTempBrick.connect(proxy, HUMITEMP_BRICK_ID);
DisplayBrick display = DisplayBrick.connect(proxy, DISPLAY_BRICK_ID);
ColorLedBrick led = ColorLedBrick.connect(proxy, COLORLED_BRICK_ID);

while (true) {
    double temp = sensor.getTemperature();
    display.setDoubleValue(temp);
    Color color = temp > 23 ? Color.RED : Color.GREEN;
    led.setColor(color);
    proxy.waitForUpdate();
}
```
### Logging system
```
HumiTempBrick sensor = HumiTempBrick.connect(proxy, HUMITEMP_BRICK_ID);
FileWriter fileWriter = null;
try {
    fileWriter = new FileWriter("log.csv", true); // append
} catch (IOException e) {
    e.printStackTrace();
}

while (true) {
    double temp = sensor.getTemperature();
    String time = sensor.getTimestampIsoUtc();
    try {
        fileWriter.append(time + ", " + temp + "\n");
        fileWriter.flush();
    } catch (IOException e) {
        e.printStackTrace();
    }
    proxy.waitForUpdate();
}
```
## Software architecture
### Public interfaces
```
public abstract class Brick {
    public String getID();
    public double getBatteryVoltage();
    public Date getTimestamp();
    public String getTimestampIsoUtc();
}

public abstract class DigitalOutputBrick extends Brick {}
public abstract class DigitalInputBrick extends Brick {}

public final class ButtonBrick extends DigitalInputBrick {
    public boolean isPressed();
    public static ButtonBrick connect(Proxy proxy, String brickID);
}

public final class BuzzerBrick extends DigitalOutputBrick {
    public void setEnabled(boolean enabled);
    public static BuzzerBrick connect(Proxy proxy, String brickID);
}

public final class ColorLedBrick extends Brick {
    public void setColor(Color value);
    public static ColorLedBrick connect(Proxy proxy, String brickID);
}

public final class HumiTempBrick extends Brick {
    public double getHumidity();
    public double getTemperature();
    public static HumiTempBrick connect(Proxy proxy, String brickID);
}

public final class DisplayBrick extends Brick;
    public void setDecimalPlaces(int value);
    public void setDoubleValue(double value);
    public static DisplayBrick connect(Proxy proxy, String brickID);
}

public abstract class Proxy {
    public final void waitForUpdate();
}

public final class MqttProxy extends Proxy {
    public static MqttProxy fromConfig(String configBaseURI);
}     

public final class MockProxy extends Proxy {
    public static MockProxy fromConfig(String configBaseURI);
}
```
### Class diagram
<table><tr><td><img width="600" src="IoTBricksClassDiagram.jpg"></td></tr></table>
