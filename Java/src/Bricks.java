// Copyright (c) 2020 FHNW, Switzerland. All rights reserved.
// Licensed under MIT License, see LICENSE for details.

// $ cd Java
// $ curl -Lo lib/minimal-json-0.9.5.jar https://github.com/ralfstx/minimal-json/\
//   releases/download/0.9.5/minimal-json-0.9.5.jar
// $ curl -Lo lib/org.eclipse.paho.client.mqttv3-1.2.3.jar \
//   https://repo.eclipse.org/content/repositories/paho-releases/org/eclipse/paho/\
//   org.eclipse.paho.client.mqttv3/1.2.3/org.eclipse.paho.client.mqttv3-1.2.3.jar
// $ javac -cp .:src:lib/org.eclipse.paho.client.mqttv3-1.2.3.jar:lib/minimal-json-0.9.5.jar src/Bricks.java
// $ java -ea -cp src:lib/org.eclipse.paho.client.mqttv3-1.2.3.jar:lib/minimal-json-0.9.5.jar Bricks

// Design principles:
// - keep it simple to use
//     - physical brick => access
//     - no type casts, no generics 
//     - getValue() remains constant
//       until waitForUpdates()
//     - mock mode for quick prototyping
// - single responsibility
//     - transport x encoding x brick type
// - minimize dependencies
//     - provide a single jar library
//     - use as few libraries as possible
//     - provide server/client certs in code

import java.awt.Color;
import java.io.FileWriter;
import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.nio.charset.StandardCharsets;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.TimeZone;
import org.eclipse.paho.client.mqttv3.IMqttClient;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.IMqttMessageListener;
import org.eclipse.paho.client.mqttv3.IMqttToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;
import com.eclipsesource.json.Json;
import com.eclipsesource.json.JsonValue;

// package ch.fhnw.imvs.iotbricks;

/* public */ final class MqttService {
    public MqttService() {}

    private IMqttClient client = null;

    public void init(String host, String username, String password) {
        try {
            String hostURI = "tcp://" + host;
            String clientID = ""; // defaults to mqtt-client-PROCESS_ID
            client = new MqttClient(hostURI, clientID, new MemoryPersistence());
        } catch (MqttException e) {
            e.printStackTrace();
        }    
    }

    public void connect() {
        try {
            MqttConnectOptions options = new MqttConnectOptions();
            options.setCleanSession(true);
            options.setAutomaticReconnect(true);
            client.connect(options);
        } catch (MqttException e) {
            e.printStackTrace();
        }
    }

    public void subscribe(String topic, IMqttMessageListener listener) {
        try {
            System.out.println("subscribe: topic = " + topic);
            client.subscribe(topic, 1, listener);
        } catch (MqttException e) {
            e.printStackTrace();
        }
    }

    public void publish(String topic, byte[] payload) {
        try {
            client.publish(topic, payload, 1, false);
        } catch (MqttException e) {
            e.printStackTrace();
        }
    }
}

/* public */ final class MqttConfig { // TODO: rename MqttProxyConfig? Ttn...
    private MqttConfig() {}

    private static final String BUTTON_ID = "0000-0002";
    private static final String BUZZER_ID = "0000-0006";
    private static final String HUMITEMP_ID = "0000-0001";
    private static final String HUMITEMP_0_ID = HUMITEMP_ID;
    private static final String HUMITEMP_1_ID = "0000-0003";
    private static final String HUMITEMP_2_ID = "0000-0004";
    private static final String LCDDISPLAY_ID = "0000-0005";
    private static final String LED_ID = "0000-0000";

    private static final String TTN_APP_ID = "fhnw-iot-bricks";
    private static final String TTN_APP_ACCESS_KEY = "<AppAccessKey>";
    private static final String TTN_HOST = "eu.thethings.network";

    private static final String HOST = "test.mosquitto.org"; // TODO: TTN_HOST
    private static final String USERNAME = null; // TODO: TTN_APP_ID
    private static final String PASSWORD = null; // TODO: TTN_APP_ACCESS_KEY

    HashMap<String, String> pubTopics;
    HashMap<String, String> subTopics;

    public String getHost() {
        return HOST;
    }

    public String getUsername() {
        return USERNAME;
    }

    public String getPassword() {
        return PASSWORD;
    }

    public String getSubscribeTopic(String brickID) { // TODO: move to MqttBrickConfig?
        String topic = subTopics.get(brickID);
        if (topic == null) {
            throw new IllegalArgumentException(brickID);
        }
        return topic;
    }

    public String getPublishTopic(String brickID) {
        String topic = pubTopics.get(brickID);
        if (topic == null) {
            throw new IllegalArgumentException(brickID);
        }
        return topic;
    }

    private void init(String configHost) {
        // TODO: get from host or use generic pattern
        subTopics = new HashMap<String, String>();
        subTopics.put(BUTTON_ID, TTN_APP_ID + "/devices/" + BUTTON_ID + "/up");
        subTopics.put(BUZZER_ID, TTN_APP_ID + "/devices/" + BUZZER_ID + "/up");
        subTopics.put(HUMITEMP_0_ID, TTN_APP_ID + "/devices/" + HUMITEMP_0_ID + "/up");
        subTopics.put(HUMITEMP_1_ID, TTN_APP_ID + "/devices/" + HUMITEMP_1_ID + "/up");
        subTopics.put(HUMITEMP_2_ID, TTN_APP_ID + "/devices/" + HUMITEMP_2_ID + "/up");
        subTopics.put(LCDDISPLAY_ID, TTN_APP_ID + "/devices/" + LCDDISPLAY_ID + "/up");
        subTopics.put(LED_ID, TTN_APP_ID + "/devices/" + LED_ID + "/up");
        pubTopics = new HashMap<String, String>();
        pubTopics.put(BUZZER_ID, TTN_APP_ID + "/devices/" + BUZZER_ID + "/down");
        pubTopics.put(LED_ID, TTN_APP_ID + "/devices/" + LED_ID + "/down");
    }

    public static MqttConfig fromHost(String configHost) {
        MqttConfig config = new MqttConfig();
        config.init(configHost);
        return config;
    }
}

/* public */ abstract class Proxy {
    abstract void connectBrick(Brick brick);
    abstract public void waitForUpdate();
    // updated = updated || now.before(brick.getNextTimestamp());
}

/* public */ final class HttpProxy extends Proxy {
    // TODO: implementation
    private HttpProxy() {
        bricks = new ArrayList<Brick>();
    }

    private final List<Brick> bricks;

    @Override
    void connectBrick(Brick brick) {
        bricks.add(brick);
    }

    @Override
    public void waitForUpdate() {}

    public static HttpProxy fromConfig(String configHost) {
        return new HttpProxy();
    }
}

/* public */ final class MockProxy extends Proxy {
    private MockProxy() {
        bricks = new ArrayList<Brick>();
    }

    private final List<Brick> bricks;

    @Override
    void connectBrick(Brick brick) {
        bricks.add(brick);
    }

    @Override
    public void waitForUpdate() {
        for (Brick brick : bricks) {
            byte[] payload = brick.getTargetPayload(true); // mock
            if (payload != null) {
                brick.setCurrentPayload(payload);
            }
        }
        try {
            TimeUnit.MILLISECONDS.sleep(1000); // ms
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public static MockProxy fromConfig(String configHost) {
        return new MockProxy();
    }
}

/* public */ final class MqttProxy extends Proxy {
    private MqttProxy(MqttConfig config) {
        mqttConfig = config;
        mqttService = new MqttService();
        bricks = new ArrayList<Brick>();
    }

    private final MqttConfig mqttConfig;
    private final MqttService mqttService;
    private final List<Brick> bricks;

    // calLed exactly once
    private void connect() {
        String host = mqttConfig.getHost();
        String username = mqttConfig.getUsername();
        String password = mqttConfig.getPassword();
        mqttService.init(host, username, password);
        mqttService.connect();
    }

    @Override
    void connectBrick(Brick brick) {
        String topic = mqttConfig.getSubscribeTopic(brick.getID());
        IMqttMessageListener listener = new IMqttMessageListener() {
            public void messageArrived(String topic, MqttMessage message) throws Exception {
                System.out.printf("topic = \"%s\", payload = \"%s\"\n", topic, message);
                byte[] payload = message.getPayload();
                brick.setCurrentPayload(payload); // setPendingPayload() ?
            }
        };
        mqttService.subscribe(topic, listener);
        bricks.add(brick);
    }

    @Override
    public void waitForUpdate() {
        for (Brick brick : bricks) {
            byte[] payload = brick.getTargetPayload(false); // not a mock
            if (payload != null) {
                String topic = mqttConfig.getPublishTopic(brick.getID());
                mqttService.publish(topic, payload);
            }
        }
        try {
            TimeUnit.MILLISECONDS.sleep(1000); // ms
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public static MqttProxy fromConfig(String configHost) {
        MqttConfig config = MqttConfig.fromHost(configHost);
        MqttProxy proxy = new MqttProxy(config); // TODO: singleton per configHost
        proxy.connect();
        return proxy;
    }
}

/* public */ abstract class Brick {
    protected Brick(String brickID) {
        this.brickID = brickID;
        // TODO: move to Proxy?
        timeZone = TimeZone.getTimeZone("UTC");
        formatter = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'");
        formatter.setTimeZone(timeZone);
    }

    private final String brickID;
    
    public String getID() {
        return brickID;
    }

    private int currentEnergyLevel = 0;
    private Date currentTimestamp = new Date(0L);
    
    private TimeZone timeZone;
    private DateFormat formatter;

    public int getEnergyLevel() {
        return currentEnergyLevel;
    }

    public Date getTimestamp() {
        return currentTimestamp;
    }

    public String getTimestampIsoUtc() {
        return formatter.format(currentTimestamp);
    }

    abstract protected void setCurrentPayload(byte[] payload);
    abstract protected byte[] getTargetPayload(boolean mock);
}

/* public */ final class ButtonBrick extends Brick {
    private ButtonBrick(String brickID) {
        super(brickID);
    }

    public boolean isPressed() { return false; }
    public void setPressed(boolean pressed) {}

    @Override
    protected void setCurrentPayload(byte[] payload) {}

    @Override
    protected byte[] getTargetPayload(boolean mock) { return null; }

    public static ButtonBrick connect(Proxy proxy, String brickID) {
        ButtonBrick brick = new ButtonBrick(brickID);
        proxy.connectBrick(brick);
        return brick;
    }
}

/* public */ final class BuzzerBrick extends Brick {
    private BuzzerBrick(String brickID) {
        super(brickID);
    }

    // TODO: rename to triggerAlert(int ms)?
    public void setEnabled(boolean enabled) {}

    @Override
    protected void setCurrentPayload(byte[] payload) {}

    @Override
    protected byte[] getTargetPayload(boolean mock) { return null; }

    public static BuzzerBrick connect(Proxy proxy, String brickID) {
        BuzzerBrick brick = new BuzzerBrick(brickID);
        proxy.connectBrick(brick);
        return brick;
    }
}

/* public */ final class HumiTempBrick extends Brick {
    private HumiTempBrick(String brickID) {
        super(brickID);
    }

    private final String SEPARATOR = ";";
    private volatile double currentHumi;
    private volatile double currentTemp;

    public double getHumidity() {
        return currentHumi;
    }

    public double getTemperature() {
        return currentTemp;
    }

    @Override
    protected void setCurrentPayload(byte[] payload) {
        try {
            String message = new String(payload, StandardCharsets.UTF_8);
            String[] parts = message.split(SEPARATOR); // treated as a regex (!)
            currentHumi = Double.parseDouble(parts[0]);
            currentTemp = Double.parseDouble(parts[1]);
        } catch(NumberFormatException e) {
            e.printStackTrace();
        }
    }

    @Override
    protected byte[] getTargetPayload(boolean mock) {
        byte[] payload;
        if (mock) {
            double targetHumi = Math.random() * 99 + 1;
            double targetTemp = Math.random() * 50 + 1;
            try {
                String payloadString = 
                    Double.toString(targetHumi) + SEPARATOR + 
                    Double.toString(targetTemp);
                payload = payloadString.getBytes("UTF-8");
            } catch (UnsupportedEncodingException e) {
                e.printStackTrace();
                payload = null;
            }
        } else {
            payload = null;
        }
        return payload;
    }

    public static HumiTempBrick connect(Proxy proxy, String brickID) {
        // TODO: proxy.getEncoding(brickID) { return mqttConfig.getEncoging(brickID); }
        // => ProtobufHumiTempBrick(), LppHumiTempBrick()
        HumiTempBrick brick = new HumiTempBrick(brickID);
        proxy.connectBrick(brick);
        return brick;
    }
}

/* public */ final class LedBrick extends Brick {
    private LedBrick(String brickID) {
        super(brickID);
    }

    private volatile Color currentColor;
    private volatile Color targetColor;

    public Color getColor() {
        return currentColor;
    }

    public void setColor(Color color) {
        targetColor = color;
    }

    @Override
    protected void setCurrentPayload(byte[] payload) {
        try {
            // TODO: decode real format
            String message = new String(payload, StandardCharsets.UTF_8);
            currentColor = Color.decode(message);
        } catch(NumberFormatException e) {
            e.printStackTrace();
        }
    }

    @Override
    protected byte[] getTargetPayload(boolean mock) {
        // ignore mock flag
        byte[] payload;
        try {
            int r = targetColor.getRed();
            int g = targetColor.getGreen();
            int b = targetColor.getBlue();
            String colorString = 
                String.format("#%02x%02x%02x", r, g, b);  
            payload = colorString.getBytes("UTF-8");
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
            payload = null;
        }
        return payload;
    }

    public static LedBrick connect(Proxy proxy, String brickID) {
        LedBrick brick = new LedBrick(brickID);
        proxy.connectBrick(brick);
        return brick;
    }
}

/* public */ final class LedStripBrick extends Brick {
    private LedStripBrick(String brickID) {
        super(brickID);
    }

    public void setColors(Color[] values) {}

    @Override
    protected void setCurrentPayload(byte[] payload) {}

    @Override
    protected byte[] getTargetPayload(boolean mock) { return null; }

    public static LedStripBrick connect(Proxy proxy, String brickID) {
        LedStripBrick brick = new LedStripBrick(brickID);
        proxy.connectBrick(brick);
        return brick;
    }
}

/* public */ final class LcdDisplayBrick extends Brick {
    private LcdDisplayBrick(String brickID) {
        super(brickID);
    }

    public void setDoubleValue(double value) {}

    public double getDoubleValue() {
        return 0.0;
    }

    @Override
    protected void setCurrentPayload(byte[] payload) {}

    @Override
    protected byte[] getTargetPayload(boolean mock) { return null; }

    public static LcdDisplayBrick connect(Proxy proxy, String brickID) {
        LcdDisplayBrick brick = new LcdDisplayBrick(brickID);
        proxy.connectBrick(brick);
        return brick;
    }
}

public final class Bricks {
    private Bricks() {}

    private static final String BUTTON_ID = "0000-0002";
    private static final String BUZZER_ID = "0000-0006";
    private static final String HUMITEMP_ID = "0000-0001";
    private static final String HUMITEMP_0_ID = HUMITEMP_ID;
    private static final String HUMITEMP_1_ID = "0000-0003";
    private static final String HUMITEMP_2_ID = "0000-0004";
    private static final String LCDDISPLAY_ID = "0000-0005";
    private static final String LED_ID = "0000-0000";

    private static void runDoorbellExample(Proxy proxy) {
        ButtonBrick buttonBrick = ButtonBrick.connect(proxy, BUTTON_ID);
        BuzzerBrick buzzerBrick = BuzzerBrick.connect(proxy, BUZZER_ID);
        while (true) {
            boolean pressed = buttonBrick.isPressed();
            String time = buttonBrick.getTimestampIsoUtc();
            System.out.println(time + ", " +  pressed);
            buzzerBrick.setEnabled(pressed);
            proxy.waitForUpdate();
        }
    }

    private static void runLoggingExample(Proxy proxy) {
        HumiTempBrick brick = HumiTempBrick.connect(proxy, HUMITEMP_ID);
        FileWriter fileWriter = null;
        String title = "Timestamp (UTC)\tTemperature\tHumidity\n";
        System.out.print(title);
        try {
            fileWriter = new FileWriter("log.csv", true); // append
            fileWriter.append(title);
        } catch (IOException e) {
            e.printStackTrace();
        }
        while (true) {
            String time = brick.getTimestampIsoUtc();
            double temp = brick.getTemperature();
            double humi = brick.getHumidity();
            String line = String.format(Locale.US, "%s, %.2f, %.2f\n", time, temp, humi);
            System.out.print(line);
            try {
                fileWriter.append(line);
                fileWriter.flush();
            } catch (IOException e) {
                e.printStackTrace();
            }
            proxy.waitForUpdate();
        }
    }

    private static void runLoggingArrayExample(Proxy proxy) {
        HumiTempBrick[] bricks = new HumiTempBrick[3];
        bricks[0] = HumiTempBrick.connect(proxy, HUMITEMP_0_ID);
        bricks[1] = HumiTempBrick.connect(proxy, HUMITEMP_1_ID);
        bricks[2] = HumiTempBrick.connect(proxy, HUMITEMP_2_ID);
        while (true) {
            for (HumiTempBrick brick : bricks) {
                String id = brick.getID();
                String time = brick.getTimestampIsoUtc();
                double temp = brick.getTemperature();
                double humi = brick.getHumidity();
                String line = String.format(Locale.US, "%s, %s, %.2f, %.2f", id, time, temp, humi);
                System.out.println(line);
            }
            proxy.waitForUpdate();
        }
    }

    private static void runMonitoringExample(Proxy proxy) {
        HumiTempBrick humiTempBrick = HumiTempBrick.connect(proxy, HUMITEMP_ID);
        LcdDisplayBrick displayBrick = LcdDisplayBrick.connect(proxy, LCDDISPLAY_ID);
        LedBrick ledBrick = LedBrick.connect(proxy, LED_ID);
        while (true) {
            double temp = humiTempBrick.getTemperature();
            Color color = temp > 23 ? Color.RED : Color.GREEN;
            String time = humiTempBrick.getTimestampIsoUtc();
            String line = String.format(Locale.US, "%s, %.2f, %s\n", time, temp, color);
            System.out.print(line);
            displayBrick.setDoubleValue(temp);
            ledBrick.setColor(color);
            proxy.waitForUpdate();
        }
    }

    public static void main(String args[]) {
        final String BASE_URL = "https://brick.li";
        final String USAGE = "usage: java Bricks http|mqtt|mock d|l|a|m";
        if (args.length == 2) {
            Proxy proxy = null;
            if ("http".equals(args[0])) {
                proxy = HttpProxy.fromConfig(BASE_URL);
            } else if ("mqtt".equals(args[0])) {
                proxy = MqttProxy.fromConfig(BASE_URL);
            } else if ("mock".equals(args[0])) {
                proxy = MockProxy.fromConfig(BASE_URL);
            } else {
                System.out.println(USAGE);
                System.exit(-1);
            }
            if ("d".equals(args[1])) {
                runDoorbellExample(proxy);
            } else if ("l".equals(args[1])) {
                runLoggingExample(proxy);
            } else if ("a".equals(args[1])) {
                runLoggingArrayExample(proxy);    
            } else if ("m".equals(args[1])) {
                runMonitoringExample(proxy);
            } else {
                System.out.println(USAGE);
                System.exit(-1);
            }
        } else {
            System.out.println(USAGE);
            System.exit(-1);
        }
    }
}