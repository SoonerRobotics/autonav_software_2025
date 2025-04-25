const WebSocket = require('ws');
const server = new WebSocket.Server({port: 8080});
server.on('connection', (ws) => {
    console.log('Client connected');

    const sendData = () => {
        let deviceState = JSON.stringify({
            op: 'data',
            topic: TOPIC_SYSTEM_STATE, // Replaced with constant
            timestamp: new Date().toISOString(),
            device: 'autonav',
            state: '0'
        });

        let logging = JSON.stringify({
            op: 'data',
            topic: TOPIC_PLAYBACK, // Replaced with constant
            timestamp: new Date().toISOString(),
            data: 'data:image/jpeg;base64,',
            node: 'autonav'
        });

        let imgData = 'https://www.greenlaundry.net/blog/wp-content/uploads/2011/11/animalsintrees-1.gif';

        let conbus = JSON.stringify({
            op: 'data',
            topic: TOPIC_CONBUS, // Replaced with constant
            timestamp: new Date().toISOString(),
            id: 0,
            data: imgData
        });

        let feelers = JSON.stringify({
            op: 'data',
            topic: TOPIC_FEELERS, // Replaced with constant
            timestamp: new Date().toISOString(),
            data: imgData
        });

        let combined = JSON.stringify({
            op: 'data',
            topic: TOPIC_COMBINED_IMAGE, // Replaced with constant
            timestamp: new Date().toISOString(),
            data: imgData
        });

        let leftSmall = JSON.stringify({
            op: 'data',
            topic: TOPIC_RAW_LEFT, // Replaced with constant
            timestamp: new Date().toISOString(),
            data: imgData
        });

        let rightSmall = JSON.stringify({
            op: 'data',
            topic: TOPIC_RAW_RIGHT, // Replaced with constant
            timestamp: new Date().toISOString(),
            data: imgData
        });

        let front = JSON.stringify({
            op: 'data',
            topic: TOPIC_RAW_FRONT, // Replaced with constant
            timestamp: new Date().toISOString(),
            data: imgData
        });

        let back = JSON.stringify({
            op: 'data',
            topic: TOPIC_RAW_BACK, // Replaced with constant
            timestamp: new Date().toISOString(),
            data: imgData
        });
        /*        let compressedRight = JSON.stringify({
                    op: 'data',
                    topic: TOPIC_RAW_RIGHT, // Replaced with constant
                    timestamp: new Date().toISOString(),
                    data: imgData
                });

                let compressedLeft = JSON.stringify({
                    op: 'data',
                    topic: TOPIC_RAW_LEFT, // Replaced with constant
                    timestamp: new Date().toISOString(),
                    data: imgData
                });*/

        let imuData = JSON.stringify({
            op: 'data',
            topic: TOPIC_IMU, // Replaced with constant
            timestamp: new Date().toISOString(),
            accel_x: Math.random() * 10,
            accel_y: Math.random() * 10,
            accel_z: Math.random() * 10,
            angular_x: Math.random() * 10,
            angular_y: Math.random() * 10,
            angular_z: Math.random() * 10,
            yaw: Math.random() * 10,
            pitch: Math.random() * 10,
            roll: Math.random() * 10
        });

        let positionData = JSON.stringify({
            op: 'data',
            topic: TOPIC_POSITION, // Replaced with constant
            timestamp: new Date().toISOString(),
            x: Math.random() * 10,
            y: Math.random() * 10,
            theta: Math.random() * 10,
            latitude: Math.random() * 10,
            longitude: Math.random() * 10
        });

        let motorFeedbackData = JSON.stringify({
            op: 'data',
            topic: TOPIC_MOTOR_FEEDBACK, // Replaced with constant
            timestamp: new Date().toISOString(),
            delta_x: Math.random() * 10,
            delta_y: Math.random() * 10,
            delta_theta: Math.random() * 10
        });

        let gpsData = JSON.stringify({
            op: 'data',
            topic: TOPIC_AUTONAV_GPS, // Replaced with constant
            timestamp: new Date().toISOString(),
            latitude: Math.random() * 10,
            longitude: Math.random() * 10,
            gps_fix: Math.random() * 10,
            is_locked: true,
            satellites: Math.random() * 10
        });

        let sysState = JSON.stringify({
            op: 'data',
            topic: TOPIC_SYSTEM_STATE, // Replaced with constant
            timestamp: new Date().toISOString(),
            state: '0',
            mode: 'autonomous',
            mobility: 'mobile'
        });

        let motorInput = JSON.stringify({
            op: 'data',
            topic: TOPIC_MOTOR_INPUT, // Replaced with constant
            timestamp: new Date().toISOString(),
            forward_velocity: Math.random() * 10,
            angular_velocity: Math.random() * 10
        });

        ws.send(deviceState);
        ws.send(logging);
        ws.send(conbus);
        setInterval(() => {
            ws.send(combined);
            ws.send(leftSmall);
            ws.send(rightSmall);
            ws.send(feelers);
            /* deprecated
                        ws.send(compressedRight);
                        ws.send(compressedLeft);
            */
            ws.send(front);
            ws.send(back);
        }, 500);
        ws.send(imuData);
        ws.send(positionData);
        ws.send(motorFeedbackData);
        ws.send(gpsData);
        ws.send(sysState);
        ws.send(motorInput);
    };
    // Send data every second
    const interval = setInterval(sendData, 1000);

    ws.on('close', () => {
        console.log('Client disconnected');
        clearInterval(interval);
    });

    ws.on('error', (error) => {
        console.error('WebSocket error:', error);
        clearInterval(interval);
    });
});


console.log('WebSocket server is running on ws://localhost:8080');

/*Topics can't be used from globals.js since they don't render client side so I'm just copy/pasting whatever's in globals.js*/
// Topic Listeneres
const TOPIC_SYSTEM_STATE = "autonav/shared/system";
const TOPIC_DEVICE_STATE = "autonav/shared/device";

// IMU Data
const TOPIC_IMU = "/autonav/imu";
const TOPIC_AUTONAV_GPS = "/autonav/gps";
const TOPIC_MOTOR_INPUT = "/autonav/MotorInput";
const TOPIC_POSITION = "/autonav/position";

const TOPIC_MOTOR_FEEDBACK = "/autonav/MotorFeedback";
const TOPIC_NUCStatistics = '/autonav/statistic';
const TOPIC_ULTRASONICS = '/autonav/ultrasonics';
const TOPIC_CONBUS = "/autonav/conbus";
const TOPIC_SAFETY_LIGHTS = '/autonav/safety_lights';
const TOPIC_PERFORMANCE = 'autonav/performance';

const TOPIC_RAW_LEFT = 'autonav/camera/left';// TODO NEW NODES, IMPLEMENT
const TOPIC_RAW_RIGHT = 'autonav/camera/right';// TODO NEW NODES, IMPLEMENT
const TOPIC_RAW_FRONT = 'autonav/camera/front';// TODO NEW NODES, IMPLEMENT
const TOPIC_RAW_BACK = 'autonav/camera/back';// TODO NEW NODES, IMPLEMENT

//Other Camera Nodes
const TOPIC_COMBINED_IMAGE = '/autonav/vision/combined/filtered'
const TOPIC_FEELERS = '/autonav/feelers/debug';// todo does this transmit an image? (assuming it does for now

// Others
const TOPIC_CONFIGURATION = "/scr/configuration";// TODO IS THIS STILL A TOPIC?
const TOPIC_PLAYBACK = "autonav/autonav_playback"; //TODO feed in new data and test if  this actually gets data in