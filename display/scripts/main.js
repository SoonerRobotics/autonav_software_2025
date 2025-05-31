document.addEventListener("DOMContentLoaded", function () {    // Check if local storage has preferences
    if (localStorage.getItem("preferences") == null) {
        savePreferences();
    } else {
        preferences = JSON.parse(localStorage.getItem("preferences"));

        $("#input_host").val(preferences.host);
        $("#input_port").val(preferences.port);

        $("html").attr("data-bs-theme", preferences.theme);
    }

    let websocket;

    const createWebsocket = () => {// check if pref.host is being overwritten somewhr? (See default values aren't being loaded in...)
        $("#main").show();
        const userID = generateUUID();
        const fallbackHost = 'localhost';
        const host = development_mode ? fallbackHost : preferences.host;
        const port = preferences.port;
        const wsUrl = `ws://${host}:${port}/?id=${userID}`;

        console.log("Connecting to WebSocket:", wsUrl);
        websocket = new WebSocket(wsUrl);

        websocket.onopen = function (event) {
            if (connected) {
                connected = !connected;
                ntfClear(); // Clear the persistent notification
            }

            ntf('Connected to the server', 'success');
            $("#connecting-state").text("Updating Data");//fixme connecting state is deprecated, need to re-estab it again

            send({op: "broadcast"});
            send({op: "get_nodes"});
            send({op: "get_presets"});

            const waitInterval = setInterval(() => {
                if (deviceStates["autonav_serial_can"] !== 3) {
                    return;
                }

                clearInterval(waitInterval);
                const conbusDeviceIds = Object.keys(conbusDevices);
                for (let i = 0; i < conbusDeviceIds.length; i++) {
                    const deviceId = parseInt(conbusDeviceIds[i]);
                    setTimeout(() => {
                        send({
                            op: "conbus",
                            ...createConbusReadInstruction(deviceId, 0xFF),
                            iterator: iterate()
                        })
                    }, 250 * i);
                }
            }, 500);

            setTimeout(() => {

            }, 1000);

            setTimeout(() => {
                if (websocket.readyState === 1) {
                    send({op: "get_presets"});
                }
            }, 3000);
        };


        websocket.onmessage = function (event) {
            const messages = event.data.split("\n");
            for (const message of messages) {
                const obj = JSON.parse(message);
                const {op, topic} = obj;

                if (op === "data") {
                    onTopicData(topic, obj);
                }

                if (op === "get_log_files_callback" || op === "get_log_file_content_callback") {
                    if (typeof handleWebSocketMessage === "function") {
                        handleWebSocketMessage(obj);
                    }
                }

                if (op === "get_presets_callback") {

                    const serverPresets = obj.presets || [];

                    const localPresets = JSON.parse(localStorage.getItem("presets") || "{}");

                    const presetElement = $("#dropdown_elements");
                    presetElement.empty();

                    for (const preset of serverPresets) {
                        if (localPresets[preset]) continue;

                        const dropdownItem = $(`<li><a class="dropdown-item" data-value="${preset}">${preset} (server)</a></li>`);
                        presetElement.append(dropdownItem);

                        dropdownItem.on("click", function () {
                            const preset_name = $(this).children().attr("data-value");
                            send({
                                op: "set_active_preset",
                                preset: preset_name
                            });
                            send({op: "get_presets"});
                        });
                    }

                    // Add localStorage presets
                    for (const preset in localPresets) {
                        const dropdownItem = $(`<li><a class="dropdown-item" data-value="${preset}">${preset}</a></li>`);
                        presetElement.append(dropdownItem);

                        dropdownItem.on("click", function() {
                            const preset_name = $(this).children().attr("data-value");
                            loadPreset(preset_name);
                        });
                    }

                    // If no presets, add a message
                    if (serverPresets.length === 0 && Object.keys(localPresets).length === 0) {
                        presetElement.append('<li><a class="dropdown-item disabled">No presets available</a></li>');
                    }

                    // Set current preset
                    if (obj.active_preset) {
                        current_preset = obj.active_preset;
                        $("#active_preset_value").text(current_preset);
                    }
                }

                if (op === "get_nodes_callback") {
                    console.log(obj);
                    for (let i = 0; i < obj.nodes.length; i++) {
                        const node = obj.nodes[i];
                        send({
                            op: "configuration",
                            device: node,
                            opcode: 4,
                            iterator: iterate()
                        });

                        const statemap = obj.states;
                        if (node in statemap) {
                            if (node === "rosbridge_websocket" || node === "rosapi" || node === "scr_core" || node === "rosapi_params") {
                                continue;
                            }

                            deviceStates[node] = statemap[node];
                            unorderedListElement = $("#element_device_states");
                            unorderedListElement.empty();
                            for (const id in deviceStates) {
                                const state = deviceStates[id];
                                unorderedListElement.append(`<h5>${id}: <span data-state=\"${state}\">${deviceStateToName(state)}</span></h5>`);
                            }
                        }
                    }

                    for (const key in obj.configs) { //TODO configs never setup, this for loop is useless
                        config[key] = obj.configs[key];
                    }
                    regenerateConfig();

                    // Update system state
                    let system = obj["system"];//TODO 5/11/2024 make system state print to ensure that its being logged
                    $("#var_system_state").text(system["state"] === 0 ? "Disabled" : system["state"] === 1 ? "Autonomous" : system["state"] === 2 ? "Manual" : "Shutdown");
                    $("#var_system_mode").text(system["mode"] === 0 ? "Competition" : system["mode"] === 1 ? "Simulation" : "Practice");
                    $("#var_system_mobility").text(system["mobility"] ? "Enabled" : "Disabled");

                    // Update some buttons
                    $("#checkbox_system_mobility").prop("checked", system["mobility"]);
                    $("#input_system_state").val(system["state"]);
                }
            }
        };

        websocket.onclose = function (event) {
            clearGlobals();
            if (!connected) {
                ntf('Disconnected from the server', 'error');
                connected = !connected
            }

            setTimeout(() => {
                createWebsocket();
            }, 1);
        };

        websocket.onerror = function (event) {
            console.error(event);

        };
    }

    if (!development_mode) {
        /* Don't really need I guess? Since user should always assume thye are not in Dev mode
                window.onload = function () {
                    ntf('Dev Mode is disabled', 'alert');
                };*/
        createWebsocket();
    } else {
        window.onload = function () {
            ntf('Development Mode is enabled', 'alert');
        };
        createWebsocket();
    }

    const sendQueue = [];

    function setSystemState() {
        send({
            op: "set_system_state",
            state: systemState.state,
            mode: systemState.mode,
            mobility: systemState.mobility,
        });
    }

    function generateElementForConbus(data, type, text, deviceId, address, readonly = false) {
        if (type === "bool") {
            const checked = fromBytesToBool(data);

            // Create a dropdown
            const div = document.createElement("div");
            div.classList.add("input-group");
            div.classList.add("mb-3");

            const select = document.createElement("select");
            select.disabled = readonly;
            select.classList.add("form-select");
            select.onchange = function () {
                const instruction = createConbusWriteInstruction(
                    parseInt(deviceId),
                    parseInt(address),
                    Array.from(fromBoolToBytes(select.value === 1))
                )
                send({
                    op: "conbus",
                    ...instruction,
                    iterator: iterate()
                });
            }

            const optionTrue = document.createElement("option");
            optionTrue.value = 1;
            optionTrue.innerText = "True";
            optionTrue.selected = checked;

            const optionFalse = document.createElement("option");
            optionFalse.value = 0;
            optionFalse.innerText = "False";
            optionFalse.selected = !checked;

            select.appendChild(optionTrue);
            select.appendChild(optionFalse);

            const span = document.createElement("span");
            span.classList.add("input-group-text");
            span.innerText = text;

            div.appendChild(span);
            div.appendChild(select);
            return div;
        } else if (type === "float") {
            const div = document.createElement("div");
            div.classList.add("input-group");
            div.classList.add("mb-3");

            const input = document.createElement("input");
            input.type = "number";
            input.classList.add("form-control");
            input.value = fromBytesToFloat(data).toFixed(6);
            input.disabled = readonly;
            input.onchange = function () {
                const instruction = createConbusWriteInstruction(
                    parseInt(deviceId),
                    parseInt(address),
                    Array.from(fromFloatToBytes(input.value))
                )
                send({
                    op: "conbus",
                    ...instruction,
                    iterator: iterate()
                });
            }

            const span = document.createElement("span");
            span.classList.add("input-group-text");
            span.innerText = text;

            div.appendChild(span);
            div.appendChild(input);
            return div;
        } else if (type === "int") {
            const div = document.createElement("div");
            div.classList.add("input-group");
            div.classList.add("mb-3");

            const input = document.createElement("input");
            input.type = "number";
            input.classList.add("form-control");
            input.value = fromBytesToInt(data);
            input.disbled = readonly;
            input.onchange = function () {
                const instruction = createConbusWriteInstruction(
                    parseInt(deviceId),
                    parseInt(address),
                    Array.from(fromIntToBytes(input.value))
                )
                send({
                    op: "conbus",
                    ...instruction,
                    iterator: iterate()
                });
            }

            const span = document.createElement("span");
            span.classList.add("input-group-text");
            span.innerText = text;

            div.appendChild(span);
            div.appendChild(input);
            return div;
        } else if (type === "uint") {
            const div = document.createElement("div");
            div.classList.add("input-group");
            div.classList.add("mb-3");

            const input = document.createElement("input");
            input.type = "number";
            input.classList.add("form-control");
            input.value = fromBytesToUInt(data);
            input.disbled = readonly;
            input.onchange = function () {
                const instruction = createConbusWriteInstruction(
                    parseInt(deviceId),
                    parseInt(address),
                    Array.from(fromUIntToBytes(input.value))
                )

                send({
                    op: "conbus",
                    ...instruction,
                    iterator: iterate()
                });
            }

            const span = document.createElement("span");
            span.classList.add("input-group-text");
            span.innerText = text;

            div.appendChild(span);
            div.appendChild(input);
            return div;
        }
    }

    setInterval(() => {
        if (sendQueue.length > 0 && websocket.readyState === 1 && websocket.bufferedAmount === 0) {
            const obj = sendQueue.shift();
            websocket.send(JSON.stringify(obj));
        }
    }, 10);

    function onTopicData(topic, msg) {
        const {iterator} = msg;
        if (iterator !== undefined && iterators.includes(iterator)) {
            iterators.splice(iterators.indexOf(iterator), 1);
            return;
        }

        switch (topic) {
            //System
            case TOPIC_BROADCAST: {
                console.log("Broadcast message received:", msg);
                $("#var_broadcast").text(JSON.stringify(msg));
                break;
            }
            case TOPIC_SYSTEM_STATE: {
                const {state, mode, mobility} = msg;

                $("#var_system_state").text(
                    state === 0 ? "Disabled" : state === 1 ? "Autonomous" : state === 2 ? "Manual" : state === 3? "SelfDrive" : "Shutdown"
                );
                $("#var_system_mode").text(
                    mode === 0 ? "Competition" : mode === 1 ? "Simulation" : "Practice"
                );
                $("#var_system_mobility").text(mobility ? "Enabled" : "Disabled");

                // Check if estop status has changed
                const previousEstop = systemState.estop;
                const currentEstop = state >= 4; // Consider estop activated when state is Shutdown (4+)

                systemState.state = state;
                systemState.mode = mode;
                systemState.mobility = mobility;
                systemState.estop = currentEstop;

                // Notify if estop is activated
                if (currentEstop && !previousEstop) {
                    ntf('Emergency Stop Activated!', 'error');
                }

                $("#input_system_state").val(state);
                $("#input_system_mode").val(mode);
                $("#input_system_mobility").prop("checked", mobility);
                break;
            }
            case TOPIC_DEVICE_STATE: {
                const {device, state} = msg;
                deviceStates[device] = state;
                unorderedListElement = $("#element_device_states");
                unorderedListElement.empty();
                for (const id in deviceStates) {
                    const state = deviceStates[id];
                    unorderedListElement.append(
                        `<h5>${id}: <span data-state="${state}">${deviceStateToName(state)}</span></h5>`
                    );
                }
                break;
            }
            case TOPIC_LOG: {
                const {level, message, node} = msg;
                logs.push({message, node, timestamp: new Date(), level});
                if (logs.length > 30) logs.shift();

                const logElement = $("#log_body");
                logElement.empty();
                for (let i = logs.length - 1; i >= 0; i--) {
                    const log = logs[i];
                    const tableEntry = $("<tr></tr>");
                    tableEntry.append(`<td>${log.timestamp.toTimeString().split(" ")[0]}</td>`);
                    tableEntry.append(`<td>${log.node}</td>`);
                    tableEntry.append(`<td data-level="${log.level || 0}">${log.message}</td>`);
                    logElement.append(tableEntry);
                }
                break;
            }

            //IMU
            case TOPIC_IMU: {
                const {accel_x, accel_y, accel_z, angular_x, angular_y, angular_z, yaw, pitch, roll} = msg;
                $("#var_imu_acceleration").text(
                    `(${formatToFixed(accel_x, 4)}, ${formatToFixed(accel_y, 4)}, ${formatToFixed(accel_z, 4)})`
                );
                $("#var_imu_angular").text(
                    `(${formatToFixed(angular_x, 4)}, ${formatToFixed(angular_y, 4)}, ${formatToFixed(angular_z, 4)})`
                );
                $("#var_imu_orientation").text(
                    `(${radiansToDegrees(parseFloat(yaw)).toFixed(3)}°, ${radiansToDegrees(parseFloat(pitch)).toFixed(3)}°, ${radiansToDegrees(parseFloat(roll)).toFixed(3)}°)`
                );
                break;
            }
            case TOPIC_AUTONAV_GPS: {
                const {latitude, longitude, gps_fix, is_locked, satellites} = msg;
                $("#var_gps_position").text(formatLatLong(latitude, longitude, true));
                $("#var_gps_fix").text(gps_fix);
                $("#var_gps_fixed").text(is_locked ? "Locked" : "Not Locked");
                $("#var_gps_satellites").text(satellites);
                break;
            }
            case TOPIC_MOTOR_INPUT: {
                const {forward_velocity, angular_velocity} = msg;
                $("#var_motors_velocity").text(
                    `(${formatToFixed(forward_velocity, 3)}, ${formatToFixed(angular_velocity, 3)})`
                );
                break;
            }
            case TOPIC_POSITION: {
                const {x, y, theta, latitude, longitude} = msg;
                $("#var_position_origin").text(
                    `(${formatToFixed(x, 4)}, ${formatToFixed(y, 4)}, ${radiansToDegrees(parseFloat(theta)).toFixed(3)}°)`
                );
                $("#var_position_global").text(
                    `(${formatToFixed(latitude, 8)}, ${formatToFixed(longitude, 8)})`
                );
                break;
            }
            case TOPIC_CONTROLLER_INPUT: {
                const {buttons, axes} = msg;
                // Display controller input data if UI elements exist
                if ($("#var_controller_buttons").length) {
                    $("#var_controller_buttons").text(JSON.stringify(buttons));
                }
                if ($("#var_controller_axes").length) {
                    $("#var_controller_axes").text(JSON.stringify(axes));
                }
                break;
            }

            // Motor and System Feedback
            case TOPIC_MOTOR_FEEDBACK: {
                const {delta_x, delta_y, delta_theta} = msg;
                $("#var_motors_feedback").text(
                    `(${formatToFixed(delta_x, 4)}, ${formatToFixed(delta_y, 4)}, ${formatToFixed(delta_theta, 4)}°)`
                );
                break;
            }
            case TOPIC_NUC_STATISTICS: {
                const {cpu_usage, memory_usage, disk_usage} = msg;
                if ($("#var_nuc_cpu").length) {
                    $("#var_nuc_cpu").text(`${formatToFixed(cpu_usage, 2)}%`);
                }
                if ($("#var_nuc_memory").length) {
                    $("#var_nuc_memory").text(`${formatToFixed(memory_usage, 2)}%`);
                }
                if ($("#var_nuc_disk").length) {
                    $("#var_nuc_disk").text(`${formatToFixed(disk_usage, 2)}%`);
                }
                break;
            }
            case TOPIC_ULTRASONICS: {
                const {distances} = msg;
                if ($("#var_ultrasonics").length) {
                    $("#var_ultrasonics").text(JSON.stringify(distances));
                }
                break;
            }
            case TOPIC_CONBUS_DATA: {
                const {id, data} = msg;
                let response = id >= 1100 && id < 1200 ? createConbusReadResponse(id, data) :
                    id >= 1300 && id < 1400 ? createConbusWriteResponse(id, data) : null;
                if (!response || !(response.id in conbusDevices)) break;

                if (!(response.id in conbus)) conbus[response.id] = {};
                conbus[response.id][response.address] = response.data;

                updateConbusUI(response);
                break;
            }
            case TOPIC_CONBUS_INSTRUCTION: {
                console.log("Conbus instruction received:", msg);
                $("#var_conbus_instruction").text(JSON.stringify(msg));
                break;
            }
            case TOPIC_CONBUS: {
                // Alias for TOPIC_CONBUS_DATA for backward compatibility
                const {id, data} = msg;
                let response = id >= 1100 && id < 1200 ? createConbusReadResponse(id, data) :
                    id >= 1300 && id < 1400 ? createConbusWriteResponse(id, data) : null;
                if (!response || !(response.id in conbusDevices)) break;

                if (!(response.id in conbus)) conbus[response.id] = {};
                conbus[response.id][response.address] = response.data;

                updateConbusUI(response);
                break;
            }
            case TOPIC_SAFETY_LIGHTS: {
                const {state} = msg;
                if ($("#var_safety_lights").length) {
                    $("#var_safety_lights").text(state ? "On" : "Off");
                }
                break;
            }
            case TOPIC_PERFORMANCE: {
                const {fps, latency} = msg;
                if ($("#var_performance_fps").length) {
                    $("#var_performance_fps").text(`${formatToFixed(fps, 2)}`);
                }
                if ($("#var_performance_latency").length) {
                    $("#var_performance_latency").text(`${formatToFixed(latency, 2)} ms`);
                }
                break;
            }

            // PID and Motor Statistics
            case TOPIC_LINEAR_PID_STATISTICS: {
                const {error, p_term, i_term, d_term, output} = msg;
                if ($("#var_linear_pid_error").length) {
                    $("#var_linear_pid_error").text(formatToFixed(error, 4));
                }
                if ($("#var_linear_pid_output").length) {
                    $("#var_linear_pid_output").text(formatToFixed(output, 4));
                }
                break;
            }
            case TOPIC_ANGULAR_PID_STATISTICS: {
                const {error, p_term, i_term, d_term, output} = msg;
                if ($("#var_angular_pid_error").length) {
                    $("#var_angular_pid_error").text(formatToFixed(error, 4));
                }
                if ($("#var_angular_pid_output").length) {
                    $("#var_angular_pid_output").text(formatToFixed(output, 4));
                }
                break;
            }
            case TOPIC_MOTOR_STATISTICS_FRONT: {
                console.log("Front motor statistics received:", msg);
                $("#var_motor_statistics_front").text(JSON.stringify(msg));
                break;
            }
            case TOPIC_MOTOR_STATISTICS_BACK: {
                console.log("Back motor statistics received:", msg);
                $("#var_motor_statistics_back").text(JSON.stringify(msg));
                break;
            }
            case TOPIC_CAN_STATS: {
                console.log("CAN statistics received:", msg);
                $("#var_can_stats").text(JSON.stringify(msg));
                break;
            }
            case TOPIC_ZERO_ENCODERS: {
                console.log("Zero encoders message received:", msg);
                $("#var_zero_encoders").text(JSON.stringify(msg));
                break;
            }

            // Camera Data
            case TOPIC_RAW_LEFT: {
                transferImageToElementByClass("target_raw_camera_left", msg.data);
                break;
            }
            case TOPIC_RAW_RIGHT: {
                transferImageToElementByClass("target_raw_camera_right", msg.data);
                break;
            }
            case TOPIC_RAW_FRONT: {
                transferImageToElementByClass("target_raw_camera_front", msg.data);
                break;
            }
            case TOPIC_RAW_BACK: {
                transferImageToElementByClass("target_raw_camera_back", msg.data);
                break;
            }
            case TOPIC_COMBINED_IMAGE: {
                transferImageToElementByClass("target_combined", msg.data);
                break;
            }
            case TOPIC_FEELERS: {
                transferImageToElementByClass("target_feelers", msg.data);
                break;
            }

            // Configuration
            case TOPIC_CONFIGURATION_BROADCAST: {
                $("#var_configuration_broadcast").text(JSON.stringify(msg));
                break;
            }
            case TOPIC_CONFIGURATION_UPDATE: {
                $("#var_configuration_update").text(JSON.stringify(msg));
                break;
            }
            case TOPIC_CONFIG_PRESTS_LOAD: {
                $("#var_config_presets_load").text(JSON.stringify(msg));
                break;
            }
            case TOPIC_CONFIG_PRESTS_SAVE: {
                $("#var_config_presets_save").text(JSON.stringify(msg));
                break;
            }
            case TOPIC_CONFIGURATION: {
                // Legacy configuration topic
                const {device, json} = msg;
                config[device] = JSON.parse(json);
                regenerateConfig();

                updateConfigurationUI(device, config[device]);
                break;
            }

            case "get_configuration_response": {
                const {device, config: deviceConfig} = msg;
                if (deviceConfig) {
                    updateConfigurationUI(device, deviceConfig);
                }
                break;
            }

            // Others
            case TOPIC_PLAYBACK: {
                logs.push({message: msg.data, node: msg.node, timestamp: new Date()});
                if (logs.length > 30) logs.shift();

                const logElement = $("#log_body");
                logElement.empty();
                for (let i = logs.length - 1; i >= 0; i--) {
                    const log = logs[i];
                    const tableEntry = $("<tr></tr>");
                    tableEntry.append(`<td>${log.timestamp.toTimeString().split(" ")[0]}</td>`);
                    tableEntry.append(`<td>${log.node}</td>`);
                    tableEntry.append(`<td>${log.message}</td>`);
                    logElement.append(tableEntry);
                }
                break;
            }
            case TOPIC_AUDIBLE_FEEDBACK: {
                console.log("Audible feedback received:", msg);
                $("#var_audible_feedback").text(JSON.stringify(msg));
                break;
            }
            default:
                break;
        }
    }

    ////////////////////////////////// Helpers //////////////////////////////////
    //feature p4 5/11/2024 Most of these are stubs
    $(".dropdown-menu a").on("click", function () {
        const parentDataTarget = $(this).parents(".dropdown").attr("data-target");
        console.log(parentDataTarget);
        if (parentDataTarget === "system_state") {
            const id = $(this).attr("data-value");
            systemState.state = parseInt(id);
            setSystemState();
        } else if (parentDataTarget === "system_mode") {
            const id = $(this).attr("data-value");
            systemState.mode = parseInt(id);
            setSystemState();
        } else if (parentDataTarget === "theme") {
            const id = $(this).attr("data-value");
            preferences.theme = id;
            savePreferences();
            $("html").attr("data-bs-theme", id);
        } else if (parentDataTarget === "gpsformat") {
            preferences.gpsFormat = $(this).attr("data-value");
            savePreferences();
        }
    });

    $("#save_preset_mode").on("click", function () {
        if (!current_preset || current_preset === "None") {
            ntf('No preset selected', 'error');
            return;
        }

        // Save the current configuration to the active preset in localStorage
        const presets = JSON.parse(localStorage.getItem("presets") || "{}");
        presets[current_preset] = config;
        localStorage.setItem("presets", JSON.stringify(presets));

        ntf(`Preset "${current_preset}" updated successfully`, 'success');

        // Also send to server if it supports it
        send({
            op: "save_preset_mode"
        });
        send({op: "get_presets"});
    });

    $("#save_preset_as").on("click", function () {
        const preset_name = $("#preset_save_name").val();
        if (!preset_name) {
            ntf('Please enter a preset name', 'error');
            return;
        }

        // Save the current configuration to localStorage
        const presets = JSON.parse(localStorage.getItem("presets") || "{}");
        presets[preset_name] = config;
        localStorage.setItem("presets", JSON.stringify(presets));

        // Update the current preset
        current_preset = preset_name;
        $("#active_preset_value").text(current_preset);

        // Update the dropdown menu
        updatePresetsDropdown();

        // Clear the input field
        $("#preset_save_name").val("");

        ntf('Preset saved successfully', 'success');

        // Also send to server if it supports it
        send({
            op: "save_preset_as",
            preset: preset_name
        });
    });

    // Function to update the presets dropdown menu
    function updatePresetsDropdown() {
        const presets = JSON.parse(localStorage.getItem("presets") || "{}");
        const presetElement = $("#dropdown_elements");
        presetElement.empty();

        // Add presets from localStorage
        for (const preset in presets) {
            const dropdownItem = $(`<li><a class="dropdown-item" data-value="${preset}">${preset}</a></li>`);
            presetElement.append(dropdownItem);

            dropdownItem.on("click", function() {
                loadPreset(preset);
            });
        }

        // If no presets, add a message
        if (Object.keys(presets).length === 0) {
            presetElement.append('<li><a class="dropdown-item disabled">No presets available</a></li>');
        }
    }

    // Function to load a preset
    function loadPreset(presetName) {
        const presets = JSON.parse(localStorage.getItem("presets") || "{}");
        if (presets[presetName]) {
            // Update the current configuration with the preset values
            config = presets[presetName];

            // Update the UI with the new configuration values
            for (const device in config) {
                updateConfigurationUI(device, config[device]);
            }

            // Update the current preset
            current_preset = presetName;
            $("#active_preset_value").text(current_preset);

            ntf(`Preset "${presetName}" loaded successfully`, 'success');

            // Also send to server if it supports it
            send({
                op: "set_active_preset",
                preset: presetName
            });
        } else {
            ntf(`Preset "${presetName}" not found`, 'error');
        }
    }

    // The presets dropdown will be initialized when the server responds to the get_presets request
    // Add a fallback to initialize the presets dropdown with localStorage presets if the server doesn't respond
    setTimeout(() => {
        // If the dropdown is empty, initialize it with localStorage presets
        if ($("#dropdown_elements").children().length === 0) {
            const localPresets = JSON.parse(localStorage.getItem("presets") || "{}");
            const presetElement = $("#dropdown_elements");
            presetElement.empty();

            // Add localStorage presets
            for (const preset in localPresets) {
                const dropdownItem = $(`<li><a class="dropdown-item" data-value="${preset}">${preset}</a></li>`);
                presetElement.append(dropdownItem);

                dropdownItem.on("click", function() {
                    const preset_name = $(this).children().attr("data-value");
                    loadPreset(preset_name);
                });
            }

            // If no presets, add a message
            if (Object.keys(localPresets).length === 0) {
                presetElement.append('<li><a class="dropdown-item disabled">No presets available</a></li>');
            }
        }
    }, 2000); // Wait 2 seconds for the server to respond

    $("#delete_preset").on("click", function () {
        if (!current_preset || current_preset === "None") {
            ntf('No preset selected', 'error');
            return;
        }

        // Remove the preset from localStorage
        const presets = JSON.parse(localStorage.getItem("presets") || "{}");
        if (presets[current_preset]) {
            delete presets[current_preset];
            localStorage.setItem("presets", JSON.stringify(presets));

            // Update the dropdown menu
            updatePresetsDropdown();

            // Reset the current preset
            current_preset = "None";
            $("#active_preset_value").text(current_preset);

            ntf(`Preset "${current_preset}" deleted successfully`, 'success');
        }

        // Also send to server if it supports it
        send({
            op: "delete_preset",
            preset: current_preset
        });
    });

    $("#checkbox_system_mobility").on("change", function () {
        systemState.mobility = $(this).is(":checked");
        setSystemState();
    });

    $("#input_port, #input_host").on("change", function () {
        switch (this.id) {
            case "input_port":
                const intt = parseInt($(this).val());
                preferences.port = isNaN(intt) ? 8080 : intt;

                if (/\D/.test($(this).val())) {//check for non-integer vals
                    $(this).val(8080);
                    ntf('Port must be an integer, assigned to default 8023', 'error');
                    console.log("Port must be an integer, assigned to default 8023. Delete following if statement " +
                        "to unforce this: if (/\\D/.test($(this).val())) {//check for non-integer vals");
                }
                break;
            case "input_host"://some IPs may have characters so no need 2 check for it
                preferences.host = $(this).val();
                break;
        }

        savePreferences();
    });


    $("clear_log").on("click", function () {
        logs = [];
        $("#log_body").empty();
    });

    //End of notFinished stubs feature

    function generateElementForConfiguration(data, type, device, text) {
        if (type === "bool") {
            const checked = data === 1;

            // Create a dropdown
            const div = document.createElement("div");
            div.classList.add("input-group");
            div.classList.add("mb-3");

            const select = document.createElement("select");
            select.classList.add("form-select");
            select.onchange = function () {
                config[device][text] = select.value === "1";
                send({
                    op: "configuration",
                    device: device,
                    json: config[device],
                });
            }

            const optionTrue = document.createElement("option");
            optionTrue.value = 1;
            optionTrue.innerText = "True";
            optionTrue.selected = checked;

            const optionFalse = document.createElement("option");
            optionFalse.value = 0;
            optionFalse.innerText = "False";
            optionFalse.selected = !checked;

            select.appendChild(optionTrue);
            select.appendChild(optionFalse);

            const span = document.createElement("span");
            span.classList.add("input-group-text");
            span.innerText = text;

            div.appendChild(span);
            div.appendChild(select);
            return div;
        } else if (type === "float") {
            const div = document.createElement("div");
            div.classList.add("input-group");
            div.classList.add("mb-3");

            const input = document.createElement("input");
            input.type = "number";
            input.classList.add("form-control");
            input.value = data;
            input.onchange = function () {
                config[device][text] = parseFloat(input.value);
                send({
                    op: "configuration",
                    device: device,
                    json: config[device],
                });
            }

            const span = document.createElement("span");
            span.classList.add("input-group-text");
            span.innerText = text;

            div.appendChild(span);
            div.appendChild(input);
            return div;
        } else if (type === "int") {
            const div = document.createElement("div");
            div.classList.add("input-group");
            div.classList.add("mb-3");

            const input = document.createElement("input");
            input.type = "number";
            input.classList.add("form-control");
            input.value = data;
            input.onchange = function () {
                config[device][text] = parseInt(input.value);
                send({
                    op: "configuration",
                    device: device,
                    json: config[device],
                });
            }

            const span = document.createElement("span");
            span.classList.add("input-group-text");
            span.innerText = text;

            div.appendChild(span);
            div.appendChild(input);
            return div;
        } else if (type === "point.int") {
            // x, y point for two integers
            const div = document.createElement("div");
            div.classList.add("input-group");
            div.classList.add("mb-3");

            const inputX = document.createElement("input");
            inputX.type = "number";
            inputX.classList.add("form-control");
            inputX.value = data[0];
            inputX.onchange = function () {
                config[device][text][0] = parseInt(inputX.value);
                send({
                    op: "configuration",
                    device: device,
                    json: config[device],
                });
            }

            const inputY = document.createElement("input");
            inputY.type = "number";
            inputY.classList.add("form-control");
            inputY.value = data[1];
            inputY.onchange = function () {
                config[device][text][1] = parseInt(inputY.value);
                send({
                    op: "configuration",
                    device: device,
                    json: config[device],
                });
            }

            const span = document.createElement("span");
            span.classList.add("input-group-text");
            span.innerText = text;

            div.appendChild(span);
            div.appendChild(inputX);
            div.appendChild(inputY);
            return div;
        } else if (type === "parallelogram.int") {
            const div = document.createElement("div");
            div.classList.add("input-group", "mb-3");

            function createCoordinateInput(value, onChangeHandler) {
                const input = document.createElement("input");
                input.type = "number";
                input.classList.add("form-control", "coordinate-input");
                input.value = value;
                input.onchange = onChangeHandler;
                return input;
            }

            const inputX1 = createCoordinateInput(data[0][0], function () {
                config[device][text][0][0] = parseInt(inputX1.value);
                send({
                    op: "configuration",
                    device: device,
                    json: config[device],
                });
            });

            const inputY1 = createCoordinateInput(data[0][1], function () {
                config[device][text][0][1] = parseInt(inputY1.value);
                send({
                    op: "configuration",
                    device: device,
                    json: config[device],
                });
            });

            const inputX2 = createCoordinateInput(data[1][0], function () {
                config[device][text][1][0] = parseInt(inputX2.value);
                send({
                    op: "configuration",
                    device: device,
                    json: config[device],
                });
            });

            const inputY2 = createCoordinateInput(data[1][1], function () {
                config[device][text][1][1] = parseInt(inputY2.value);
                send({
                    op: "configuration",
                    device: device,
                    json: config[device],
                });
            });

            const inputX3 = createCoordinateInput(data[2][0], function () {
                config[device][text][2][0] = parseInt(inputX3.value);
                send({
                    op: "configuration",
                    device: device,
                    json: config[device],
                });
            });

            const inputY3 = createCoordinateInput(data[2][1], function () {
                config[device][text][2][1] = parseInt(inputY3.value);
                send({
                    op: "configuration",
                    device: device,
                    json: config[device],
                });
            });

            const inputX4 = createCoordinateInput(data[3][0], function () {
                config[device][text][3][0] = parseInt(inputX4.value);
                send({
                    op: "configuration",
                    device: device,
                    json: config[device],
                });
            });

            const inputY4 = createCoordinateInput(data[3][1], function () {
                config[device][text][3][1] = parseInt(inputY4.value);
                send({
                    op: "configuration",
                    device: device,
                    json: config[device],
                });
            });

            const span = document.createElement("span");
            span.classList.add("input-group-text");
            span.innerText = text;

            div.appendChild(span);
            div.appendChild(inputX1);
            div.appendChild(inputY1);
            div.appendChild(inputX2);
            div.appendChild(inputY2);
            div.appendChild(inputX3);
            div.appendChild(inputY3);
            div.appendChild(inputX4);
            div.appendChild(inputY4);
            return div;
        } else if (type === "waypoints") {
            const div = document.createElement("div");
            div.classList.add("input-group");
            div.classList.add("mb-3");

            const waypointsContainer = document.createElement("div");
            waypointsContainer.classList.add("waypoints-container");

            // Display existing waypoints
            if (data && Array.isArray(data)) {
                data.forEach((waypoint, index) => {
                    const waypointRow = document.createElement("div");
                    waypointRow.classList.add("waypoint-row", "d-flex", "mb-2");

                    const latInput = document.createElement("input");
                    latInput.type = "number";
                    latInput.classList.add("form-control", "me-2");
                    latInput.value = waypoint[0];
                    latInput.placeholder = "Latitude";
                    latInput.step = "0.000001";

                    const lngInput = document.createElement("input");
                    lngInput.type = "number";
                    lngInput.classList.add("form-control", "me-2");
                    lngInput.value = waypoint[1];
                    lngInput.placeholder = "Longitude";
                    lngInput.step = "0.000001";

                    const removeBtn = document.createElement("button");
                    removeBtn.classList.add("btn", "btn-danger");
                    removeBtn.innerText = "Remove";
                    removeBtn.onclick = function() {
                        waypointRow.remove();
                        // Update config
                        const newWaypoints = [];
                        waypointsContainer.querySelectorAll('.waypoint-row').forEach(row => {
                            const lat = parseFloat(row.querySelector('input:nth-child(1)').value);
                            const lng = parseFloat(row.querySelector('input:nth-child(2)').value);
                            newWaypoints.push([lat, lng]);
                        });
                        config[device][text] = newWaypoints;
                        send({
                            op: "configuration",
                            device: device,
                            json: config[device],
                        });
                    };

                    waypointRow.appendChild(latInput);
                    waypointRow.appendChild(lngInput);
                    waypointRow.appendChild(removeBtn);
                    waypointsContainer.appendChild(waypointRow);
                });
            }

            // Add button to add new waypoint
            const addBtn = document.createElement("button");
            addBtn.classList.add("btn", "btn-primary", "mt-2");
            addBtn.innerText = "Add Waypoint";
            addBtn.onclick = function() {
                const waypointRow = document.createElement("div");
                waypointRow.classList.add("waypoint-row", "d-flex", "mb-2");

                const latInput = document.createElement("input");
                latInput.type = "number";
                latInput.classList.add("form-control", "me-2");
                latInput.placeholder = "Latitude";
                latInput.step = "0.000001";

                const lngInput = document.createElement("input");
                lngInput.type = "number";
                lngInput.classList.add("form-control", "me-2");
                lngInput.placeholder = "Longitude";
                lngInput.step = "0.000001";

                const removeBtn = document.createElement("button");
                removeBtn.classList.add("btn", "btn-danger");
                removeBtn.innerText = "Remove";
                removeBtn.onclick = function() {
                    waypointRow.remove();
                    // Update config
                    const newWaypoints = [];
                    waypointsContainer.querySelectorAll('.waypoint-row').forEach(row => {
                        const lat = parseFloat(row.querySelector('input:nth-child(1)').value);
                        const lng = parseFloat(row.querySelector('input:nth-child(2)').value);
                        newWaypoints.push([lat, lng]);
                    });
                    config[device][text] = newWaypoints;
                    send({
                        op: "configuration",
                        device: device,
                        json: config[device],
                    });
                };

                waypointRow.appendChild(latInput);
                waypointRow.appendChild(lngInput);
                waypointRow.appendChild(removeBtn);
                waypointsContainer.appendChild(waypointRow);

                // Update inputs to trigger change events
                latInput.onchange = lngInput.onchange = function() {
                    const newWaypoints = [];
                    waypointsContainer.querySelectorAll('.waypoint-row').forEach(row => {
                        const lat = parseFloat(row.querySelector('input:nth-child(1)').value);
                        const lng = parseFloat(row.querySelector('input:nth-child(2)').value);
                        newWaypoints.push([lat, lng]);
                    });
                    config[device][text] = newWaypoints;
                    send({
                        op: "configuration",
                        device: device,
                        json: config[device],
                    });
                };
            };

            const span = document.createElement("span");
            span.classList.add("input-group-text");
            span.innerText = text;

            div.appendChild(span);
            div.appendChild(waypointsContainer);
            div.appendChild(addBtn);
            return div;
        } else {
            const options = addressKeys[device][text];
            if (typeof options == "object") {
                const index = data;

                // Create a dropdown
                const div = document.createElement("div");
                div.classList.add("input-group");
                div.classList.add("mb-3");

                const select = document.createElement("select");
                select.classList.add("form-select");
                select.onchange = function () {
                    config[device][text] = parseInt(select.value);
                    send({
                        op: "configuration",
                        device: device,
                        json: config[device],
                    });
                }

                for (let i = 0; i < Object.keys(options).length; i++) {
                    const option = document.createElement("option");
                    option.value = i;
                    option.selected = i === index;
                    option.innerText = options[i];
                    select.appendChild(option);
                }

                const span = document.createElement("span");
                span.classList.add("input-group-text");
                span.innerText = text;

                div.appendChild(span);
                div.appendChild(select);
                return div;
            }
        }
    }

    const regenerateConfig = () => {
        const configElement = $("#options");
        configElement.empty();

        // Sort the keys in each config by their addressKeys
        for (const deviceId in addressKeys) {
            if (!(deviceId in config)) {
                continue;
            }

            const title = addressKeys[deviceId]["internal_title"];
            const deviceElement = $(`<div class="card" style="margin-bottom: 10px;"></div>`);
            deviceElement.append(`<div class="card-header"><h5>${title}</h5></div>`);
            const deviceBody = $(`<div class="card-body"></div>`);
            deviceElement.append(deviceBody);

            const deviceConfig = config[deviceId];
            for (const address in addressKeys[deviceId]) {
                if (address === "internal_title") {
                    continue;
                }

                if (!(address in deviceConfig)) {
                    const alert = $(`<div class="alert alert-warning" role="alert">Key not found: ${address}</div>`);
                    deviceBody.append(alert);
                    continue;
                }

                const data = deviceConfig[address];
                const type = addressKeys[deviceId][address];
                const inputElement = generateElementForConfiguration(data, type, deviceId, address);
                deviceBody.append(inputElement);
            }

            configElement.append(deviceElement);
        }

        // config = outputConfig;
        // for (const deviceId in config) {
        //     const deviceConfig = config[deviceId];
        //     if (addressKeys[deviceId] == undefined) {
        //         console.log(`Unknown Device Config: ${deviceId}`);
        //         // const alert = $(`<div class="alert alert-danger" role="alert">Unknown Device Config: ${deviceId}</div>`);
        //         // configElement.append(alert);
        //         continue;
        //     }

        //     const title = addressKeys[deviceId]["internal_title"];
        //     const deviceElement = $(`<div class="card" style="margin-bottom: 10px;"></div>`);
        //     deviceElement.append(`<div class="card-header"><h5>${title}</h5></div>`);
        //     const deviceBody = $(`<div class="card-body"></div>`);
        //     deviceElement.append(deviceBody);

        //     for (const address of Object.keys(deviceConfig).sort()) {
        //         const data = deviceConfig[address];
        //         const type = addressKeys[deviceId][address];
        //         if (type == undefined) {
        //             const alert = $(`<div class="alert alert-warning" role="alert">Unknown Type: ${address}</div>`);
        //             deviceBody.append(alert);
        //             continue;
        //         }

        //         const inputElement = generateElementForConfiguration(data, type, deviceId, address);
        //         deviceBody.append(inputElement);
        //     }

        //     for (const address in addressKeys[deviceId]) {
        //         if (address in deviceConfig || address == "internal_title") {
        //             continue;
        //         }

        //         const alert = $(`<div class="alert alert-danger" role="alert">Unknown Configuration Entry: ${address}</div>`);
        //         deviceBody.append(alert);
        //     }

        //     configElement.append(deviceElement);
        // }
    }

    function send(obj) {
        sendQueue.push(obj);
    }

    function updateConfigurationUI(device, deviceConfig) {
        console.log("Updating UI for device:", device, deviceConfig);

        switch(device) {
            case "autonav_vision_transformer":
                // Update Vision Transformer UI
                $("#region_of_disinterest_offset").val(deviceConfig.region_of_disinterest_offset || 0);
                $("#lower_hue").val(deviceConfig.lower_hue || 0);
                $("#lower_saturation").val(deviceConfig.lower_saturation || 0);
                $("#lower_value").val(deviceConfig.lower_value || 0);
                $("#upper_hue").val(deviceConfig.upper_hue || 0);
                $("#upper_saturation").val(deviceConfig.upper_saturation || 0);
                $("#upper_value").val(deviceConfig.upper_value || 0);
                $("#blur").val(deviceConfig.blur || 0);
                $("#blur_iterations").val(deviceConfig.blur_iterations || 0);
                break;

            case "autonav_nav_astar":
                // Update A* Navigation UI
                $("#latitude_length").val(deviceConfig.latitude_length || 111086.2);
                $("#longitude_length").val(deviceConfig.longitude_length || 91978.2);
                $("#waypoint_pop_distance").val(deviceConfig.waypoint_pop_distance || 1.0);
                $("#waypoint_delay").val(deviceConfig.waypoint_delay || 10);
                $("#robot_y").val(deviceConfig.robot_y || 70);
                $("#use_only_waypoints").prop("checked", deviceConfig.use_only_waypoints || false);
                break;

            case "zemlin_path_resolver":
                // Update Path Resolver UI
                $("#forward_speed").val(deviceConfig.forward_speed || 0.5);
                $("#reverse_speed").val(deviceConfig.reverse_speed || -0.5);
                $("#angular_aggressiveness").val(deviceConfig.angular_aggressiveness || 1.0);
                $("#max_angular_speed").val(deviceConfig.max_angular_speed || 1.0);
                $("#radius_multiplier").val(deviceConfig.radius_multiplier || 1.0);
                $("#radius_max").val(deviceConfig.radius_max || 1.0);
                $("#radius_start").val(deviceConfig.radius_start || 0.5);
                break;

            case "zemlin_filters":
                // Update Position Filters UI
                $("#filter_latitude_length").val(deviceConfig.latitude_length || 111086.2);
                $("#filter_longitude_length").val(deviceConfig.longitude_length || 91978.2);
                $("#filter_type").val(deviceConfig.filter_type || 0);
                break;

            default:
                console.log("Unknown device:", device);
                break;
        }
    }

    // Event handlers for configuration page buttons

    // Vision Transformer
    // Added -c event handlers for Vision Transformer inps
    $("#region_of_disinterest_offset, #lower_hue, #lower_saturation, #lower_value, #upper_hue, #upper_saturation, #upper_value, #blur, #blur_iterations").on("change", function() {
        const configData = {
            region_of_disinterest_offset: parseInt($("#region_of_disinterest_offset").val()) || 0,
            lower_hue: parseInt($("#lower_hue").val()) || 0,
            lower_saturation: parseInt($("#lower_saturation").val()) || 0,
            lower_value: parseInt($("#lower_value").val()) || 0,
            upper_hue: parseInt($("#upper_hue").val()) || 0,
            upper_saturation: parseInt($("#upper_saturation").val()) || 0,
            upper_value: parseInt($("#upper_value").val()) || 0,
            blur: parseInt($("#blur").val()) || 0,
            blur_iterations: parseInt($("#blur_iterations").val()) || 0
        };

        send({
            op: "configuration",
            device: "autonav_vision_transformer",
            json: JSON.stringify(configData)
        });

        ntf('Vision Transformer configuration saved', 'success');
    });

    $("#reset_vision_transformer").on("click", function() {
        // Reset to default values or fetch from server
        send({
            op: "get_configuration",
            device: "autonav_vision_transformer"
        });

        ntf('Vision Transformer configuration reset', 'info');
    });

    // A* Navigation
    // Add change event handlers to all A* Navigation inputs
    $("#latitude_length, #longitude_length, #waypoint_pop_distance, #waypoint_delay, #robot_y").on("change", function() {
        const configData = {
            latitude_length: parseFloat($("#latitude_length").val()) || 111086.2,
            longitude_length: parseFloat($("#longitude_length").val()) || 91978.2,
            waypoint_pop_distance: parseFloat($("#waypoint_pop_distance").val()) || 1.0,
            waypoint_delay: parseFloat($("#waypoint_delay").val()) || 10,
            robot_y: parseInt($("#robot_y").val()) || 70,
            use_only_waypoints: $("#use_only_waypoints").is(":checked")
        };

        send({
            op: "configuration",
            device: "autonav_nav_astar",
            json: JSON.stringify(configData)
        });

        ntf('A* Navigation configuration saved', 'success');
    });

    // Add change event handler for checkbox
    $("#use_only_waypoints").on("change", function() {
        const configData = {
            latitude_length: parseFloat($("#latitude_length").val()) || 111086.2,
            longitude_length: parseFloat($("#longitude_length").val()) || 91978.2,
            waypoint_pop_distance: parseFloat($("#waypoint_pop_distance").val()) || 1.0,
            waypoint_delay: parseFloat($("#waypoint_delay").val()) || 10,
            robot_y: parseInt($("#robot_y").val()) || 70,
            use_only_waypoints: $(this).is(":checked")
        };

        send({
            op: "configuration",
            device: "autonav_nav_astar",
            json: JSON.stringify(configData)
        });

        ntf('A* Navigation configuration saved', 'success');
    });

    $("#reset_astar_navigation").on("click", function() {
        // Reset to default values or fetch from server
        send({
            op: "get_configuration",
            device: "autonav_nav_astar"
        });

        ntf('A* Navigation configuration reset', 'info');
    });

    // Path Resolver
    // Add change event handlers to all Path Resolver inputs
    $("#forward_speed, #reverse_speed, #angular_aggressiveness, #max_angular_speed, #radius_multiplier, #radius_max, #radius_start").on("change", function() {
        const configData = {
            forward_speed: parseFloat($("#forward_speed").val()) || 0.5,
            reverse_speed: parseFloat($("#reverse_speed").val()) || -0.5,
            angular_aggressiveness: parseFloat($("#angular_aggressiveness").val()) || 1.0,
            max_angular_speed: parseFloat($("#max_angular_speed").val()) || 1.0,
            radius_multiplier: parseFloat($("#radius_multiplier").val()) || 1.0,
            radius_max: parseFloat($("#radius_max").val()) || 1.0,
            radius_start: parseFloat($("#radius_start").val()) || 0.5
        };

        send({
            op: "configuration",
            device: "zemlin_path_resolver",
            json: JSON.stringify(configData)
        });

        ntf('Path Resolver configuration saved', 'success');
    });

    $("#reset_path_resolver").on("click", function() {
        // Reset to default values or fetch from server
        send({
            op: "get_configuration",
            device: "zemlin_path_resolver"
        });

        ntf('Path Resolver configuration reset', 'info');
    });

    // Position Filters
    // Add change event handlers to all Position Filters inputs
    $("#filter_latitude_length, #filter_longitude_length, #filter_type").on("change", function() {
        const configData = {
            latitude_length: parseFloat($("#filter_latitude_length").val()) || 111086.2,
            longitude_length: parseFloat($("#filter_longitude_length").val()) || 91978.2,
            filter_type: parseInt($("#filter_type").val()) || 0
        };

        send({
            op: "configuration",
            device: "zemlin_filters",
            json: JSON.stringify(configData)
        });

        ntf('Position Filters configuration saved', 'success');
    });

    $("#reset_position_filters").on("click", function() {
        // Reset to default values or fetch from server
        send({
            op: "get_configuration",
            device: "zemlin_filters"
        });

        ntf('Position Filters configuration reset', 'info');
    });
})
//Old function meant to toggle dev mode with button press
/*
document.getElementById('toggle_dev_mode').addEventListener('click', function () {
    development_mode = !development_mode;
    console.log('Development mode:', development_mode);
});
*/
