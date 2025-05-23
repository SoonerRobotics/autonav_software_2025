/**
 $(document).ready(function () {
 // Check if local storage has preferences
 if (localStorage.getItem("preferences") == null) {
 savePreferences();
 } else {
 preferences = JSON.parse(localStorage.getItem("preferences"));

 $("#input_host").val(preferences.host);
 $("#input_port").val(preferences.port);

 $("html").attr("data-bs-theme", preferences.theme);
 }


 ////////////////////////////////// Websocekt //////////////////////////////////

 let websocket;
 const createWebsocket = () => {
 const userID = generateUUID();
 const url = `ws://${preferences.host}:${preferences.port}/?id=${userID}`
 websocket = new WebSocket(url);

 websocket.onopen = function (event) {
 $("#connecting-state").text("Updating Data");
 $(".connecting-input").hide();

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
 $(".connecting").hide();
 $("#main").show();
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

 if (op === "get_presets_callback") {
 const presets = obj.presets;
 const presetElement = $("#dropdown_elements");
 presetElement.empty();
 for (const preset of presets) {
 const dropdownItem = $(`<li><a class="dropdown-item" data-value="${preset}">${preset}</a></li>`);
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

 current_preset = obj.active_preset;
 $("#active_preset_value").text(current_preset);
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

 for (const key in obj.configs) {
 config[key] = obj.configs[key];
 }
 regenerateConfig();

 // Update system state
 let system = obj["system"];
 $("#var_system_state").text(system["state"] === 0 ? "Diabled" : system["state"] === 1 ? "Autonomous" : system["state"] === 2 ? "Manual" : "Shutdown");
 $("#var_system_mode").text(system["mode"] === 0 ? "Competition" : system["mode"] === 1 ? "Simulation" : "Practice");
 $("#var_system_mobility").text(system["mobility"] ? "Enabled" : "Disabled");

 // Update some buttons
 $("#checkbox_system_mobility").prop("checked", system["mobility"]);
 $("#input_system_state").val(system["state"]);
 }
 }
 };

 websocket.onclose = function (event) {
 $("#connecting-state").text("Waiting for the Danger Zone");
 $(".connecting").show();
 $(".connecting-input").show();
 $("#main").hide();
 clearGlobals();

 setTimeout(() => {
 // createWebsocket();
 location.reload();
 }, 2000);
 };

 websocket.onerror = function (event) {
 console.error(event);
 };
 }

 if (!development_mode) {
 createWebsocket();
 } else {
 $("#connecting-state").text("Updating Data");
 $(".connecting-input").hide();
 $(".connecting").hide();
 $("#main").show();
 }

 var sendQueue = [];

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

 if (topic === "/scr/state/system") {
 const {state, mode, mobility} = msg;

 $("#var_system_state").text(state === 0 ? "Diabled" : state === 1 ? "Autonomous" : state === 2 ? "Manual" : "Shutdown");
 $("#var_system_mode").text(mode === 0 ? "Competition" : mode === 1 ? "Simulation" : "Practice");
 $("#var_system_mobility").text(mobility ? "Enabled" : "Disabled");

 systemState.state = state;
 systemState.mode = mode;
 systemState.mobility = mobility;

 $("#input_system_state").val(state);
 $("#input_system_mode").val(mode);
 $("#input_system_mobility").prop("checked", mobility);
 return;
 }

 if (topic === "/scr/state/device") {
 const {device, state} = msg;

 deviceStates[device] = state;
 unorderedListElement = $("#element_device_states");
 unorderedListElement.empty();
 for (const id in deviceStates) {
 const state = deviceStates[id];
 unorderedListElement.append(`<h5>${id}: <span data-state=\"${state}\">${deviceStateToName(state)}</span></h5>`);
 }
 return;
 }

 if (topic === "/scr/configuration") {
 const {device, json} = msg;
 config[device] = JSON.parse(json);
 regenerateConfig();
 return;
 }

 if (topic === "/scr/logging") {
 logs.push({message: msg.data, node: msg.node, timestamp: new Date()});
 if (logs.length > 30) {
 logs.shift();
 }

 const logElement = $("#log_body");
 logElement.empty();
 for (let i = logs.length - 1; i >= 0; i--) {
 const log = logs[i];
 const tableEntry = $(`<tr></tr>`);
 // Format as: HH:MM:SS
 tableEntry.append(`<td>${log.timestamp.toTimeString().split(" ")[0]}</td>`);
 tableEntry.append(`<td>${log.node}</td>`);
 tableEntry.append(`<td>${log.message}</td>`);
 logElement.append(tableEntry);
 }
 return;
 }

 if (topic === "/autonav/gps") {
 const {latitude, longitude, gps_fix, is_locked, satellites} = msg;
 $("#var_gps_position").text(formatLatLong(latitude, longitude, true));
 $("#var_gps_fix").text(gps_fix);
 $("#var_gps_fixed").text(is_locked ? "Locked" : "Not Locked");
 $("#var_gps_satellites").text(satellites);
 return;
 }

 if (topic === "/autonav/MotorFeedback") {
 const {delta_x, delta_y, delta_theta} = msg;
 $("#var_motors_feedback").text(`(${formatToFixed(delta_x, 4)}, ${formatToFixed(delta_y, 4)}, ${formatToFixed(delta_theta, 4)}°)`);
 return;
 }

 if (topic === "/autonav/MotorInput") {
 const {forward_velocity, angular_velocity} = msg;
 $("#var_motors_velocity").text(`(${formatToFixed(forward_velocity, 3)}, ${formatToFixed(angular_velocity, 3)})`);
 return;
 }

 if (topic === "/autonav/debug/astar") {
 const {
 desired_heading,
 desired_latitude,
 desired_longitude,
 distance_to_destination,
 waypoints,
 time_until_use_waypoints
 } = msg;
 $("#var_astar_heading").text(`${radiansToDegrees(parseFloat(desired_heading)).toFixed(3)}°`);
 $("#var_astar_waypoint").text(formatLatLong(desired_latitude, desired_longitude, true));
 $("#var_astar_distance").text(formatToFixed(distance_to_destination, 3));
 $("#var_astar_waypoints").text(
 waypoints.reduce((acc, val, i) => {
 if (i % 2 === 0) {
 acc.push([val, waypoints[i + 1]]);
 }

 return acc;
 }, []).map((waypoint) => {
 return formatLatLong(waypoint[0], waypoint[1], true);
 }).join(", ")
 );
 $("#var_astar_time").text(formatToFixed(time_until_use_waypoints, 3));
 return;
 }

 if (topic === "/autonav/position") {
 const {x, y, theta, latitude, longitude} = msg;
 $("#var_position_origin").text(`(${formatToFixed(x, 4)}, ${formatToFixed(y, 4)}, ${radiansToDegrees(parseFloat(theta)).toFixed(3)}°)`);
 $("#var_position_global").text(`(${formatToFixed(latitude, 8)}, ${formatToFixed(longitude, 8)})`);
 return;
 }

 if (topic === "/autonav/camera/compressed/left") {
 transferImageToElement("target_raw_camera_left", msg.data);
 return;
 }

 if (topic === "/autonav/camera/compressed/right") {
 transferImageToElement("target_raw_camera_right", msg.data);
 return;
 }

 if (topic === "/autonav/cfg_space/raw/image/left_small") {
 transferImageToElement("target_filtered_left", msg.data);
 return;
 }

 if (topic === "/autonav/cfg_space/raw/image/right_small") {
 transferImageToElement("target_filtered_right", msg.data);
 return;
 }

 if (topic === "/autonav/cfg_space/combined/image") {
 transferImageToElement("target_combined", msg.data);
 return;
 }

 if (topic === "/autonav/debug/astar/image") {
 transferImageToElement("target_astar", msg.data);
 return;
 }

 if (topic === "/autonav/imu") {
 const {accel_x, accel_y, accel_z, angular_x, angular_y, angular_z, yaw, pitch, roll} = msg;
 $("#var_imu_acceleration").text(`(${formatToFixed(accel_x, 4)}, ${formatToFixed(accel_y, 4)}, ${formatToFixed(accel_z, 4)})`);
 $("#var_imu_angular").text(`(${formatToFixed(angular_x, 4)}, ${formatToFixed(angular_y, 4)}, ${formatToFixed(angular_z, 4)})`);
 $("#var_imu_orientation").text(`(${radiansToDegrees(parseFloat(yaw)).toFixed(3)}°, ${radiansToDegrees(parseFloat(pitch)).toFixed(3)}°, ${radiansToDegrees(parseFloat(roll)).toFixed(3)}°)`);
 }

 if (topic === "/autonav/conbus") {
 const {id, data} = msg;
 let response;
 if (id >= 1100 && id < 1200) {
 response = createConbusReadResponse(id, data);
 if (!(response.id in conbusDevices)) {
 return;
 }
 } else if (id >= 1300 && id < 1400) {
 response = createConbusWriteResponse(id, data);
 if (!(response.id in conbusDevices)) {
 return;
 }
 } else {
 return;
 }

 if (!(response.id in conbus)) {
 conbus[response.id] = {};
 }
 conbus[response.id][response.address] = response.data;

 const conbusElement = $(`#conbus`);
 const conbusCard = $(`#conbus_${response.id}`);
 if (conbusCard !== undefined || conbusCard.length !== 0) {
 conbusCard.remove();
 }

 const card = $(`<div class="card" id="conbus_${response.id}" style="margin-bottom: 10px;"></div>`);
 card.append(`<div class="card-header"><h5>${conbusDevices[response.id].title}</h5></div>`);
 const cardBody = $(`<div class="card-body"></div>`);
 card.append(cardBody);

 for (const address in conbus[response.id]) {
 const data = conbus[response.id][address];
 if (!(address in conbusDevices[response.id].registers)) {
 const title = conbusDevices[response.id]?.registers?.[address]?.title ?? address.toString();
 const alert = $(`<div class="alert alert-danger" role="alert">Unknown Address: ${title}</div>`);
 cardBody.append(alert);
 continue;
 }
 const type = conbusDevices[response.id].registers[address].type;
 const title = conbusDevices[response.id].registers[address].title;
 const readonly = conbusDevices[response.id].registers[address].readonly || false;
 const inputElement = generateElementForConbus(data, type, title, response.id, address, readonly);
 cardBody.append(inputElement);
 }

 for (const address in conbusDevices[response.id].registers) {
 if (!(address in conbus[response.id])) {
 const title = conbusDevices[response.id].registers[address].title;
 const alert = $(`<div class="alert alert-warning" role="alert">Missing Address: ${title}</div>`);
 cardBody.append(alert);
 }
 }

 conbusElement.append(card);

 }
 }

 ////////////////////////////////// Helpers //////////////////////////////////
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
 send({
 op: "save_preset_mode"
 });
 send({op: "get_presets"});
 });

 $("#save_preset_as").on("click", function () {
 const preset_name = $("#preset_save_name").val();
 send({
 op: "save_preset_as",
 preset: preset_name
 });
 send({op: "get_presets"});
 $("#preset_save_name").val("");
 });

 $("#delete_preset").on("click", function () {
 send({
 op: "delete_preset",
 preset: current_preset
 });
 send({op: "get_presets"});
 });

 $("#checkbox_system_mobility").on("change", function () {
 systemState.mobility = $(this).is(":checked");
 setSystemState();
 });

 $("#input_port").on("change", function () {
 const intt = parseInt($(this).val());
 preferences.port = isNaN(intt) ? 8023 : intt;

 if (isNaN(intt)) {
 $(this).val(8023);
 }

 savePreferences();
 });

 $("#input_host").on("change", function () {
 preferences.host = $(this).val();

 savePreferences();
 });

 $("clear_log").on("click", function () {
 logs = [];
 $("#log_body").empty();
 });

 //End of stubs

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
 config[device][text] = select.value === 1 ? true : false;
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
 })
 */