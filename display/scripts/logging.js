let logFiles = [];
let currentLogFile = null;


function initLoggingPage() {

    $("#refresh_log_files").on("click", function () {
        requestLogFiles();
    });


    requestLogFiles();
}


function requestLogFiles() {
    $("#log_files_list").html('<div class="text-center"><p>Loading log files...</p></div>');

    send({
        op: "get_log_files"
    });
}

function handleLogFilesResponse(data) {
    logFiles = data.log_files;

    $("#log_files_list").empty();

    if (logFiles.length === 0) {
        $("#log_files_list").html('<div class="text-center"><p>No log files found</p></div>');
        return;
    }

    for (let i = 0; i < logFiles.length; i++) {
        const logFile = logFiles[i];
        const isZip = logFile.is_zip === true;

        const size = formatFileSize(logFile.size);

        const listItem = $(`
            <a href="#" class="list-group-item list-group-item-action" data-index="${i}">
                <div class="d-flex w-100 justify-content-between">
                    <h5 class="mb-1">${logFile.timestamp}</h5>
                    <small>${size}</small>
                </div>
                <p class="mb-1">Mode: ${logFile.mode}</p>
                <small>${isZip ? '(Zip file)' : 'Click to view'}</small>
            </a>
        `);


        listItem.on("click", function () {
            const index = $(this).data("index");
            selectLogFile(index);
        });


        $("#log_files_list").append(listItem);
    }
}

function selectLogFile(index) {

    $("#log_files_list .list-group-item").removeClass("active");
    $(`#log_files_list .list-group-item[data-index="${index}"]`).addClass("active");


    currentLogFile = logFiles[index];


    $("#log_file_title").text(`Log File: ${currentLogFile.timestamp} (${currentLogFile.mode})`);


    $("#log_file_content").html('<div class="text-center"><p>Loading content...</p></div>');


    if (currentLogFile.is_zip === true) {
        $("#log_file_content").html('<div class="alert alert-info">Zip files cannot be viewed directly. Please download and extract the file to view its contents.</div>');
        return;
    }


    send({
        op: "get_log_file_content",
        log_file_path: currentLogFile.path
    });
}


function handleLogFileContentResponse(data) {
    if (data.error) {
        $("#log_file_content").html(`<div class="alert alert-danger">${data.error}</div>`);
        return;
    }


    if (data.is_plain_text) {

        $("#log_file_content").html(`<pre>${data.content}</pre>`);
        return;
    }


    const content = data.content;


    if (Array.isArray(content)) {

        let tableHtml = `
            <table class="table table-striped">
                <thead>
                    <tr>
                        <th>Time</th>
                        <th>Type</th>
                        <th>Event</th>
                    </tr>
                </thead>
                <tbody>
        `;


        for (const event of content) {
            const timestamp = event.timestamp.toFixed(2);
            const type = event.type;
            const eventData = JSON.stringify(event.event, null, 2);

            tableHtml += `
                <tr>
                    <td>${timestamp}</td>
                    <td>${type}</td>
                    <td><pre>${eventData}</pre></td>
                </tr>
            `;
        }

        tableHtml += `
                </tbody>
            </table>
        `;

        $("#log_file_content").html(tableHtml);
    } else {

        $("#log_file_content").html(`<pre>${JSON.stringify(content, null, 2)}</pre>`);
    }
}

function formatFileSize(bytes) {
    switch (true) {
        case (bytes < 1024):
            return bytes + " B";
        case (bytes < 1024 * 1024):
            return (bytes / 1024).toFixed(2) + " KB";
        case (bytes < 1024 * 1024 * 1024):
            return (bytes / (1024 * 1024)).toFixed(2) + " MB";
        default:
            return (bytes / (1024 * 1024 * 1024)).toFixed(2) + " GB";
    }
}

$(document).ready(function () {//on doc ready init
    initLoggingPage();
});

/**
 *
 * @param obj
 */
function handleWebSocketMessage(obj) {
    obj.op === "get_log_files_callback" ? handleLogFilesResponse(obj) : obj.op === "get_log_file_content_callback" ? handleLogFileContentResponse(obj) : {};
}