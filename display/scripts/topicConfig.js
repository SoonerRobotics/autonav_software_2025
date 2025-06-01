let topicValues = {};

function initTopicConfig() {
    const topicContainer = document.getElementById('topic-config-container');
    if (!topicContainer) return;

    topicContainer.innerHTML = '';
    const topicConstants = getAllTopicConstants();
    
    // config box 4 e/a topic
    topicConstants.forEach(topic => {
        createTopicConfigBox(topic, topicContainer);
    });
}

function getAllTopicConstants() {
    const topics = [];

    for (const key in window) {
        if (key.startsWith('TOPIC_') && typeof window[key] === 'string') {
            topics.push({
                name: key,
                path: window[key],
                value: topicValues[window[key]] || ''
            });
        }
    }

    topics.sort((a, b) => a.name.localeCompare(b.name));
    
    return topics;
}

function createTopicConfigBox(topic, container) {
    const itemDiv = document.createElement('div');
    itemDiv.className = 'topic-config-item';
    
    const nameDiv = document.createElement('div');
    nameDiv.className = 'topic-name';
    nameDiv.textContent = `${topic.name} (${topic.path})`;

    const input = document.createElement('input');
    input.type = 'text';
    input.className = 'topic-value-input';
    input.placeholder = topic.value || 'Enter custom value';
    input.dataset.topic = topic.path;

    const saveBtn = document.createElement('button');
    saveBtn.className = 'btn btn-sm btn-primary topic-save-btn';
    saveBtn.textContent = 'Save';
    saveBtn.onclick = function() {
        saveTopicValue(topic.path, input.value);
    };
    
    itemDiv.appendChild(nameDiv);
    itemDiv.appendChild(input);
    itemDiv.appendChild(saveBtn);
    container.appendChild(itemDiv);
}

function saveTopicValue(topicPath, value) {

    topicValues[topicPath] = value;


    send({
        op: "topic_config",
        topic: topicPath,
        value: value
    });
    

    ntf(`Value for ${topicPath} saved`, 'success');
}

// Update a topic value when received from server
function updateTopicValue(topicPath, value) {
    topicValues[topicPath] = value;
    
    // Update the input field if it exists
    const input = document.querySelector(`.topic-value-input[data-topic="${topicPath}"]`);
    if (input) {
        input.placeholder = value;
    }
}


document.addEventListener("DOMContentLoaded", function() {
    document.querySelectorAll('.nav-link').forEach(link => {
        link.addEventListener('click', function() {
            if (this.getAttribute('data-page-id') === 'configuration') {
                setTimeout(initTopicConfig, 100); // Small delay to ensure the page is visible
            }
        });
    });
    
    if (document.querySelector('.nav-link[data-page-id="configuration"]').classList.contains('active')) {
        setTimeout(initTopicConfig, 100);
    }
});


function handleTopicConfigMessage(msg) {
    const { topic, value } = msg;
    if (topic && value !== undefined) {
        updateTopicValue(topic, value);
    }
}

function onTopicConfigData(topic, msg) {
    updateTopicValue(topic, msg.value);
}