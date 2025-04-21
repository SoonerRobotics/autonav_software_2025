$(document).keydown(function (e) {
    let RIGHTKEY = 37,
        LEFTKEY = 39,
        QKEY = 81,
        EKEY = 69;


    switch (e.which) {
        case RIGHTKEY:
            i = currentPageIndex();
            i === 0 ? i = dataPageIds.length - 1 : i--;//i===lastPage? 0 : ++
            hideCurrentActivePage();
            showNewActivePAge();
            break;
        case LEFTKEY:
            i = currentPageIndex();
            i === dataPageIds.length - 1 ? i = 0 : i++;//i===lastPage? 0 : ++
            hideCurrentActivePage();
            showNewActivePAge();
            break;
        case QKEY:
            if (e.shiftKey && e.altKey) {
                const generalSettings = document.getElementById('General Settings');
                generalSettings.style.display = generalSettings.style.display === 'none' ? 'block' : 'none';
            }
            break;
        case EKEY:
            if (e.shiftKey && e.altKey) {
                const generalSettings = document.getElementById('Device State Column');
                generalSettings.style.display = generalSettings.style.display === 'none' ? 'block' : 'none';
            }
            break;
        default:
            return;
    }
    e.preventDefault();
});

// These are the page IDs that are used to navigate through the tabs (basically tab names)
const dataPageIds = [
    "dashboard",
    "vision",
    "configuration",
    "preferences"
];
let i = -1;

function hideCurrentActivePage() {
    const activeNav = $('.nav-link.active'); // grabs all elements with .nav-link.active classes
    activeNav.removeClass('active'); // Disable current tab from active tab list
    const oldPageId = activeNav.attr('data-page-id'); // Current page id
    $('#' + oldPageId).hide(); // Hide old page
}

function showNewActivePAge() {
    const newPageId = dataPageIds[i]; // New page ID to navigate to
    $(`.nav-link[data-page-id="${newPageId}"]`).addClass('active'); // New active nav element
    $('#' + newPageId).show(); // Show the current page
}


function currentPageIndex() {
    let currentPageId = $('.nav-link.active').attr('data-page-id');
    for (let i = 0; i < dataPageIds.length; i++) {
        if (dataPageIds[i] === currentPageId) {
            return i;
        }
    }
    console.log(`No page found with id ${currentPageId} likely problem with never updating the page index`);
    throw new Error(`No page found with id ${currentPageId}`);
}