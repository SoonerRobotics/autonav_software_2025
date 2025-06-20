$(document).ready(function () {
    $('.page').hide();
    $('.page:first').show();


    $('.nav-link').on('click', function () {
        // Bootstrap tab? skip
        if ($(this).attr('data-bs-toggle')) {
            return;
        }

        const activeNav = $('.nav-link.active'); //grabs all elements with .nav-link.active classes
        //console.log(activeNav.attr('data-page-id')); //prints out the page id came from
        activeNav.removeClass('active');//Disable current tab
        const oldPageId = activeNav.attr('data-page-id'); //Current page id
        $('#' + oldPageId).hide(); //hide old page with IDD

        $(this).addClass('active');//make clicked element active
        //console.log($(this).attr('data-page-id')); //Print the clicked element to the console
        const pageId = $(this).attr('data-page-id'); //new page ID to be shown where (this) is the clicked element
        $('#' + pageId).show();
    });
})
