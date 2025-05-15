import cv2
import numpy as np


# copied/pasted from combination.py (really these should be grabbed from the image itself or something)
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

COMBINED_IMAGE_WIDTH = IMAGE_HEIGHT*2 + IMAGE_WIDTH
COMBINED_IMAGE_HEIGHT = IMAGE_HEIGHT*2 + IMAGE_WIDTH

videoReader = cv2.VideoCapture("./data/debug_combined.mp4")

written = False

while True:
    ret, combined_image = videoReader.read()

    if not ret:
        break

    old_combined = combined_image
    old_combined = cv2.resize(old_combined, (640, 480))
    # cv2.imshow("before", old_combined)

    # add an alpha channel to the combined image so we can use transparency for blitting the images together later
    combined_image_front = cv2.cvtColor(combined_image, cv2.COLOR_BGR2BGRA)
    combined_image_left = cv2.cvtColor(combined_image, cv2.COLOR_BGR2BGRA)
    combined_image_right = cv2.cvtColor(combined_image, cv2.COLOR_BGR2BGRA)
    combined_image_back = cv2.cvtColor(combined_image, cv2.COLOR_BGR2BGRA)

    #TODO this was also copypastaed from combination.py
    x_offset = (COMBINED_IMAGE_WIDTH//2)-(IMAGE_WIDTH//2)

    y_shrink = 240
    x_shrink = 240

    image_front = combined_image[0 : IMAGE_HEIGHT, x_offset : x_offset+IMAGE_WIDTH]
    image_left = combined_image[IMAGE_HEIGHT : IMAGE_HEIGHT+IMAGE_WIDTH, 0 : IMAGE_HEIGHT]
    image_right = combined_image[IMAGE_HEIGHT : IMAGE_HEIGHT+IMAGE_WIDTH, -IMAGE_HEIGHT : ]
    image_back = combined_image[COMBINED_IMAGE_HEIGHT-IMAGE_HEIGHT : , x_offset : x_offset+IMAGE_WIDTH]

    image_left = cv2.rotate(image_left, cv2.ROTATE_90_CLOCKWISE)
    image_right = cv2.rotate(image_right, cv2.ROTATE_90_COUNTERCLOCKWISE)
    image_back = cv2.rotate(image_back, cv2.ROTATE_180)

    # cv2.imshow("pre-flattened", combined_image)
    
    # top left, top right, bottom left, botom right
    # src_pts = np.float32([(0, 0), (IMAGE_WIDTH, 0), (0, IMAGE_HEIGHT), (IMAGE_WIDTH, IMAGE_HEIGHT)])
    # src_pts = np.float32([(218, 64), (414, 64), (0, IMAGE_HEIGHT), (IMAGE_WIDTH, IMAGE_HEIGHT)])
    src_pts = np.float32([(200, 0), (IMAGE_WIDTH-200, 0), (0, IMAGE_HEIGHT), (IMAGE_WIDTH, IMAGE_HEIGHT)])
    dest_pts = np.float32([(0, 0), (IMAGE_WIDTH, 0), (200, IMAGE_HEIGHT), (IMAGE_WIDTH-200, IMAGE_HEIGHT)])

    src_pts = np.float32([(0, 0), (640, 0), (0, 480), (640, 480)])
    dest_pts = np.float32([(0, 0), (640, 0), (0, 480), (640, 480)])

    src_pts = np.float32([(240, 80), (380, 80), (0, 480), (640, 480)])
    dest_pts = np.float32([(240, 0), (380, 0), (240, 480), (400, 480)])

    # matrix = cv2.getPerspectiveTransform(dest_pts, src_pts)

    # essentially what we need to do is:
    #  - make a big blank combined_image
    #  - warp the individual perspectives into a bigger image thingy
    #  - blit the warpped image into its position on the big image
    #  - make masks for all the big images using alpha transparency or whatever
    #  - binary and all of the masks together

    # nah nah nah we just need to do like inRange() and then binary mask the images together and whatnot
    # that is gonna require 4 separate like, big blank images and stuff

    bigImageFront = np.zeros_like(combined_image)
    bigImageLeft = np.zeros_like(combined_image)
    bigImageRight = np.zeros_like(combined_image)
    bigImageBack = np.zeros_like(combined_image)

    matrix = cv2.getPerspectiveTransform(src_pts, dest_pts)
    image_front_flattened = cv2.warpPerspective(image_front, matrix, (640, 480), borderValue=(0, 0, 0, 1))
    image_left_flattened = cv2.warpPerspective(image_left, matrix, (640, 480), borderValue=(0, 0, 0, 1))
    image_right_flattened = cv2.warpPerspective(image_right, matrix, (640, 480), borderValue=(0, 0, 0, 1))
    image_back_flattened = cv2.warpPerspective(image_back, matrix, (640, 480), borderValue=(0, 0, 0, 1))

    # print(image_front_flattened.shape)
    if not written:
        cv2.imwrite("./data/flattened_front.png", image_front_flattened)
        written = True

    image_left_flattened = cv2.rotate(image_left_flattened, cv2.ROTATE_90_COUNTERCLOCKWISE)
    image_right_flattened = cv2.rotate(image_right_flattened, cv2.ROTATE_90_CLOCKWISE)
    image_back_flattened = cv2.rotate(image_back_flattened, cv2.ROTATE_180)

    # empty combined_image as it will be our destination
    combined_image.fill(0)

    #TODO https://stackoverflow.com/questions/40895785/using-opencv-to-overlay-transparent-image-onto-another-image we don't want to clip the images when we move them inwards using x_shrink and y_shrink
    # put each individual transformed camera image (left, right, etc) into its own big combined image with just itself
    bigImageFront[0 + y_shrink : IMAGE_HEIGHT + y_shrink, x_offset : x_offset+IMAGE_WIDTH] = image_front_flattened
    bigImageLeft[IMAGE_HEIGHT : IMAGE_HEIGHT+IMAGE_WIDTH, 0 + x_shrink : IMAGE_HEIGHT + x_shrink] = image_left_flattened
    bigImageRight[IMAGE_HEIGHT : IMAGE_HEIGHT+IMAGE_WIDTH, -IMAGE_HEIGHT - x_shrink : -x_shrink] = image_right_flattened
    bigImageBack[COMBINED_IMAGE_HEIGHT-IMAGE_HEIGHT - y_shrink : -y_shrink, x_offset : x_offset+IMAGE_WIDTH] = image_back_flattened

    # get masks for each of those big images
    mask_front = cv2.inRange(bigImageFront, (1,1,1,0), (255, 255, 255, 255))
    mask_left = cv2.inRange(bigImageLeft, (1,1,1,0), (255, 255, 255, 255))
    mask_right = cv2.inRange(bigImageRight, (1,1,1,0), (255, 255, 255, 255))
    mask_back = cv2.inRange(bigImageBack, (1,1,1,0), (255, 255, 255, 255))

    # https://docs.opencv.org/4.x/d0/d86/tutorial_py_image_arithmetics.html
    # https://stackoverflow.com/questions/44333605/what-does-bitwise-and-operator-exactly-do-in-opencv
    # overlay all the individual images (bigImageFront, etc) onto the final image (combined_image) using the masks
    combined_image = cv2.bitwise_or(bigImageFront, combined_image, mask_front)
    combined_image = cv2.bitwise_or(bigImageLeft, combined_image, mask_left)
    combined_image = cv2.bitwise_or(bigImageRight, combined_image, mask_right)
    combined_image = cv2.bitwise_or(bigImageBack, combined_image, mask_back)

    # shrunk_image = cv2.resize(combined_image, (COMBINED_IMAGE_WIDTH//2, COMBINED_IMAGE_HEIGHT//2))
    # shrunk_image = combined_image
    shrunk_image = cv2.resize(combined_image, (640, 480))
    
    # cv2.imshow("flattened big", combined_image)
    cv2.imshow("combined flattened", shrunk_image)
    # cv2.imshow("flattened", image_front_flattened)
    # cv2.imshow("regular", image_front)
    keycode = cv2.waitKey()

    if keycode == ord("q"):
        break


videoReader.release()
cv2.destroyAllWindows()