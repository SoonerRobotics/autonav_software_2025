import cv2
import numpy as np

# THIS HAS ALL BEEN COPY/PASTED FROM vision_test.py

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
    # cv2.imshow("before", old_combined)

    #TODO this was also copypastaed from combination.py
    x_offset = (COMBINED_IMAGE_WIDTH//2)-(IMAGE_WIDTH//2)

    # y_shrink = 150
    # x_shrink = 150

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
    src_pts = np.float32([
        (0, 0),
        (IMAGE_WIDTH-200, 0),
        (0, IMAGE_HEIGHT),
        (IMAGE_WIDTH, IMAGE_HEIGHT)
    ])
    dest_pts = np.float32([
        (0, 0),
        (IMAGE_WIDTH, 0),
        (0, IMAGE_HEIGHT),
        (IMAGE_WIDTH, IMAGE_HEIGHT)
    ])

    bigImage = cv2.Mat()

    # matrix = cv2.getPerspectiveTransform(dest_pts, src_pts)
    matrix = cv2.getPerspectiveTransform(src_pts, dest_pts)
    image_front_flattened = cv2.warpPerspective(image_front, matrix, (640+400, 480+400))
    image_front_flattened = cv2.resize(image_front_flattened, (640, 480))
    # image_left_flattened = cv2.warpPerspective(image_left, matrix, (640, 480))
    # image_right_flattened = cv2.warpPerspective(image_right, matrix, (640, 480))
    # image_back_flattened = cv2.warpPerspective(image_back, matrix, (640, 480))

    # print(image_front_flattened.shape)
    if not written:
        cv2.imwrite("./data/unflattened_front.png", image_front)
        cv2.imwrite("./data/flattened_front.png", image_front_flattened)
        written = True

    # image_left_flattened = cv2.rotate(image_left_flattened, cv2.ROTATE_90_COUNTERCLOCKWISE)
    # image_right_flattened = cv2.rotate(image_right_flattened, cv2.ROTATE_90_CLOCKWISE)
    # image_back_flattened = cv2.rotate(image_back_flattened, cv2.ROTATE_180)

    # combined_image.fill(0)

    # combined_image[0 + y_shrink : IMAGE_HEIGHT + y_shrink, x_offset : x_offset+IMAGE_WIDTH] = image_front_flattened
    # combined_image[IMAGE_HEIGHT : IMAGE_HEIGHT+IMAGE_WIDTH, 0 + x_shrink : IMAGE_HEIGHT + x_shrink] = image_left_flattened
    # combined_image[IMAGE_HEIGHT : IMAGE_HEIGHT+IMAGE_WIDTH, -IMAGE_HEIGHT - x_shrink : -x_shrink] = image_right_flattened
    # combined_image[COMBINED_IMAGE_HEIGHT-IMAGE_HEIGHT - y_shrink : -y_shrink, x_offset : x_offset+IMAGE_WIDTH] = image_back_flattened

    # shrunk_image = cv2.resize(combined_image, (COMBINED_IMAGE_WIDTH//2, COMBINED_IMAGE_HEIGHT//2))
    # shrunk_image = combined_image
    
    # cv2.imshow("flattened big", combined_image)
    # cv2.imshow("combined flattened", shrunk_image)
    # cv2.imshow("flattened", image_front_flattened)
    # cv2.imshow("regular", image_front)
    # keycode = cv2.waitKey()

    # if keycode == ord("q"):
        # break
    break


videoReader.release()
cv2.destroyAllWindows()