# image shape is 800x1600x3; 1600 because it's two 800x800 side by side because dual camera
WIDTH = 960 #TODO
HEIGHT = 640 #TODO

# return the sign of the number
def sign(x):
    return -1 if x < 0 else 1

class Feeler:
    # create a 2 dimensional vector with cartesian coordinates
    # (0, 0) is assumed to be the center of the image, this will take some finangling to make work
    # length is auto-calculated and should never be manually set
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.length = self.dist(x, y)

        # necessary so the vector can grow back up to original size if there isn't an obstacle in the way
        self.original_x = x
        self.original_y = y

        self.color = BLUE
    
    # get x and y cartesian coordinates as a tuple, from (0,0) so not centered in the image
    def getXY(self):
        return self.x, self.y

    # get the vector as an angle and length FIXME
    def toPolar(self):
        try:
            angle = atan(self.y / self.x)
        except ZeroDivisionError:
            angle = 180

        return angle, self.length
    
    # set the x and y cartesian coordinates of the vector, other attributes will be updated accordingly
    def setXY(self, x, y):
        self.x = x
        self.y = y

        self.length = self.dist(x, y)
    
    # convert x and y coordinates so that they are relative to the center of the image
    def centerCoordinates(self, x, y):
        return (x + WIDTH//2), (y + HEIGHT//2 + 100)

    # draw the feeler on the given image
    def draw(self, image):
        startPt = self.centerCoordinates(0, 0)
        endPt = self.centerCoordinates(self.x, self.y)
        
        cv2.line(image, startPt, endPt, self.color, thickness=5)
    
    # mask is supposed to be a binary openCV image I think
    def update(self, mask):
        x = 0
        y = 0
        
        prev_x = 0
        prev_y = 0

        new_x = 0
        new_y = 0

        x_dir = sign(self.original_x)
        y_dir = sign(self.original_y)

        # for vertical lines, assign a slope of None and treat them as a special case in the main loop
        try:
            slope = self.original_y / self.original_x
        except ZeroDivisionError:
            slope = None

        while True:
            # vertical line, just need to move along the y-axis
            if slope is None:
                y += 1
            # horizontal line, just move along th x-axis
            elif slope == 0:
                x += 1

            # if slope is shallow, make x the independent variable
            elif abs(slope) <= 1:
                # get the y as a function of x
                new_y = abs(slope) * x

                # if the new y is higher than the previous one
                if (new_y - prev_y) > 0:
                    y += 1 # then go up by 1 y

                x += 1
                prev_y = y

            # slope is steep, do y as independent variable
            else:
                # get x as a function of y
                new_x = abs(1/slope) * y

                # and then if the new x is larger than the old one
                if (new_x - prev_x) > 0:
                    x += 1 # go up by one

                y += 1
                prev_x = x
            
            # if any of the pixel's color values (in RGB I think) are > 0 then
            check_x, check_y = self.centerCoordinates(x*x_dir, y*y_dir)
            if mask[check_y, check_x].any() > 0:
                # that is our new length
                self.setXY(x*x_dir, y*y_dir)
                return # and quit so we don't keep looping 'cause we found an obstacle
            
            elif abs(x) > abs(self.original_x) or abs(y) > abs(self.original_y): # if we're past our original farthest point
                self.setXY(self.original_x, self.original_y) # then we found no obstacle, and should stop looping
                return


    def __add__(self, other):
        ret = Feeler(self.x + other.x, self.y + other.y)
        ret.color = self.color
        
        return ret
    
    def __sub__(self, other):
        ret = Feeler(self.x - other.x, self.y - other.y)
        ret.color = self.color
        
        return ret

    # distance from (0, 0) to given coordinates
    def dist(self, x, y):
        return sqrt(x**2 + y**2)
