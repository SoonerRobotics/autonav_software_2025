#pragma once

#include <cmath>

//TODO move to a utils.cpp file or something
int sign(int x) {
    if (x < 0) {
        return -1;
    }
    return 1;
}

class Feeler {
public:
    Feeler();
    ~Feeler();

    int getX();
    int getY();
    double getLength();

    std::vector<double> toPolar();
    std::vector<int> centerCoordinates(int x, int y);
    double dist(int x, int y);

    int getOriginalX();
    int getOriginalY();
    double getOriginalLength();

    cv::Scalar getColor();

    void setXY(int x, int y);
    void update(cv::Mat mask);
    void draw(cv::Mat image); //TODO

    def add(); //TODO
    def sub(); //TODO
private:
    int x = 0
    int y = 0;
    double length = 0.0;

    int original_x = 0;
    int original_y = 0;
    double original_length = 0.0;

    cv::Scalar color;
};

/**
 * @return x coordinate of the end of the feeler
 */
int Feeler::getX() {
    return this->x;
}

/**
 * @return y coordinate of the end of the feeler
 */
int Feeler::getY() {
    return this->y;
}

//TODO I don't think we need this
// /**
//  * @return length of the feeler
//  */
// double getLength() {
//     return this->length;
// }

//TODO
std::vector<double> toPolar();
//TODO
std::vector<int> centerCoordinates(int x, int y);

double Feeler::dist(int x, int y) {
    return math.sqrt((x*x) + (y*y));
}

/**
 * @return x coordinate of the end of the feeler when it was created
 */
int Feeler::getOriginalX() {
    return this->original_x;
}

/**
 * @return y coordinate of the end of the feeler when it was created
 */
int Feeler::getOriginalY() {
    return this->original_y;
}

/**
 * @return length of the feeler when it was created
 */
double Feeler::getOriginalLength() {
    return this->original_length;
}

/**
 * @return color of the feeler (for drawing purposes)
 */
cv::Scalar Feeler::getColor() {
    return this->color;
}

/**
 * Set the x and y of the end point of the feeler, relative to the origin
 * the origin being the center of the image
 */
void Feeler::setXY(int x, int y) {
    this->x = x;
    this->y = y;
}

/**
 * Update the end of the feeler / its lnegth from the image
 * It goes pixel by pixel along its length until it reaches a white pixel (obstacle)
 * or until it reaches its original length in which case it stops.
 * @param the thresholded image to perform feeler on
 */
void Feeler::update(cv::Mat mask) {
    //TODO
    int x_ = 0;
    int y_ = 0;

    int prev_x = 0;
    int prev_y = 0;

    double new_y = 0;
    double new_x = 0;

    int x_dir = sign(this->original_x);
    int y_dir = sign(this->original_y);

    double slope = 0;
    bool slopeIsInfinity = false;
    if (this->original_y != 0) {
        slope = this->original_y / this->original_y;
    } else {
        slopeIsInfinity = true;
    }

    // vertical line, just need to move along the y-axis
    if (slopeIsInfinity) {
        y_ += 1;
    } else if (slope == 0) {
        x_ += 1;
    } else if (abs(slope) <= 1) {
        // if slope is shallow, make x the independent variable
        // get the y as a function of x
        new_y = abs(slope) * x_;

        // if the new y is higher than the previous one
        if ((new_y - prev_y) > 0) {
            y_ += 1; // then go up by 1 y
        }

        x_ += 1;
        prev_y = y_;
    } else { // slope is steep, do y as independent variable
        // get x as a function of y
        new_x = abs(1/slope) * y_;

        // and then if the new x is larger than the old one
        if ((new_x - prev_x) > 0) {
            x_ += 1; // go up by one
        }

        y_ += 1;
        prev_x = x_;
    }
    
    std::vector<double> coords = self.centerCoordinates(x_*x_dir, y_*y_dir);

    // if any of the pixel's color values (in RGB I think) are > 0 then
    if (mask[coords[1], coords[0]].any() > 0) {
        // that is our new length
        this->setXY(this->x_*x_dir, this->y_*y_dir);
        return; // and quit so we don't keep looping 'cause we found an obstacle
    } else if (abs(x_) > abs(this->original_x) || abs(y_) > abs(this->original_y)) { // if we're past our original farthest point
        this->setXY(self.original_x, self.original_y); // then we found no obstacle, and should stop looping
        return;
    }
}

/**
 * Draw the feeler using its color on the provided image.
 * @param image an image that the feeler can be drawn on
 */
void Feeler::draw(cv::Mat image) {
    startPt = Point(this->centerCoordinates(0, 0));
    endPt = Point(this->centerCoordinates(this->x, this->y));
    cv2::line(image, startPt, endPt, this->color, 5); // thickness of 5
}

// def add(); //TODO
// def sub(); //TODO
