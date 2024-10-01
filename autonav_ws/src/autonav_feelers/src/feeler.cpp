#pragma once

#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/msg/image.hpp"

#define PI 3.141592653

/**
 * Convert degrees to radians
 * @param degrees the angle in degrees [0, 360] to convert to radians
 * @return the angle in radians from [0, 2pi]
 */
double radians(double degrees) {
    return degrees * (PI / 180);
}

class Feeler {
public:
    Feeler(int x, int y);
    ~Feeler();

    int getX();
    int getY();
    double getLength();

    std::vector<double> toPolar();
    std::vector<int> centerCoordinates(int x, int y, int width, int height);
    double dist(int x, int y);

    int getOriginalX();
    int getOriginalY();
    double getOriginalLength();

    cv::Scalar getColor();
    void setColor(cv::Scalar c);

    void setXY(int x, int y);
    void update(cv::Mat *mask);
    void draw(cv::Mat image);

    Feeler operator+(Feeler const &other);
    Feeler operator-(Feeler const &other);
private:
    int x = 0;
    int y = 0;
    double length = 0.0;

    int original_x = 0;
    int original_y = 0;
    double original_length = 0.0;

    cv::Scalar color;
};

/**
 * Feeler constructor. Takes integer arguments for fast math and because the image uses integer coordinate for pixels.
 * Acts like a math vector of type <a, b> where (a, b) is the terminal point of the 2-d vector.
 * (0, 0) is assumed to be the center of the image
 * length is autocalulated and should never be manually set
 * @param x the x component of the feeler
 * @param y the y component of the feeler
 */
Feeler::Feeler(int x, int y) {
    this->x = x;
    this->y = y;
    this->length = this->dist(x, y);

    // this is necessary so we can remember our original size and grow back up to it in the absence of an obstacle
    this->original_x = original_x;
    this->original_y = original_y;
    this->original_length = this->length;

    this->color = cv::Scalar(200, 0, 0);
}

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

/**
 * Set the color of the feeler. This is important if it gets drawn on an image on the UI for debugging
 * @param the color of the feeler, in BGR because OpenCV
 */
void Feeler::setColor(cv::Scalar c) {
    this->color = c;
}

/**
 * Get the polar coordinates of the end of the feeler from its x and y coordinates in radians.
 * @return the polar coordinates of the feeler in radians
 */
std::vector<double> Feeler::toPolar() {
    std::vector<double> polar;

    double length = this->dist(this->x, this->y);
    polar.push_back(length);

    double angle = std::tan(static_cast<double>(this->y) / static_cast<double>(this->x)); //FIXME this can ZeroDivisionError
    polar.push_back(angle);

    return polar;
}

/**
 * Get the coordinates of a pixel in top-left origin (opencv) from assuming origin is the center of the image (feelers)
 * @param x the x coordinate to translate
 * @param y the y coordiante to translate
 * @return the coordinates of the pixel in opencv-land
 */
std::vector<int> Feeler::centerCoordinates(int x, int y, int width, int height) {
    std::vector<int> ret;

    ret.push_back(x + width/2);
    ret.push_back(y + height/2);

    return ret;
}

double Feeler::dist(int x, int y) {
    return std::sqrt((x*x) + (y*y));
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
 * This was copied/pasted from feeler.py, see feeler.py for details
 * FIXME TODO we should pass this a pointer not the whole matrix so we can throw it into some threads.
 * @param the thresholded image to perform feeler on
 */
void Feeler::update(cv::Mat *mask) {
    int channels = mask->channels();

    int x_ = 0;
    int y_ = 0;

    int prev_x = 0;
    int prev_y = 0;

    double new_y = 0;
    double new_x = 0;

    int x_dir = this->original_x < 0 ? -1 : 1;
    int y_dir = this->original_y < 0 ? -1 : 1;

    double slope = 0;
    bool slopeIsInfinity = false;
    if (this->original_y != 0) {
        slope = static_cast<double>(this->original_y) / static_cast<double>(this->original_y);
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
    
    auto coords = this->centerCoordinates(x_*x_dir, y_*y_dir, mask->cols, mask->rows);

    // if any of the pixel's color values (in RGB I think) are > 0 then
    if (mask[coords[1], coords[0]].any() > 0) {
        //TODO: https://stackoverflow.com/questions/7899108/opencv-get-pixel-channel-value-from-mat-image
        // that is our new length
        this->setXY(x_*x_dir, y_*y_dir);
        return; // and quit so we don't keep looping 'cause we found an obstacle
    } else if (abs(x_) > abs(this->original_x) || abs(y_) > abs(this->original_y)) { // if we're past our original farthest point
        this->setXY(this->original_x, this->original_y); // then we found no obstacle, and should stop looping
        return;
    }
}

/**
 * Draw the feeler using its color on the provided image.
 * @param image an image that the feeler can be drawn on
 */
void Feeler::draw(cv::Mat image) {
    cv::Point startPt, endPt;
    auto startCoords = this->centerCoordinates(0, 0, image.cols, image.rows);
    startPt.x = startCoords[0];
    startPt.y = startCoords[1];

    auto endCoords = this->centerCoordinates(this->x, this->y, image.cols, image.rows);
    endPt.x = endCoords[0];
    endPt.y = endCoords[1];

    cv::line(image, startPt, endPt, this->color, 5); // thickness of 5
}

/**
 * Add a feeler to another feeler
 * does basic vector addition of the type <a,b>+<c,d> = <a+c, b+d>
 * @param the feeler to add to this feeler
 * @return a new feeler with values copied from the current feeler plus the other feeler
 */
Feeler Feeler::operator+(Feeler const &other) {
    Feeler ret = Feeler(this->x + other.x, this->y + other.y);
    ret.color = this->color;

    return ret;
}

/**
 * Subtract a feeler from another feeler
 * does basic vector subtraction of the type <a,b>-<c,d> = <a-c, b-d>
 * @param the feeler to subtract from this feeler
 * @return a new feeler with values copied from the current feeler minus the other feeler
 */
Feeler Feeler::operator-(Feeler const &other) {
    Feeler ret = Feeler(this->x - other.x, this->y - other.y);
    ret.color = this->color;

    return ret;
}