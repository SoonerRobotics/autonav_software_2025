#pragma once

#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/msg/image.hpp"
// #include "feeler_node.cpp"
#include "autonav_shared/node.hpp"
// #include "autonav_shared/types.hpp"

#define PI 3.141592653

/**
 * Convert degrees to radians
 * @param degrees the angle in degrees [0, 360] to convert to radians
 * @return the angle in radians from [0, 2pi]
 */
double radians(double degrees) {
    return degrees * (PI / 180);
}

/**
 * Interpolates (linearly) between two colors. For some reason openCV doesn't have a function like this?
 * At least not one I could find. Both colors should have the same number of channels.
 * https://stackoverflow.com/questions/4353525/floating-point-linear-interpolation
 * @param src the base color
 * @param dest the color to interpolate towards
 * @param percentAmount a number between 0 and 1 inclusive to represent how much each color contributes to the final color
 * @return an openCV color somewhere between the two given colors, inclusive
 */
cv::Scalar lerp(cv::Scalar src, cv::Scalar dest, double percentAmount) {
    double ch1 = (src[0] * (1.0 - percentAmount)) + (dest[0] * percentAmount);
    double ch2 = (src[1] * (1.0 - percentAmount)) + (dest[1] * percentAmount);
    double ch3 = (src[2] * (1.0 - percentAmount)) + (dest[2] * percentAmount);
    // ignore channel 4 'cause we shouldn't ever use it for Feelers (though potentially this we could use it for other things,
    //  as this is not a class method)

    return cv::Scalar(ch1, ch2, ch3);
}

class Feeler {
public:
    Feeler(int x, int y);
    ~Feeler() {};

    int getX() const;
    int getY() const;
    double getLength() const;

    std::vector<double> toPolar();
    std::vector<int> centerCoordinates(int x, int y, int width, int height);
    double dist(int x, int y);

    int getOriginalX();
    int getOriginalY();
    double getOriginalLength();
    double getBiasAmount();
    std::string to_string();

    cv::Scalar getColor();
    void setColor(cv::Scalar c);

    void setXY(int x, int y);
    void setOriginalXY(int x, int y);
    void setLength(double newLength);
    void bias(double amount);
    void update(cv::Mat *mask, AutoNav::Node *node);
    void draw(cv::Mat image);

    Feeler operator+(Feeler const &other);
    Feeler operator-(Feeler const &other);
    Feeler operator*(int &other);
    double operator*(Feeler const &other);
private:
    int x = 0;
    int y = 0;
    double length = 0.0;

    int original_x = 0;
    int original_y = 0;
    double original_length = 0.0;
    
    int biased_x = 0;
    int biased_y = 0;
    bool is_biased = false;
    double bias_amount = 0.0;

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
    this->original_x = x;
    this->original_y = y;
    this->original_length = this->length;

    this->color = cv::Scalar(0, 200, 0);
}

/**
 * @return x coordinate of the end of the feeler
 */
int Feeler::getX() const {
    return this->x;
}

/**
 * @return y coordinate of the end of the feeler
 */
int Feeler::getY() const {
    return this->y;
}

/**
 * @return the calculated length of the feeler as it currently stands
 */
double Feeler::getLength() const {
    return std::sqrt((this->x * this->x) + (this->y * this->y));
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

    double angle;
    if (this->x != 0) {
        //TODO FIXME should this be an atan2?
        angle = std::tan(static_cast<double>(this->y) / static_cast<double>(this->x));
    } else {
        angle = this->y > 0 ? PI/2 : 3*PI/2;
    }
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
    ret.push_back(-y + height/2); // flip y coordinate, because if the top-left corner of an image is the origin, then the x axis will still work like normal (left is negative, etc) but the y axis will not

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
 * @return the bias amount (total) of the feeler
 */
double Feeler::getBiasAmount() {
    return this->bias_amount;
}

/**
 * @return color of the feeler (for drawing purposes)
 */
cv::Scalar Feeler::getColor() {
    return this->color;
}

/**
 * TODO figure out which data we actually want to have here
 * @return a string representation of the feeler
 */
std::string Feeler::to_string() {
    std::string string;

    string = "(" + std::to_string(this->x) + ", " + std::to_string(this->y) + ") | length " + std::to_string(this->length) + " | original: " + std::to_string(this->original_length);

    return string;
}

/**
 * Set the x and y of the end point of the feeler, relative to the origin
 * the origin being the center of the image
 */
void Feeler::setXY(int x_, int y_) {
    this->x = x_;
    this->y = y_;

    this->length = this->dist(x_, y_);
}

/**
 * Set the original x and y of the feeler. This is mostly used for all the operator magic methods (*, -, +)
 */
void Feeler::setOriginalXY(int x, int y) {
    this->original_x = x;
    this->original_y = y;

    this->original_length = this->dist(x, y);
}

/**
 * Set the length of the feeler (don't call this directly for the vision feelers, they should set their own length via update())
 * This may or may not work but I think this is how vectors work
 * @param length the length of the feeler
 */
void Feeler::setLength(double newLength) {
    double scaleFactor = newLength / this->length;

    this->x *= scaleFactor;
    this->y *= scaleFactor;
    this->length = dist(this->x, this->y);
}

/**
 * Biases the feeler by an amount of pixels.
 * This essentially allows the feeler to grow past its original maximum length,
 * for the purposes of navigating towards a waypoint/biasing 'forward'/etc
 * @param amount the amount to bias by, in pixels
 */
void Feeler::bias(double amount) {
    // unbias by setting it to 0
    if (amount <= 0.0) {
        this->is_biased = false;
        this->bias_amount = 0.0;
        return;
    }

    this->is_biased = true;

    this->bias_amount = amount;
    double scaleFactor = (this->length+amount) / this->length;
    
    this->biased_x = this->original_x * scaleFactor;
    this->biased_y = this->original_y * scaleFactor;
}

/**
 * Update the end of the feeler / its length from the image
 * It goes pixel by pixel along its length until it reaches a white pixel (obstacle)
 * or until it reaches its original length in which case it stops.
 * This was copied/pasted from feeler.py, see feeler.py for details
 * FIXME TODO we should pass this a pointer not the whole matrix so we can throw it into some threads.
 * @param the thresholded image to perform feeler on
 */
void Feeler::update(cv::Mat *mask, AutoNav::Node *node) {
    int channels = mask->channels();
    auto pixelPtr = (uint8_t*)mask->data;

    // node->log(std::to_string(*pixelPtr));
    // node->log(std::to_string(channels));

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
        slope = static_cast<double>(this->original_y) / static_cast<double>(this->original_x);
    } else {
        slopeIsInfinity = true;
    }

    int pixelsChecked = 0; //FIXME remove when done debugging

    // loop until we hit an obstacle or max_length
    while (1) {
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

        // node->log("Checking pixel: (" + std::to_string(coords[0]) + ", " + std::to_string(coords[1]) + ")", AutoNav::Logging::LogLevel::INFO);
        // node->log("Pixel value is (" + std::to_string(pixelPtr[coords[1]*mask->cols*channels + coords[0]*channels + 0]) + ", " + std::to_string(pixelPtr[coords[1]*mask->cols*channels + coords[0]*channels + 1]) + "," + std::to_string(pixelPtr[coords[1]*mask->cols*channels + coords[0]*channels + 2]) + ")");

        // for every one of the pixel's values (out of blue, green, and red as per openCV standard)
        pixelsChecked++;
        for (int i = 0; i < 3; i++) {
            //reference https://stackoverflow.com/questions/7899108/opencv-get-pixel-channel-value-from-mat-image
            if (pixelPtr[coords[1]*mask->cols*channels + coords[0]*channels + i] > 0) {
                // that is our new length
                // node->log("OBSTACLE FOUND! Pixels checked: " + std::to_string(pixelsChecked), AutoNav::Logging::ERROR);
                // node->log(this->to_string(), AutoNav::Logging::ERROR);
                this->setXY(x_*x_dir, y_*y_dir);
                // node->log(this->to_string(), AutoNav::Logging::ERROR);
                return; // and quit so we don't keep looping 'cause we found an obstacle
            } else if (abs(x_) > abs(this->original_x) || abs(y_) > abs(this->original_y)) { // if we're past our original farthest point
                // check if we have a farther point (aka if we're biased)
                if (this->is_biased) {
                    if (abs(x_) > abs(this->biased_x) || abs(y_) > abs(this->biased_y)) {
                        this->setXY(this->biased_x, this->biased_y); // then we found no obstacle, and should stop looping
                        return;
                    }
                } else {
                    this->setXY(this->original_x, this->original_y); // then we found no obstacle, and should stop looping
                    // node->log("NO OBSTACLE FOUND! Pixels checked: " + std::to_string(pixelsChecked), AutoNav::Logging::ERROR);
                    return;
                }
            }
        }
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
    endPt.x = std::clamp(endCoords[0], 0, image.cols);
    endPt.y = std::clamp(endCoords[1], 0, image.rows);

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
    ret.setColor(this->color);

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
    ret.setColor(this->color);

    return ret;
}

/**
 * Multiply a feeler by a scalar value.
 * This works exactly like in math. 
 * Could do with a double version too.
 * @param scalarNum an integer to multiply by
 * @return a new feeler with values copied from the old one
 */
Feeler Feeler::operator*(int &scalarNum) {
    Feeler ret = Feeler(this->x * scalarNum, this->y * scalarNum);
    
    ret.setColor(this->color);

    return ret;
}

/**
 * Dot product a feeler with another feeler.
 * This works exactly like in math. v1 = <x1, y1>; v2 = <x2, y2>
 * result = (x1*x2) + (y1*y2) 
 * @param other the other Feeler
 * @return a number representing the dot product of the two Feelers.
 */
// int Feeler::operator*(Feeler const &other) {
//     return (this->x * other.getX()) + (this->y * other.getY());
// }

/**
 * Dot product a feeler with another feeler, but normalize both of them first.
 * So we're essentially just getting the angle of each.
 * This works exactly like in math. v1 = |<x1, y1>|; v2 = |<x2, y2>|
 * result = v1 dot v2
 * @param other the other Feeler
 * @return a number representing the dot product of the two Feelers after both have been normalized.
 */
double Feeler::operator*(Feeler const &other) {
    double x_norm = this->x / this->getLength();
    double y_norm = this->y / this->getLength();

    double other_x_norm = other.getX() / other.getLength();
    double other_y_norm = other.getY() / other.getLength();

    return (x_norm * other_x_norm) + (y_norm * other_y_norm);
}