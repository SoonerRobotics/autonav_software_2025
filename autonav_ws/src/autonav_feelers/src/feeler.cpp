#pragma once

class Feeler {
private:
    int x = 0;
    int y = 0;
    double length = 0.0;

    int original_x = 0;
    int original_y = 0;
    double original_length = 0.0;

    cv::Scalar color;
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
}

/**
 * Draw the feeler using its color on the provided image.
 * @param image an image that the feeler can be drawn on
 */
void Feeler::draw(cv::Mat image) {
    //TODO
}

// def add(); //TODO
// def sub(); //TODO
