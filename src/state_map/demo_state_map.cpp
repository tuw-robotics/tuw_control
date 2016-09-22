#include <tuw_control/state_map/base_types.h>

using namespace tuw;

// \brief Main
int main() {
    RectWithVector rectangleWithVector {{{0., 1.}, {2., 3.}}, {4.,5.,6.}};
    RectWithVector otherRectangleWithVector;

    std::cout << ( rectangleWithVector == rectangleWithVector ) << std::endl;
    std::cout << ( rectangleWithVector == otherRectangleWithVector ) << std::endl << std::endl;

    rectangleWithVector[2] = 3.;
    rectangleWithVector[3] = 2.;

    std::cout << rectangleWithVector.size() << std::endl;
    std::cout << rectangleWithVector.rectangle.size() << std::endl;
    std::cout << rectangleWithVector.rectangle.lowerLeftPoint.size() << std::endl;
    std::cout << rectangleWithVector.rectangle.upperRightPoint.size() << std::endl;
    std::cout << rectangleWithVector.doubleVector.size() << std::endl << std::endl;

    std::cout << rectangleWithVector.rectangle.lowerLeftPoint.x << " = " << rectangleWithVector[0] << std::endl;
    std::cout << rectangleWithVector.rectangle.lowerLeftPoint.y << " = " << rectangleWithVector[1] << std::endl;
    std::cout << rectangleWithVector.rectangle.upperRightPoint.x << " = " << rectangleWithVector[2] << std::endl;
    std::cout << rectangleWithVector.rectangle.upperRightPoint.y << " = " << rectangleWithVector[3] << std::endl;
    std::cout << rectangleWithVector.doubleVector[0] << " = " << rectangleWithVector[4] << std::endl;
    std::cout << rectangleWithVector.doubleVector[1] << " = " << rectangleWithVector[5] << std::endl;
    std::cout << rectangleWithVector.doubleVector[2] << " = " << rectangleWithVector[6] << std::endl;

    return 0;
}
