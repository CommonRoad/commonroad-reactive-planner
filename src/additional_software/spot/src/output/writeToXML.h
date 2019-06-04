#include "../world/lane/lane.h"
#include "../world/obstacle/staticObstacle.h"
#include "../world/obstacle/vehicle.h"
#include "../world/obstacle/pedestrian.h"

void writeToXML(std::vector <vehicularLanelet> lanelets, std::vector <obstacle*> obstacles, std::string xmlFile, float timeStepSize);

