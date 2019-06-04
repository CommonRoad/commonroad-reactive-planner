/*
 * class for representing a drivable union of road segments
 */

#ifndef HEADER_LANE
#define HEADER_LANE

#include "../lanelets/lanelet_operations.h"
#include <vector>
#include <memory>

struct shortestPath
{
	std::vector <size_t> indexBorder;
	std::vector <double> xi;
	std::vector <size_t> side;
	std::vector <double> curvature;
};


class lane
{
public:

	/*
	 * constructor
	 */
	lane();

	/*
	 * destructor
	 */
	~lane();

	/*
	 * setter functions
	 */
	void setId(const size_t num);
	void setLeftBorder(const std::vector <vertice> vertices,
						const std::vector <double> distances,
						const std::vector <double> curvature);
	void setRightBorder(const std::vector <vertice> vertices,
						const std::vector <double> distances,
						const std::vector <double> curvature);
	void setLeftBorderVertices(const std::vector <vertice> vertices);
	void setRightBorderVertices(const std::vector <vertice> vertices);
	void addAssemblingLanelet(vehicularLanelet* ass);
	void setAssemblingLanelet(std::vector <vehicularLanelet*> ass);
	void setSpeedLimit(const std::vector <double> lim);
	void setCenterVertices(const center_struct vertice);
	void setAdjacentLeft(lane* adjLane, const std::string drv);
	void setAdjacentRight(lane* adjLane, const std::string drv);
	//void setSide(const std::string sd);
	void setCenterVertices(const std::vector <vertice> centerVer);
	void addCenterVertice(const vertice centerVer);
	void addCenterVertices(const std::vector <vertice> centerVer);
	void setLeftBorderDistances(const std::vector <double> distances);
	void setRightBorderDistances(const std::vector <double> distances);
	void setLeftBorderCurvature(const std::vector <double> curvature);
	void setRightBorderCurvature(const std::vector <double> curvature);
	void setShortestPath(shortestPath shortest);

	// Copy constructor
	void copyLane(lane copiedLane);

	/*
	* getter functions
	*/
	size_t getId() const;
	border getLeftBorder() const;
	std::vector <vertice> getLeftBorderVertices() const;
	std::vector <double> getLeftBorderDistances() const;
	std::vector <double> getLeftBorderCurvature() const;
	std::vector <double> getRightBorderDistances() const;
	std::vector <double> getRightBorderCurvature() const;
	border getRightBorder() const;
	std::vector <vertice> getRightBorderVertices() const;
	std::vector <vehicularLanelet*> getAssLanelets() const;
	std::vector <double> getSpeedLimit() const;
	double getMaxSpeedLimit() const;
	center_struct getCenter() const;
	std::vector <lane*> getAdjacentLeft();
	std::string getAdjacentLeftDir() const;
	std::vector <lane*> getAdjacentRight();
	std::string getAdjacentRightDir() const;
	//std::string getSide() const;
	std::vector <vertice> getCenterVertices() const;
	shortestPath getShortestPath() const;
	std::vector <double> getOptVelProfile() const;

	void calculateShortestPath();

	void calculateOptimalVelocityProfile();

	static size_t laneIdCount;
	static bool initialized;

private:

	size_t id; // unique id
	border leftBorder;
	border rightBorder;
	std::vector <vehicularLanelet*> assemblingLanelets; // sorted list of lanelets which assemble to the lane object
	std::vector <double> speedLimit; // official speed limit of the lane in m/s at each vertice
	center_struct center;
	std::vector <lane*> adjacentLeft;
	std::string adjacentLeftDir;
	std::vector <lane*> adjacentRight;
	std::string adjacentRightDir;
	shortestPath shPath;
	std::vector <vertice> centerVertices;
	std::vector<double> optVelocityProfile;
};




#endif
