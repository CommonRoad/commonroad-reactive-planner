/*
 * representation of lanelets
 */

#ifndef HEADER_LANELET
#define HEADER_LANELET

#include "../../auxiliaryDefs/structs.h"

class lanelet
{
public:

	/*
	 * constructor
	 */
	lanelet();

	/*
	 * destructor
	 */
	~lanelet();

	/*
	 * setter functions
	 */
	void setId(const size_t num);
	void addLeftVertice(const vertice left);
	void addRightVertice(const vertice right);
	void addCenterVertice(const vertice center);
	void createCenterVertices();

	/*
	* getter functions
	*/
	size_t getId() const;
	std::vector <vertice> getLeftBorder() const;
	std::vector <vertice> getRightBorder() const;
	std::vector <vertice> getCenter() const;

private:
	size_t id; // unique id
	std::vector <vertice> leftBorder; // vertices of left border
	std::vector <vertice> rightBorder; // vertices of right border
	std::vector <vertice> centerVertices; // center vertices of lanelet
};




#endif
