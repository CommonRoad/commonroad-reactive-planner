/*
 * provides functions to obtain the informations of a xml file
 */


#ifndef HEADER_READXMLFILE
#define HEADER_READXMLFILE

#include <iostream>

void readXMLfile(std::string xmlFile);

// get time step from xml file
float getTimeStep(std::string xmlFile);

#endif
