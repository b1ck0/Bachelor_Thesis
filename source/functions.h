#ifndef FUNCTIONS_H_INCLUDED
#define FUNCTIONS_H_INCLUDED

void calculateNormal(stl_facet*);
double calculateDistance(stl_vertex*, stl_vertex*);
double calculateArea(stl_facet*);
plane_vector computePlanarVector(stl_vertex, stl_vertex);
double dotProduct(plane_vector, plane_vector);
double crossProduct(plane_vector, plane_vector);
bool checkInside(stl_facet, stl_vertex);
void linearInterpolation3D_z(stl_vertex*, stl_vertex*, stl_vertex*, double);
void linearInterpolation3D_y(stl_vertex*, stl_vertex*, stl_vertex*, double);
user_variables readEntryData(int);
void calculateDomainDimensions(stl_file&, user_variables&);
int calculateDomains(int);
long double determinant1(stl_vertex, stl_vertex, stl_vertex);
long double determinant2(stl_vertex, stl_vertex, stl_vertex);
long double determinant3(stl_vertex, stl_vertex, stl_vertex);
void initializeFacet(double,stl_facet*,float);
void defineEdge(stl_edge&, stl_vertex, stl_vertex);
void printEdge(stl_edge);
int ifEqualVertex(stl_vertex, stl_vertex);
int ifCommonEdge(stl_facet, stl_facet);
int ifWaterlineEdge(stl_file, stl_facet);
int ifWaterlineVertex(stl_file, stl_facet);
int ifCommonVertex(stl_facet, stl_facet);
int ifMerelyEqualVertex(stl_vertex, stl_vertex, double);
int ifNearFacet(stl_facet, stl_facet, double);
double calculateDistanceDP(stl_vertex, stl_vertex);
int commonVer(stl_edge, stl_edge);
int ifInteriorEdge(stl_facet, stl_edge);
void addEdgeToFacet(stl_facet, stl_facet&, stl_facet&, stl_facet&, stl_facet&, stl_facet&, stl_facet&, stl_edge, stl_file, user_variables);
void addPointToFacet(stl_facet, stl_facet&, stl_facet&, stl_facet&, stl_vertex, stl_file, user_variables);
int intersectSegments(stl_edge, stl_edge);
void drawWaterlineLines(stl_vertex, stl_edge&, stl_edge&, stl_edge&, stl_edge&, stl_file, user_variables);
int segmentSegments(stl_edge, stl_edge);
unsigned int maxOfFour(unsigned int, unsigned int, unsigned int, unsigned int);
void intersectionCoords(stl_vertex&, stl_edge, stl_edge, stl_file);
void splitEdge(stl_edge, stl_vertex, stl_edge&, stl_edge&);
void splitEdgeOnHalf(stl_edge, stl_edge&, stl_edge&, stl_vertex&);
int ifCommonEdge1(stl_facet, stl_edge);
int ifVertexLieOnEdge(stl_edge, stl_vertex);
double round_double(double, int);
void setFacet(stl_facet&, stl_facet, user_variables);
int ifEqualEdge(stl_edge, stl_edge);
int ifParallel(stl_edge, stl_edge);
int ifCommonVertex1(stl_edge, stl_edge);
double minOfFour(double, double, double, double);
bool edgeComparison(stl_edge, stl_edge);
void initializeVertex(stl_vertex, stl_vertex&);
int ifEqualVertex1(stl_vertex, stl_vertex,user_variables);
int returnUncommonVertex(stl_facet, stl_edge, user_variables);
int between(stl_edge, stl_vertex);
int between1(stl_edge, stl_vertex);
int ifWaterlineEdge1(stl_file, stl_facet);
void printFacet(stl_facet);
void printVertex(stl_vertex);

#endif // FUNCTIONS_H_INCLUDED
