#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include "header.h"

using namespace std;
typedef vector<stl_facet> facetVector;

double round_double(double digit, int precision){
    double result;

    result = digit*pow(10,precision);
    result = ceil(result);
    result = result/pow(10,precision);

    return result;
}

void calculateNormal(stl_facet *facet){
  double v1[3];
  double v2[3];

  v1[0] = facet->vertex[1].x - facet->vertex[0].x;
  v1[1] = facet->vertex[1].y - facet->vertex[0].y;
  v1[2] = facet->vertex[1].z - facet->vertex[0].z;
  v2[0] = facet->vertex[2].x - facet->vertex[0].x;
  v2[1] = facet->vertex[2].y - facet->vertex[0].y;
  v2[2] = facet->vertex[2].z - facet->vertex[0].z;

  facet->normal.x = ((double)v1[1] * (double)v2[2]) - ((double)v1[2] * (double)v2[1]);
  facet->normal.y = ((double)v1[2] * (double)v2[0]) - ((double)v1[0] * (double)v2[2]);
  facet->normal.z = ((double)v1[0] * (double)v2[1]) - ((double)v1[1] * (double)v2[0]);
}

double calculateDistance(stl_vertex * v1, stl_vertex * v2){
	return sqrt(pow((v2->x - v1->x),2) + pow((v2->y - v1->y),2) + pow((v2->z - v1->z),2));
}

double calculateArea(stl_facet *facet){
    double a = calculateDistance(&facet->vertex[0], &facet->vertex[1]);
    double b = calculateDistance(&facet->vertex[1], &facet->vertex[2]);
    double c = calculateDistance(&facet->vertex[2], &facet->vertex[0]);

    double p = (a+b+c)/2;

    return sqrt(p*(p-a)*(p-b)*(p-c));
}

plane_vector computePlanarVector(stl_vertex v1, stl_vertex v2){
    plane_vector v;

    v.x = v1.x - v2.x;
    v.y = v1.y - v2.y;

    return v;
}

double dotProduct(plane_vector u, plane_vector v){

    return u.x*v.x + u.y*v.y;
}

double crossProduct(plane_vector u, plane_vector v){
    return u.x*v.y - u.y*v.x;
}

bool checkInside(stl_facet facet, stl_vertex vertex){
    plane_vector v0,v1,v2;
    double dot00,dot01,dot02,dot11,dot12;
    double invDenom,u,v;

    //compute vectors
    v0 = computePlanarVector(facet.vertex[0],facet.vertex[1]);
    v1 = computePlanarVector(facet.vertex[2],facet.vertex[1]);
    v2 = computePlanarVector(vertex,facet.vertex[1]);

    //compute dot products
    dot00 = dotProduct(v0, v0);
    dot01 = dotProduct(v0, v1);
    dot02 = dotProduct(v0, v2);
    dot11 = dotProduct(v1, v1);
    dot12 = dotProduct(v1, v2);

    // Compute barycentric coordinates
    invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
    u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    // Check if point is in triangle
    return (u > 0) && (v > 0) && ( u + v < 1);
}

bool checkOutside(stl_vertex ver1,stl_vertex ver2, stl_vertex ver3, stl_vertex vertex){
    plane_vector v0,v1,v2;
    double dot00,dot01,dot02,dot11,dot12;
    double invDenom,u,v;

    //compute vectors
    v0 = computePlanarVector(ver1,ver2);
    v1 = computePlanarVector(ver3,ver2);
    v2 = computePlanarVector(vertex,ver2);

    //compute dot products
    dot00 = dotProduct(v0, v0);
    dot01 = dotProduct(v0, v1);
    dot02 = dotProduct(v0, v2);
    dot11 = dotProduct(v1, v1);
    dot12 = dotProduct(v1, v2);

    // Compute barycentric coordinates
    invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
    u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    // Check if point is in triangle
    return (u > 0) && (v > 0) && ( u + v > 1);
}

int ifInteriorEdge(stl_facet facet, stl_edge edge){
    unsigned int result = 0;
    bool r1 = false;
    bool r2 = false;

    r1 = checkInside(facet,edge.v1);
    r2 = checkInside(facet,edge.v2);

    if( r1 == true && r2 == true){
        result = 1;
    } else {
        result = 0;
    }

    return result;
}

void linearInterpolation3D_y(stl_vertex * v, stl_vertex * v0, stl_vertex * v1, double y){
	/*v->x = v0->x + (z-v0->z)*(v1->x-v0->x)/(v1->z-v0->z);
	v->y = v0->y + (z-v0->z)*(v1->y-v0->y)/(v1->z-v0->z);
	v->z = z;*/

	//direction vector
	double dx = v1->x - v0->x;
	double dy = v1->y - v0->y;
	double dz = v1->z - v0->z;

	//parametric form
	double xp[] = {v0->x, dx};
	double yp[] = {v0->y, dy};
	double zp[] = {v0->z, dz};

	//finding t
	double t = (y-yp[0])/yp[1];

	//finding point
    v->x = xp[0] + xp[1]*t;
    v->y = y;
    v->z = zp[0] + zp[1]*t;
}

void linearInterpolation3D_z(stl_vertex * v, stl_vertex * v0, stl_vertex * v1, double z){
	/*v->x = v0->x + (z-v0->z)*(v1->x-v0->x)/(v1->z-v0->z);
	v->y = v0->y + (z-v0->z)*(v1->y-v0->y)/(v1->z-v0->z);
	v->z = z;*/

	//direction vector
	double dx = v1->x - v0->x;
	double dy = v1->y - v0->y;
	double dz = v1->z - v0->z;

	//parametric form
	double xp[] = {v0->x, dx};
	double yp[] = {v0->y, dy};
	double zp[] = {v0->z, dz};

	//finding t
	double t = (z-zp[0])/zp[1];

	//finding point
    v->x = xp[0] + xp[1]*t;
    v->y = yp[0] + yp[1]*t;
    v->z = z; //zp[0] + zp[1]*t;
}

/* This is the first version of the function it's using the sum of areas ...
int checkInside(stl_facet *facet, stl_vertex *vertex){
    stl_facet childFacet1, childFacet2, childFacet3;

    float motherArea = calculateArea(facet);
    childFacet1.vertex[0] = facet->vertex[0];
    childFacet1.vertex[1] = facet->vertex[1];
    childFacet1.vertex[2].x = vertex->x;
    childFacet1.vertex[2].y = vertex->y;
    childFacet1.vertex[2].z = vertex->z;
    float child1Area = calculateArea(&childFacet1);
    childFacet2.vertex[0] = facet->vertex[1];
    childFacet2.vertex[1] = facet->vertex[2];
    childFacet2.vertex[2].x = vertex->x;
    childFacet2.vertex[2].y = vertex->y;
    childFacet2.vertex[2].z = vertex->z;
    float child2Area = calculateArea(&childFacet2);
    childFacet3.vertex[0] = facet->vertex[2];
    childFacet3.vertex[1] = facet->vertex[0];
    childFacet3.vertex[2].x = vertex->x;
    childFacet3.vertex[2].y = vertex->y;
    childFacet3.vertex[2].z = vertex->z;
    float child3Area = calculateArea(&childFacet3);

    if( (child1Area + child2Area + child3Area) <= motherArea) {
        return 1;
    } else {
        return 0;
    }
}*/

user_variables manualVariables(){
    user_variables variables;

    printf("Distance behind the model: "); scanf("%lf", &variables.aft);
    printf("Distance in front the model: "); scanf("%lf", &variables.fore);
    printf("Distance aside of model: "); scanf("%lf", &variables.port);
    printf("Distance below the model: "); scanf("%lf", &variables.depth);
    printf("Height of waterline: "); scanf("%lf", &variables.waterline);
    printf("Position of the centerline: "); scanf("%lf", &variables.center);
    printf("Number of rows and colums: "); scanf("%ud", &variables.N);

    return variables;
}


user_variables autoVariables(){
    user_variables variables;

    variables.fore = 2;      //this is the space in front the model
    variables.aft = 2;       //this is the space behind the model
    variables.port = 0.5;      //this is the space in transverse direction
    variables.depth = 0.5;     //this is the space under the model
    variables.waterline = 0.10; //this is the immersion of the model
    variables.center = 0;    //symmetry Y
    variables.N = 10;

    return variables;
}

user_variables fileVariables(){
//    int precision = 6;
    FILE * settings_file;
    user_variables variables;

    settings_file = fopen("settings.conf","r");
    if(settings_file != NULL){
    //    rewind(settings_file);

        fscanf(settings_file, "%*s %lf \n", &variables.fore);
        fscanf(settings_file, "%*s %lf \n", &variables.aft);
        fscanf(settings_file, "%*s %lf \n", &variables.port);
        fscanf(settings_file, "%*s %lf \n", &variables.starboard);
        fscanf(settings_file, "%*s %lf \n", &variables.depth);
        fscanf(settings_file, "%*s %lf \n", &variables.waterline);
        fscanf(settings_file, "%*s %lf \n", &variables.center);
        fscanf(settings_file, "%*s %ud \n", &variables.N);
        fscanf(settings_file, "%*s %lf \n", &variables.tolerance);
        fscanf(settings_file, "%*s %*s %*s %i \n", &variables.bodies);
        fscanf(settings_file, "%*s %*s %i \n", &variables.transom);
        fscanf(settings_file, "%*s %*s %*s %i \n", &variables.bottom);
        fscanf(settings_file, "%*s %*s %*s %i \n", &variables.sides);
        fclose(settings_file);
        variables.boolmethod = 2;
        variables.factor = 1;
    } else {
        cout << "There is no settings file in current folder";
        abort();
    }

    return variables;
}


user_variables readEntryData(int mode){
    user_variables variables;

    switch(mode)
    {
        case 0: variables = manualVariables(); break;
        case 1: variables = autoVariables(); break;
        case 2: variables = fileVariables(); break;
    }

    return variables;
}

void calculateDomainDimensions(stl_file & stl, user_variables & variables){

    if( variables.bodies == 1 ){
        stl.stats.size_x = ABS(stl.stats.max_x - stl.stats.min_x);                        //size of the model in x
        stl.stats.size_y = ABS(stl.stats.max_y - stl.stats.min_y);                        //size of the model in y
        stl.stats.size_z = ABS(stl.stats.max_z - stl.stats.min_z);                        //size of the model in z
        stl.stats.waterline_height = variables.waterline + stl.stats.min_z;               //height of waterline
        stl.stats.domain_length = variables.fore + variables.aft + stl.stats.size_x;      //length of water domain
        stl.stats.domain_beam = variables.port + stl.stats.max_y;                         //beam of water domain
        stl.stats.domain_heigth = variables.depth + variables.waterline;                  //height of water domain
        variables.dx = (stl.stats.domain_length)/variables.N;
        variables.dy = (stl.stats.domain_beam)/variables.N;
        variables.dz = (stl.stats.domain_heigth)/variables.N;
        stl.stats.bottom_height = stl.stats.waterline_height - variables.N*variables.dz;  //position of the bottom
    } else if ( variables.bodies >= 2 ){
        stl.stats.size_x = ABS(stl.stats.max_x - stl.stats.min_x);                        //size of the model in x
        stl.stats.size_y = ABS(stl.stats.max_y - stl.stats.min_y);                        //size of the model in y
        stl.stats.size_z = ABS(stl.stats.max_z - stl.stats.min_z);                        //size of the model in z
        stl.stats.waterline_height = variables.waterline + stl.stats.min_z;               //height of waterline
        stl.stats.domain_length = variables.fore + variables.aft + stl.stats.size_x;      //length of water domain
        stl.stats.domain_beam = variables.port + fabs(stl.stats.max_y) + fabs(stl.stats.min_y) + variables.starboard;                         //beam of water domain
        stl.stats.domain_heigth = variables.depth + variables.waterline;                  //height of water domain
        variables.dx = (stl.stats.domain_length)/variables.N;
        variables.dy = (stl.stats.domain_beam)/variables.N;
        variables.dz = (stl.stats.domain_heigth)/variables.N;
        stl.stats.bottom_height = stl.stats.waterline_height - variables.N*variables.dz;  //position of the bottom

    } else {
        abort();
    }

    printf("=================== File information ==================== \n");
    printf("x in ( %+4.5f ; %+4.5f )\n", stl.stats.min_x,stl.stats.max_x);
    printf("y in ( %+4.5f ; %+4.5f )\n", stl.stats.min_y,stl.stats.max_y);
    printf("z in ( %+4.5f ; %+4.5f )\n", stl.stats.min_z,stl.stats.max_z);
    printf("Length x:          %4.5f \n", stl.stats.size_x );
    printf("Length y:          %4.5f \n", stl.stats.size_y );
    printf("Length z:          %4.5f \n", stl.stats.size_z );
    printf("========================================================= \n \n");
    printf("=================== Domain information ==================== \n");
    printf("Waterline height:  %4.5f \n", stl.stats.waterline_height );
    printf("Length x:          %4.5f \n", stl.stats.domain_length );
    printf("Length y:          %4.5f \n", stl.stats.domain_beam );
    printf("Length z:          %4.5f \n", stl.stats.domain_heigth );
    printf("========================================================= \n \n");
}

int calculateDomains(int num){
    unsigned int domains = 0;
    switch(num)
    {
        case 1000: domains = 8;  break;
        case 100:  domains = 8;  break;
        case 10:   domains = 8;  break;
        case 1100: domains = 9;  break;
        case 1110: domains = 10; break;
        case 110:  domains = 9;  break;
        case 1010: domains = 9;  break;
        default:   domains = 7;  break;
    }

    return domains;
}

double determinant1(stl_vertex A, stl_vertex B, stl_vertex C){
    return ( ( (A.x * B.y * 1) + (B.x * C.y * 1) + (A.y * 1 * C.x) ) - ((C.x * B.y * 1)+(B.x * A.y * 1)+(C.y * 1 * A.x)));
}

double determinant2(stl_vertex A, stl_vertex B, stl_vertex C){
    return ( ( (A.y * B.z * 1) + (B.y * C.z * 1) + (A.z * 1 * C.y) ) - ((C.y * B.z * 1)+(B.y * A.z * 1)+(C.z * 1 * A.y)));
}

double determinant3(stl_vertex A, stl_vertex B, stl_vertex C){
    return ( ( (A.z * B.x * 1) + (B.z * C.x * 1) + (A.x * 1 * C.z) ) - ((C.z * B.x * 1)+(B.z * A.x * 1)+(C.x * 1 * A.z)));
}

void computeCoefficients(stl_edge &edge){
    edge.A = edge.v2.y - edge.v1.y;
    edge.B = edge.v1.x - edge.v2.x;
    edge.C = edge.A*edge.v1.x + edge.B*edge.v1.y;
}


void defineEdge(stl_edge &edge, stl_vertex ver1, stl_vertex ver2){

    edge.v1.x = ver1.x; edge.v1.y = ver1.y; edge.v1.z = ver1.z;
    edge.v2.x = ver2.x; edge.v2.y = ver2.y; edge.v2.z = ver2.z;
    edge.vec.x = ver2.x - ver1.x;
    edge.vec.y = ver2.y - ver1.y;
    edge.vec.z = ver2.z - ver1.z;
    edge.length = sqrt ( pow(edge.vec.x,2) +  pow(edge.vec.y,2) + pow(edge.vec.z,2) );
    computeCoefficients(edge);
}

void initializeFacet(double z, stl_facet * facet, float factor){
//    int precision = 6;

    long double det1 = determinant1(facet->vertex[0],facet->vertex[1],facet->vertex[2]);
    long double det2 = determinant2(facet->vertex[0],facet->vertex[1],facet->vertex[2]);
    long double det3 = determinant3(facet->vertex[0],facet->vertex[1],facet->vertex[2]);

    defineEdge(facet->edge[0],facet->vertex[0],facet->vertex[1]);
    defineEdge(facet->edge[1],facet->vertex[1],facet->vertex[2]);
    defineEdge(facet->edge[2],facet->vertex[2],facet->vertex[0]);
    facet->used = 0;
    facet->numberOfEdges = 3;

    facet->area = round_double((1/2)*sqrt( pow(det1,2) + pow(det2,2) + pow(det3,2) ),5);
}

void printEdge(stl_edge edge){
    printf(" %.4lf | %.4lf | %.4lf \n %.4lf | %.4lf | %.4lf \n", edge.v1.x, edge.v1.y, edge.v1.z ,edge.v2.x, edge.v2.y, edge.v1.z);
//    printf(" %.10lf .x + %10lf .y + %10lf = 0 \n", edge.A, edge.B, edge.C);
//    printf(" Length: %.10lf  \n", edge.length);
}

void printFacet(stl_facet facet){
    printf("facet ----- \n");
    printf(" %lf | %.4lf | %.4lf \n %.4lf | %.4lf | %.4lf \n %.4lf | %.4lf | %.4lf \n", facet.vertex[0].x, facet.vertex[0].y, facet.vertex[0].z
    ,facet.vertex[1].x, facet.vertex[1].y, facet.vertex[1].z,facet.vertex[2].x, facet.vertex[2].y, facet.vertex[2].z );
    printf("endfacet ----- \n");
}

void printVertex(stl_vertex vertex){
    printf(" %.7lf | %.7lf | %.7lf \n", vertex.x, vertex.y, vertex.z);
}

int ifEqualVertex(stl_vertex A, stl_vertex B){
    unsigned int result;

    if(A.x == B.x && A.y == B.y && A.z == B.z){
        result = 1;
    } else {
        result = 0;
    }

    return result;
}

double calculateDistanceDP(stl_vertex A, stl_vertex B){
    double dx = A.x - B.x;
    double dy = A.y - B.y;
    double dz = A.z - B.z;

    return sqrt(dx*dx + dy*dy + dz*dz);
}

int ifEqualVertex1(stl_vertex A, stl_vertex B, user_variables variables){
    double dist;

    dist = calculateDistanceDP(A,B);

    if( dist < variables.tolerance ){
        return 1;
    } else {
        return 0;
    }
}

int ifMerelyEqualVertex(stl_vertex A, stl_vertex B, double min){
    unsigned int result;
    double pm = 1;
    double dist = calculateDistanceDP(A, B);

    if( (dist < pm && dist >= 0) ){
        result = 1;
    } else {
        result = 0;
    }

    return result;
}

int ifNearFacet(stl_facet F1, stl_facet F2, double min){
    unsigned int r1 = ifMerelyEqualVertex(F1.vertex[0],F2.vertex[0],min);
    unsigned int r2 = ifMerelyEqualVertex(F1.vertex[0],F2.vertex[1],min);
    unsigned int r3 = ifMerelyEqualVertex(F1.vertex[0],F2.vertex[2],min);
    unsigned int s1 = r1+r2+r3;

    unsigned int r4 = ifMerelyEqualVertex(F1.vertex[1],F2.vertex[0],min);
    unsigned int r5 = ifMerelyEqualVertex(F1.vertex[1],F2.vertex[1],min);
    unsigned int r6 = ifMerelyEqualVertex(F1.vertex[1],F2.vertex[2],min);
    unsigned int s2 = r4+r5+r6;

    unsigned int r7 = ifMerelyEqualVertex(F1.vertex[2],F2.vertex[0],min);
    unsigned int r8 = ifMerelyEqualVertex(F1.vertex[2],F2.vertex[1],min);
    unsigned int r9 = ifMerelyEqualVertex(F1.vertex[2],F2.vertex[2],min);
    unsigned int s3 = r7+r8+r9;

    unsigned int sum = s1+s2+s3;
    unsigned int result = 0;

    if(sum > 0 && sum < 3){
        result = 1;
    } else {
        result = 0;
    }

    return result;
}

int ifCommonEdge(stl_facet F1, stl_facet F2){
    unsigned int r1 = ifEqualVertex(F1.vertex[0],F2.vertex[0]);
    unsigned int r2 = ifEqualVertex(F1.vertex[0],F2.vertex[1]);
    unsigned int r3 = ifEqualVertex(F1.vertex[0],F2.vertex[2]);
    unsigned int s1 = r1+r2+r3;

    unsigned int r4 = ifEqualVertex(F1.vertex[1],F2.vertex[0]);
    unsigned int r5 = ifEqualVertex(F1.vertex[1],F2.vertex[1]);
    unsigned int r6 = ifEqualVertex(F1.vertex[1],F2.vertex[2]);
    unsigned int s2 = r4+r5+r6;

    unsigned int r7 = ifEqualVertex(F1.vertex[2],F2.vertex[0]);
    unsigned int r8 = ifEqualVertex(F1.vertex[2],F2.vertex[1]);
    unsigned int r9 = ifEqualVertex(F1.vertex[2],F2.vertex[2]);
    unsigned int s3 = r7+r8+r9;

    unsigned int sum= s1+s2+s3;
    unsigned int result = 0;

    if(sum == 2){
        result = 1;
    } else {
        result = 0;
    }

    return result;
}

int ifCommonVertex(stl_facet F1, stl_facet F2){
    unsigned int r1 = ifEqualVertex(F1.vertex[0],F2.vertex[0]);
    unsigned int r2 = ifEqualVertex(F1.vertex[0],F2.vertex[1]);
    unsigned int r3 = ifEqualVertex(F1.vertex[0],F2.vertex[2]);
    unsigned int s1 = r1+r2+r3;

    unsigned int r4 = ifEqualVertex(F1.vertex[1],F2.vertex[0]);
    unsigned int r5 = ifEqualVertex(F1.vertex[1],F2.vertex[1]);
    unsigned int r6 = ifEqualVertex(F1.vertex[1],F2.vertex[2]);
    unsigned int s2 = r4+r5+r6;

    unsigned int r7 = ifEqualVertex(F1.vertex[2],F2.vertex[0]);
    unsigned int r8 = ifEqualVertex(F1.vertex[2],F2.vertex[1]);
    unsigned int r9 = ifEqualVertex(F1.vertex[2],F2.vertex[2]);
    unsigned int s3 = r7+r8+r9;

    unsigned int sum= s1+s2+s3;
    unsigned int result = 0;

    if(sum == 1){
        result = 1;
    } else {
        result = 0;
    }

    return result;
}

int ifCommonVertex1(stl_edge e1, stl_edge e2){
    unsigned int s1 = 0;
    unsigned int result = 0;

    unsigned int r1 = ifEqualVertex(e1.v1, e2.v1);
    unsigned int r2 = ifEqualVertex(e1.v1, e2.v2);
    unsigned int r3 = ifEqualVertex(e1.v2, e2.v1);
    unsigned int r4 = ifEqualVertex(e1.v2, e2.v2);
    s1 = r1+r2+r3+r4;

    if(s1 > 0){
        result = 1;
    } else {
        result = 0;
    }

    return result;
}

int ifWaterlineEdge1(stl_file stl, stl_facet F){

    if(F.edge[0].v1.z == stl.stats.waterline_height && F.edge[0].v2.z == stl.stats.waterline_height){ return 0; }
    if(F.edge[1].v1.z == stl.stats.waterline_height && F.edge[1].v2.z == stl.stats.waterline_height){ return 1; }
    if(F.edge[2].v1.z == stl.stats.waterline_height && F.edge[2].v2.z == stl.stats.waterline_height){ return 2; }

    return 55;
}

int ifWaterlineEdge(stl_file stl, stl_facet F){
    unsigned int sum = 0;
    unsigned int result = 0;

    if(F.vertex[0].z == stl.stats.waterline_height){ sum++; }
    if(F.vertex[1].z == stl.stats.waterline_height){ sum++; }
    if(F.vertex[2].z == stl.stats.waterline_height){ sum++; }

    if(sum == 2) {
        result = 1;
    } else {
        result = 0;
    }

    return result;
}

int ifWaterlineVertex(stl_file stl, stl_facet F){
    unsigned int result = 0;

    if(F.vertex[0].z == stl.stats.waterline_height){ result = 1; }
    if(F.vertex[1].z == stl.stats.waterline_height){ result = 1; }
    if(F.vertex[2].z == stl.stats.waterline_height){ result = 1; }

    return result;
}

int commonVer(stl_edge E1, stl_edge E2){
    int result = 0;

    if( E1.v1.x == E2.v1.x      &&      E1.v1.y == E2.v1.y      &&      E1.v1.z == E2.v1.z){
        result = 1;
    }
    if( E1.v1.x == E2.v2.x      &&      E1.v1.y == E2.v2.y      &&      E1.v1.z == E2.v2.z){
        result = 1;
    }
    if( E1.v2.x == E2.v2.x      &&      E1.v2.y == E2.v2.y      &&      E1.v2.z == E2.v2.z){
        result = 1;
    }
    if( E1.v2.x == E2.v1.x      &&      E1.v2.y == E2.v1.y      &&      E1.v2.z == E2.v1.z){
        result = 1;
    }

    return result;
}

void addPointToFacet(stl_facet facet, stl_facet &facet1, stl_facet &facet2, stl_facet &facet3, stl_vertex vertex, stl_file stl, user_variables variables){

    facet1.vertex[0].x = facet.vertex[0].x;
    facet1.vertex[0].y = facet.vertex[0].y;
    facet1.vertex[0].z = facet.vertex[0].z;

    facet1.vertex[1].x = facet.vertex[1].x;
    facet1.vertex[1].y = facet.vertex[1].y;
    facet1.vertex[1].z = facet.vertex[1].z;

    facet1.vertex[2].x = vertex.x;
    facet1.vertex[2].y = vertex.y;
    facet1.vertex[2].z = vertex.z;

    calculateNormal(&facet1);
    initializeFacet(stl.stats.waterline_height,&facet1,variables.factor);

    facet2.vertex[0].x = facet1.vertex[0].x;
    facet2.vertex[0].y = facet1.vertex[0].y;
    facet2.vertex[0].z = facet1.vertex[0].z;

    facet2.vertex[1].x = facet1.vertex[2].x;
    facet2.vertex[1].y = facet1.vertex[2].y;
    facet2.vertex[1].z = facet1.vertex[2].z;

    facet2.vertex[2].x = facet.vertex[2].x;
    facet2.vertex[2].y = facet.vertex[2].y;
    facet2.vertex[2].z = facet.vertex[2].z;

    calculateNormal(&facet2);
    initializeFacet(stl.stats.waterline_height,&facet2,variables.factor);

    facet3.vertex[0].x = facet2.vertex[1].x;
    facet3.vertex[0].y = facet2.vertex[1].y;
    facet3.vertex[0].z = facet2.vertex[1].z;

    facet3.vertex[1].x = facet1.vertex[1].x;
    facet3.vertex[1].y = facet1.vertex[1].y;
    facet3.vertex[1].z = facet1.vertex[1].z;

    facet3.vertex[2].x = facet2.vertex[2].x;
    facet3.vertex[2].y = facet2.vertex[2].y;
    facet3.vertex[2].z = facet2.vertex[2].z;

    calculateNormal(&facet3);
    initializeFacet(stl.stats.waterline_height,&facet3,variables.factor);
}

void addEdgeToFacet(stl_facet F, stl_facet& F1, stl_facet& F2, stl_facet& F3, stl_facet& F4, stl_facet& F5, stl_facet& F6, stl_edge edge, stl_file stl, user_variables variables){

    if(checkInside(F, edge.v1) == true ){
        addPointToFacet(F,F1,F2,F3,edge.v1,stl,variables);
    } else {
        F1.used = 5;
        F2.used = 5;
        F3.used = 5;
        F4.used = 5;
        F5.used = 5;
        F6.used = 5;
    }

    if(checkInside(F1, edge.v2) == true ){
        addPointToFacet(F1,F4,F5,F6,edge.v2,stl,variables);
        F1.used = 5;
    }
    if(checkInside(F2, edge.v2) == true){
        addPointToFacet(F2,F4,F5,F6,edge.v2,stl,variables);
        F2.used = 5;
    }
    if(checkInside(F3, edge.v2) == true){
        addPointToFacet(F3,F4,F5,F6,edge.v2,stl,variables);
        F3.used = 5;
    }
}

void splitEdge(stl_edge edge, stl_vertex vertex, stl_edge &edge1, stl_edge &edge2){
    defineEdge(edge1, edge.v1, vertex);
    defineEdge(edge2, vertex, edge.v2);
}

double maxOfTwo(double p1, double p2){
    double max = p1;

    if(max < p2) { max = p2; }

    return max;
}

double minOfTwo(double p1, double p2){
    double min = p1;

    if(min > p2) { min = p2; }

    return min;
}

void intersectionCoords(stl_vertex &v, stl_edge e1, stl_edge e2, stl_file stl){
    double det = e1.A*e2.B - e2.A*e1.B;

    v.x = (e2.B*e1.C - e1.B*e2.C)/det;
    v.y = (e1.A*e2.C - e2.A*e1.C)/det;
    v.z = stl.stats.waterline_height;
}

int ifParallel(stl_edge e1, stl_edge e2){
    double det = e1.A*e2.B - e2.A*e1.B; // tova e vqrno
    if( det == 0){
        return 1;
    } else {
        return 0;
    }
}

int segmentSegments(stl_edge e1, stl_edge e2){
    unsigned int result = 0;
    unsigned int parallel = 0;

    double det = e1.A*e2.B - e2.A*e1.B; // tova e vqrno
    double x,y;

    //finding intersecting point
    if(det == 0){
        parallel = 1;  // tova e vqrno
    } else {
        x = (e2.B*e1.C - e1.B*e2.C)/det; // tova e vqrno
        y = (e1.A*e2.C - e2.A*e1.C)/det; // tova e vqrno
    }

    //finding segments intervals
    //interval za x za pyrviq edge
    double x1_min = minOfTwo(e1.v1.x, e1.v2.x);
    double x1_max = maxOfTwo(e1.v1.x, e1.v2.x);

    //interval za y za pyrviq edge
    double y1_min = minOfTwo(e1.v1.y, e1.v2.y);
    double y1_max = maxOfTwo(e1.v1.y, e1.v2.y);

    //interval za x za vtoriq edge
    double x2_min = minOfTwo(e2.v1.x, e2.v2.x);
    double x2_max = maxOfTwo(e2.v1.x, e2.v2.x);

    //interval za y za vtoriq edge
    double y2_min = minOfTwo(e2.v1.y, e2.v2.y);
    double y2_max = maxOfTwo(e2.v1.y, e2.v2.y);

    if( x > x1_min && x < x1_max && x > x2_min && x < x2_max && y > y1_min && y < y1_max && y > y2_min && y < y2_max ){
        result = 1;
    } else {
        result = 0;
    }

    if(parallel == 1 ){ result = 0; }

    return result;
}

void drawWaterlineLines(stl_vertex vertex, stl_edge &e1, stl_edge &e2, stl_edge &e3, stl_edge &e4, stl_file stl, user_variables variables){
    stl_vertex A1,A2,A3,A4;

    A1.x = stl.stats.min_x - variables.aft;
    A1.y = stl.stats.max_y + variables.port;
    A1.z = stl.stats.waterline_height;

    A2.x = stl.stats.min_x - variables.aft + variables.N*variables.dx;
    A2.y = stl.stats.max_y + variables.port;
    A2.z = stl.stats.waterline_height;

    A3.x = stl.stats.min_x - variables.aft + variables.N*variables.dx;
    A3.y = stl.stats.max_y + variables.port - variables.N*variables.dy;
    A3.z = stl.stats.waterline_height;

    A4.x = stl.stats.min_x - variables.aft;
    A4.y = stl.stats.max_y + variables.port - variables.N*variables.dy;
    A4.z = stl.stats.waterline_height;

    defineEdge(e1, vertex, A1);
    defineEdge(e2, vertex, A2);
    defineEdge(e3, vertex, A3);
    defineEdge(e4, vertex, A4);
}

unsigned int maxOfFour(unsigned int p1, unsigned int p2, unsigned int p3, unsigned int p4){
    unsigned int max = 0;
    if(max < p1){ max = p1; }
    if(max < p2){ max = p2; }
    if(max < p3){ max = p3; }
    if(max < p4){ max = p4; }

    return max;
}

double minOfFour(double p1, double p2, double p3, double p4){
    double min = 999999;
    if(min > p1){ min = p1; }
    if(min > p2){ min = p2; }
    if(min > p3){ min = p3; }
    if(min > p4){ min = p4; }

    return min;
}

int ifCommonEdge1(stl_facet F1, stl_edge edge){
    unsigned int r1 = ifEqualVertex(F1.edge[0].v1,edge.v1);
    unsigned int r2 = ifEqualVertex(F1.edge[0].v2,edge.v2);
    unsigned int s1 = r1+r2;
    if(s1 == 2){ return 1;}

    r1 = ifEqualVertex(F1.edge[0].v2,edge.v1);
    r2 = ifEqualVertex(F1.edge[0].v1,edge.v2);
    s1 = r1+r2;
    if(s1 == 2){ return 1;}

    unsigned int r4 = ifEqualVertex(F1.edge[1].v1,edge.v1);
    unsigned int r5 = ifEqualVertex(F1.edge[1].v2,edge.v2);
    unsigned int s2 = r4+r5;
    if(s2 == 2){ return 1;}

    r4 = ifEqualVertex(F1.edge[1].v2,edge.v1);
    r5 = ifEqualVertex(F1.edge[1].v1,edge.v2);
    s2 = r4+r5;
    if(s2 == 2){ return 1;}

    unsigned int r7 = ifEqualVertex(F1.edge[2].v1,edge.v1);
    unsigned int r8 = ifEqualVertex(F1.edge[2].v2,edge.v2);
    unsigned int s3 = r7+r8;
    if(s3 ==2 ){ return 1;}

    r7 = ifEqualVertex(F1.edge[2].v2,edge.v1);
    r8 = ifEqualVertex(F1.edge[2].v1,edge.v2);
    s3 = r7+r8;
    if(s3 ==2 ){ return 1;}

    return 0;
}

void setFacet(stl_facet &newf, stl_facet oldf, user_variables variables){
    newf.vertex[0].x = oldf.vertex[0].x;
    newf.vertex[0].y = oldf.vertex[0].y;
    newf.vertex[0].z = oldf.vertex[0].z;

    newf.vertex[1].x = oldf.vertex[1].x;
    newf.vertex[1].y = oldf.vertex[1].y;
    newf.vertex[1].z = oldf.vertex[1].z;

    newf.vertex[2].x = oldf.vertex[2].x;
    newf.vertex[2].y = oldf.vertex[2].y;
    newf.vertex[2].z = oldf.vertex[2].z;

    calculateNormal(&newf);
    initializeFacet(5, &newf, variables.factor);
}

int ifEqualEdge(stl_edge e1, stl_edge e2){
    if( ifEqualVertex(e1.v1,e2.v1) && ifEqualVertex(e1.v2,e2.v2)) {
        return 1;
    }
    if( ifEqualVertex(e1.v2,e2.v1) && ifEqualVertex(e1.v1,e2.v2)) {
        return 1;
    }

    return 0;
}

bool edgeComparison(stl_edge e1, stl_edge e2){
    return e1.v1.x <= e2.v1.x && e1.v2.x <= e2.v2.x;
}

void initializeVertex(stl_vertex oldv, stl_vertex &newv){
    newv.x = oldv.x;
    newv.y = oldv.y;
    newv.z = oldv.z;
}

int returnUncommonVertex(stl_facet facet, stl_edge edge, user_variables variables){
    double dist1,dist2;
    int result = 99;

    dist1 = calculateDistanceDP(facet.vertex[0],edge.v1);
    dist2 = calculateDistanceDP(facet.vertex[0],edge.v2);
    if( dist1 > variables.tolerance && dist2 > variables.tolerance ){
        result = 0;
    }

    dist1 = calculateDistanceDP(facet.vertex[1],edge.v1);
    dist2 = calculateDistanceDP(facet.vertex[1],edge.v2);
    if( dist1 > variables.tolerance && dist2 > variables.tolerance ){
        result = 1;
    }

    dist1 = calculateDistanceDP(facet.vertex[2],edge.v1);
    dist2 = calculateDistanceDP(facet.vertex[2],edge.v2);
    if( dist1 > variables.tolerance && dist2 > variables.tolerance ){
        result = 2;
    }

    return result;
}

int between(stl_edge edge, stl_vertex vertex){
    if( edge.v1.y < vertex.y && edge.v2.y > vertex.y ){
        return 12;
    }
    if( edge.v2.y < vertex.y && edge.v1.y > vertex.y ){
        return 21;
    }

    return 0;
}

int between1(stl_edge edge, stl_vertex vertex){
    if( edge.v1.z < vertex.z && edge.v2.z > vertex.z ){
        return 12;
    }
    if( edge.v2.z < vertex.z && edge.v1.z > vertex.z ){
        return 21;
    }

    return 0;
}
