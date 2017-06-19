#ifndef HEADER_H_
#define HEADER_H_

#define STL_MAX(A,B) ((A)>(B)? (A):(B))
#define STL_MIN(A,B) ((A)<(B)? (A):(B))
#define ABS(X)  ((X) < 0 ? -(X) : (X))
#define ASCII_LINES_PER_FACET 7

typedef struct{ //tuka ...
	double x;
	double y;
	double z;
}stl_vertex;

typedef struct{
	double x;
	double y;
}plane_vector;

typedef struct{
    double x;
    double y;
    double z;
}stl_vector;

typedef struct{
	double x;
	double y;
	double z;
}stl_normal;

typedef struct{
	stl_vertex v1;
	stl_vertex v2;
	int number;
    stl_vector vec;
    double length;
    double A,B,C;
    int Tindex;
}stl_edge;

typedef struct{
	stl_normal normal;
	stl_vertex vertex[3];
	long double area;
	stl_edge edge[10];
	int neighbour[2];
	unsigned int used;
	unsigned int index;
	unsigned int Tindex;
	unsigned int numberOfEdges;
}stl_facet;

typedef struct{
	unsigned long number_facets,number_facets_end;
	unsigned long number_domain_facets;
	unsigned long number_lines;
	double waterline_height,bottom_height;
	unsigned long number_waterline_hull_vertices,number_waterline_domain_vertices;
	double max_x,max_y,max_z,min_x,min_y,min_z;
	double size_x,size_y,size_z;
	double domain_length,domain_heigth,domain_beam;
	int numberOfDomains;
	double wettedArea;
}stl_stats;

typedef struct{
	FILE * in_file;
	FILE * out_file;
	FILE * prop_file;
	stl_stats stats;
	unsigned long fileSize;
}stl_file;

typedef struct{
    double aft,fore,port,depth,waterline,center,starboard;
    unsigned int N;
    double dx,dy,dz;
    double factor;
    double tolerance;
    int bodies,boolmethod,transom,sides,bottom;
}user_variables;



#endif /* HEADER_H_ */
