#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <vector>
#include "header.h"
#include "functions.h"

using namespace std;

void generateCenter(stl_file stl, user_variables variables, vector<stl_facet> &center_facets);
void generateBottom(stl_file stl, user_variables variables, vector<stl_facet> &bottom_facets);
void generateInlet(stl_file stl, user_variables variables, vector<stl_facet> &inlet_facets);
void generatePort(stl_file stl, user_variables variables, vector<stl_facet> &port_facets);
void generateOutflow(stl_file stl, user_variables variables, vector<stl_facet> &outflow_facets);
void writeFile(stl_file stl,user_variables variables, vector<stl_facet> v1, vector<stl_facet> v2, vector<stl_facet> v3, vector<stl_facet> v4, vector<stl_facet> v5, vector<stl_facet> v6, vector<stl_facet> v7 );
void writeVectorToFile(FILE fpp, vector<stl_facet> vector);
void deleteUpperFacets(stl_file stl, user_variables variables, vector<stl_facet> entry, vector<stl_facet> &result);
void generateWaterline(stl_file stl, user_variables variables, vector<stl_facet> &waterline_facets);
void readFile(stl_file &stl, vector<stl_facet> &facets, user_variables variables);
void initializeEdge(unsigned int num, stl_vertex ver1, stl_vertex ver2, vector<stl_edge> &edges);
void writeVectorDomain(FILE *fpp, vector<stl_facet> vector, int domain);
void writePropFile(stl_file stl,user_variables variables, vector<stl_facet> v1, vector<stl_facet> v2, vector<stl_facet> v3, vector<stl_facet> v4, vector<stl_facet> v5, vector<stl_facet> v6, vector<stl_facet> v7 );
void printEdge(stl_edge edge);
void cleanFile();
int topologyDetection(vector<stl_facet> &hull_facets, vector<stl_facet> &fs, vector<stl_facet> &fb, vector<stl_facet> &tr, user_variables variables);
void checkFacets(stl_file stl, vector<stl_facet> facets);
void findWaterlineEdges(stl_file stl, vector<stl_facet> hull_facets, vector<stl_edge> &edges, user_variables variables);
void findCenterlineEdges(user_variables variables, vector<stl_facet> hull_facets, vector<stl_edge> &edges);
void cleanWaterline(vector<stl_facet> &mesh, vector<stl_edge> edges, user_variables variables, stl_file stl);
void sortEdges(vector<stl_edge> start, vector<stl_edge> &final, user_variables variables);
void checkEdges(vector<stl_edge> edges);
void copyVectorOfEdges(vector<stl_edge> vec1, vector<stl_edge> &vec2 );
void copyVectorOfFacets(vector<stl_facet> vec1, vector<stl_facet> &vec2 );
void copyVectorOfVerticies(vector<stl_vertex> vec1, vector<stl_vertex> &vec2 );
void deleteCenterFacets(stl_file stl, user_variables variables, vector<stl_facet> entry, vector<stl_facet> &result);
void monohull_1(stl_file &stl, user_variables &variables);
void catamaran(stl_file& stl, user_variables &variables);
void catamaran_1(stl_file &stl, user_variables& variables);
stl_edge swapEdge(stl_edge edge);
void recalculateNormals( vector<stl_facet> &facets );

int main(){

    int mode = 2; // 1 - auto , 0 - manual, 2 - file
    stl_file stl;

    user_variables variables = readEntryData(mode);

    if( variables.bodies == 1 ){
            monohull_1(stl,variables);
    }
    if( variables.bodies == 2 ){
            catamaran_1(stl,variables);
    }

	return 0;
}


void writePointsToFile(FILE *fpp, vector<stl_vertex> vector){
    long long a,b;
    if(!vector.empty()){
        for(unsigned long i = 0; i < vector.size(); i++){
            a = vector.at(i).x*1000000;
            b = vector.at(i).y*1000000;
            fprintf(fpp, "%lld %lld \n", a, b);
        }
    } else {
        cout << "You were trying to write an empty array!" << endl;
    }
}

void writePointsToFile1(FILE *fpp, vector<stl_vertex> vector){
    long long a,b;
    if(!vector.empty()){
        for(unsigned long i = 0; i < vector.size(); i++){
            a = vector.at(i).x*1000000;
            b = vector.at(i).z*1000000;
            fprintf(fpp, "%lld %lld \n", a, b);
        }
    } else {
        cout << "You were trying to write an empty array!" << endl;
    }
}

void writePointsToFile2(FILE *fpp, vector<stl_vertex> vector){
    long long a,b;
    fprintf(fpp, "[ ");
    if(!vector.empty()){
        for(unsigned long i = 0; i < vector.size(); i++){
            a = vector.at(i).x*1000000;
            b = vector.at(i).y*1000000;
            fprintf(fpp, "[ %lld , %lld ], ",a ,b );
        }
    } else {
        cout << "You were trying to write an empty array!" << endl;
    }
    fprintf(fpp, "] ");
}

void EdgesToPoints(vector<stl_edge> edges, vector<stl_vertex> &points){
    for(unsigned int i=0;i<edges.size();i++){
        points.push_back(edges.at(i).v1);
        points.push_back(edges.at(i).v2);
    }

   for(unsigned int i=0;i<points.size();i++){
        for(unsigned int j=0;j<points.size();j++){
            if( ifEqualVertex(points.at(i), points.at(j)) && i != j){
                points.erase(points.begin()+j); j=0;
            }
            if( fabs(calculateDistanceDP(points.at(i),points.at(j))) < 0.0001 && i != j ){
               points.erase(points.begin()+j); j=0;
            }
        }
    }
}

void reverseVector(vector<stl_vertex> &vec){
    vector<stl_vertex> temp;
    for(unsigned int i=1;i<=vec.size();i++){
        temp.push_back(vec.at(vec.size()-i));
    }
    vec.clear();
    for(unsigned int i=0;i<temp.size();i++){
        vec.push_back(temp.at(i));
    }
}

double f(long long p){
    return p;
}

void readFile2(FILE *f1,vector<stl_facet> &waterline_facets, stl_file stl, user_variables variables){
    stl_facet facet;
    long long v[3][2];
    double d[3][2];

    while(fscanf(f1, "%lld %lld %lld %lld %lld %lld", &v[0][0], &v[0][1], &v[1][0], &v[1][1], &v[2][0], &v[2][1]) != EOF){
        for(unsigned int i=0;i<3;i++){
            for(unsigned int j=0;j<2;j++){
                d[i][j] = f(v[i][j]) / 1000000;
            }
        }
        facet.vertex[0].x = d[0][0];
        facet.vertex[0].y = d[0][1];
        facet.vertex[0].z = stl.stats.waterline_height;
        facet.vertex[1].x = d[1][0];
        facet.vertex[1].y = d[1][1];
        facet.vertex[1].z = stl.stats.waterline_height;
        facet.vertex[2].x = d[2][0];
        facet.vertex[2].y = d[2][1];
        facet.vertex[2].z = stl.stats.waterline_height;
        calculateNormal(&facet);
        initializeFacet(stl.stats.waterline_height,&facet,variables.factor);
        waterline_facets.push_back(facet);
    }
    fclose(f1);
}

void readFile2a(FILE *f1,vector<stl_facet> &waterline_facets, stl_file stl, user_variables variables){
    stl_facet facet;
    long long v[3][2];
    double d[3][2];

    while(fscanf(f1, "%lld %lld %lld %lld %lld %lld", &v[0][0], &v[0][1], &v[1][0], &v[1][1], &v[2][0], &v[2][1]) != EOF){
        for(unsigned int i=0;i<3;i++){
            for(unsigned int j=0;j<2;j++){
                d[i][j] = f(v[i][j]) / 1000000;
            }
        }
        facet.vertex[0].x = d[0][0];
        facet.vertex[0].y = variables.center;
        facet.vertex[0].z = d[0][1];
        facet.vertex[1].x = d[1][0];
        facet.vertex[1].y = variables.center;
        facet.vertex[1].z = d[1][1];
        facet.vertex[2].x = d[2][0];
        facet.vertex[2].y = variables.center;
        facet.vertex[2].z = d[2][1];
        calculateNormal(&facet);
        initializeFacet(stl.stats.waterline_height,&facet,variables.factor);
        waterline_facets.push_back(facet);
    }
    fclose(f1);
}

void doInlet(vector<stl_facet> &inlet, stl_vertex foremost, stl_edge edge, stl_file stl, user_variables variables, stl_vertex &intersection){
    int ID;
    stl_facet f1,f2;

    initializeVertex(inlet.front().vertex[0],intersection);

    for( unsigned int i=0; i<inlet.size();i++ ){
        if( ifCommonEdge1(inlet.at(i), edge) ){
            ID = i;
        }
    }

    if( ifEqualEdge(inlet.at(ID).edge[0],edge) ){
        linearInterpolation3D_y(&intersection, &inlet.at(ID).edge[0].v1, &inlet.at(ID).edge[0].v2, foremost.y );
        f1.vertex[0] = inlet.at(ID).vertex[0];
        f1.vertex[1] = intersection;
        f1.vertex[2] = inlet.at(ID).vertex[2];
        calculateNormal(&f1);
        initializeFacet(stl.stats.waterline_height,&f1,variables.factor);


        f2.vertex[0] = inlet.at(ID).vertex[2];
        f2.vertex[1] = intersection;
        f2.vertex[2] = inlet.at(ID).vertex[1];
        calculateNormal(&f2);
        initializeFacet(stl.stats.waterline_height,&f2,variables.factor);

    }
    if( ifEqualEdge(inlet.at(ID).edge[1],edge) ){
        linearInterpolation3D_y(&intersection, &inlet.at(ID).edge[1].v1, &inlet.at(ID).edge[1].v2, foremost.y );
        f1.vertex[0] = inlet.at(ID).vertex[0];
        f1.vertex[1] = inlet.at(ID).vertex[1];
        f1.vertex[2] = intersection;
        calculateNormal(&f1);
        initializeFacet(stl.stats.waterline_height,&f1,variables.factor);

        f2.vertex[0] = f1.vertex[2];
        f2.vertex[1] = inlet.at(ID).vertex[2];
        f2.vertex[2] = f1.vertex[0];
        calculateNormal(&f2);
        initializeFacet(stl.stats.waterline_height,&f2,variables.factor);
    }
    if( ifEqualEdge(inlet.at(ID).edge[2],edge) ){
        linearInterpolation3D_y(&intersection, &inlet.at(ID).edge[2].v1, &inlet.at(ID).edge[2].v2, foremost.y );
        f1.vertex[0] = inlet.at(ID).vertex[0];
        f1.vertex[1] = inlet.at(ID).vertex[1];
        f1.vertex[2] = intersection;
        calculateNormal(&f1);
        initializeFacet(stl.stats.waterline_height,&f1,variables.factor);

        f2.vertex[0] = f1.vertex[2];
        f2.vertex[1] = f1.vertex[1];
        f2.vertex[2] = inlet.at(ID).vertex[2];
        calculateNormal(&f2);
        initializeFacet(stl.stats.waterline_height,&f2,variables.factor);
    }

    inlet.erase(inlet.begin()+ID);
    inlet.push_back(f1);
    inlet.push_back(f2);
}

void doOutflow(vector<stl_facet> &outflow, stl_vertex foremost, stl_edge edge, stl_file stl, user_variables variables, stl_vertex &intersection){
    int ID;
    stl_facet f1,f2;

    for( unsigned int i=0; i<outflow.size();i++ ){
        if( ifCommonEdge1(outflow.at(i), edge) ){
            ID = i;
        }
    }

    if( ifEqualEdge(outflow.at(ID).edge[0],edge) ){
        linearInterpolation3D_y(&intersection, &outflow.at(ID).edge[0].v1, &outflow.at(ID).edge[0].v2, foremost.y );
        f1.vertex[0] = outflow.at(ID).vertex[0];
        f1.vertex[1] = intersection;
        f1.vertex[2] = outflow.at(ID).vertex[2];
        calculateNormal(&f1);
        initializeFacet(stl.stats.waterline_height,&f1,variables.factor);

        f2.vertex[0] = f1.vertex[2];
        f2.vertex[1] = f1.vertex[1];
        f2.vertex[2] = outflow.at(ID).vertex[1];
        calculateNormal(&f2);
        initializeFacet(stl.stats.waterline_height,&f2,variables.factor);
    }
    if( ifEqualEdge(outflow.at(ID).edge[1],edge) ){
        linearInterpolation3D_y(&intersection, &outflow.at(ID).edge[1].v1, &outflow.at(ID).edge[1].v2, foremost.y );
        f1.vertex[0] = outflow.at(ID).vertex[0];
        f1.vertex[1] = outflow.at(ID).vertex[1];
        f1.vertex[2] = intersection;
        calculateNormal(&f1);
        initializeFacet(stl.stats.waterline_height,&f1,variables.factor);

        f2.vertex[0] = f1.vertex[2];
        f2.vertex[1] = outflow.at(ID).vertex[2];
        f2.vertex[2] = f1.vertex[0];
        calculateNormal(&f2);
        initializeFacet(stl.stats.waterline_height,&f2,variables.factor);
    }
    if( ifEqualEdge(outflow.at(ID).edge[2],edge) ){
        linearInterpolation3D_y(&intersection, &outflow.at(ID).edge[2].v1, &outflow.at(ID).edge[2].v2, foremost.y );
        f1.vertex[0] = outflow.at(ID).vertex[0];
        f1.vertex[1] = outflow.at(ID).vertex[1];
        f1.vertex[2] = intersection;
        calculateNormal(&f1);
        initializeFacet(stl.stats.waterline_height,&f1,variables.factor);

        f2.vertex[0] = f1.vertex[2];
        f2.vertex[1] = f1.vertex[1];
        f2.vertex[2] = outflow.at(ID).vertex[2];
        calculateNormal(&f2);
        initializeFacet(stl.stats.waterline_height,&f2,variables.factor);
    }

    outflow.erase(outflow.begin()+ID);
    outflow.push_back(f1);
    outflow.push_back(f2);
}

void doHull(vector<stl_facet> &hull, stl_vertex foremost, stl_edge edge, stl_file stl, user_variables variables, stl_vertex &intersection){
    int ID;
    stl_facet f1,f2;

    for( unsigned int i=0; i<hull.size();i++ ){
        if( ifCommonEdge1(hull.at(i), edge) ){
            ID = i;
            break;
        }
    }

    if( ifEqualEdge(hull.at(ID).edge[0],edge) ){
        linearInterpolation3D_y(&intersection, &hull.at(ID).edge[0].v1, &hull.at(ID).edge[0].v2, foremost.y );
        f1.vertex[0] = hull.at(ID).vertex[0];
        f1.vertex[1] = intersection;
        f1.vertex[2] = hull.at(ID).vertex[2];
        calculateNormal(&f1);
        initializeFacet(stl.stats.waterline_height,&f1,variables.factor);

        f2.vertex[0] = f1.vertex[2];
        f2.vertex[1] = f1.vertex[1];
        f2.vertex[2] = hull.at(ID).vertex[1];
        calculateNormal(&f2);
        initializeFacet(stl.stats.waterline_height,&f2,variables.factor);
    }
    if( ifEqualEdge(hull.at(ID).edge[1],edge) ){
        linearInterpolation3D_y(&intersection, &hull.at(ID).edge[1].v1, &hull.at(ID).edge[1].v2, foremost.y );
        f1.vertex[0] = hull.at(ID).vertex[0];
        f1.vertex[1] = hull.at(ID).vertex[1];
        f1.vertex[2] = intersection;
        calculateNormal(&f1);
        initializeFacet(stl.stats.waterline_height,&f1,variables.factor);

        f2.vertex[0] = f1.vertex[2];
        f2.vertex[1] = hull.at(ID).vertex[2];
        f2.vertex[2] = f1.vertex[0];
        calculateNormal(&f2);
        initializeFacet(stl.stats.waterline_height,&f2,variables.factor);
    }
    if( ifEqualEdge(hull.at(ID).edge[2],edge) ){
        linearInterpolation3D_y(&intersection, &hull.at(ID).edge[2].v1, &hull.at(ID).edge[2].v2, foremost.y );
        f1.vertex[0] = hull.at(ID).vertex[0];
        f1.vertex[1] = hull.at(ID).vertex[1];
        f1.vertex[2] = intersection;
        calculateNormal(&f1);
        initializeFacet(stl.stats.waterline_height,&f1,variables.factor);

        f2.vertex[0] = f1.vertex[2];
        f2.vertex[1] = f1.vertex[1];
        f2.vertex[2] = hull.at(ID).vertex[2];
        calculateNormal(&f2);
        initializeFacet(stl.stats.waterline_height,&f2,variables.factor);
    }

    hull.erase(hull.begin()+ID);
    hull.push_back(f1);
    hull.push_back(f2);
}

void removeDuplicateEdges(vector<stl_edge> edges){
    for(unsigned int i=0;i<edges.size();i++){
        for( unsigned int j=0;j<edges.size();j++ ){
            if( i==j ) {
                continue;
            } else if( ifEqualEdge(edges.at(i),edges.at(j)) != 0){
                edges.erase(edges.begin()+j);j--;
            }
        }
    }
}

void connectWaterline(vector<stl_edge> &hull_edges, vector<stl_edge> &domain_edges, user_variables variables, vector<stl_facet> &waterline_facets, stl_file stl,
                        vector<stl_facet> &inlet, vector<stl_facet> &outflow, vector<stl_facet> &hull_facets, vector<stl_facet> &port_facets,
                            vector<stl_facet> center){

    printf("--------> Triangulating Waterline \n");

    double max_x = -9999999;
    double min_x = 9999999;
    stl_vertex foremost,fore_domain, aft_domain, aft_hull;
    stl_edge e1,e2;
    initializeVertex(hull_edges.front().v1, foremost);
    initializeVertex(hull_edges.front().v2, foremost);

    for(unsigned int i=0;i<hull_edges.size();i++){
        if(hull_edges.at(i).v1.x > max_x && hull_edges.at(i).v1.z == stl.stats.waterline_height){
            max_x = hull_edges.at(i).v1.x;
            initializeVertex(hull_edges.at(i).v1, foremost);
        }
        if(hull_edges.at(i).v2.x > max_x && hull_edges.at(i).v2.z == stl.stats.waterline_height){
            max_x = hull_edges.at(i).v2.x;
            initializeVertex(hull_edges.at(i).v2, foremost);
        }
    }

    for(unsigned int i=0;i<hull_edges.size();i++){
        if(hull_edges.at(i).v1.x < min_x && hull_edges.at(i).v1.z == stl.stats.waterline_height ){
            min_x = hull_edges.at(i).v1.x;
        }
        if(hull_edges.at(i).v2.x < min_x && hull_edges.at(i).v2.z == stl.stats.waterline_height ){
            min_x = hull_edges.at(i).v2.x;
        }
    }

    initializeVertex(domain_edges.front().v1,fore_domain);
    for(unsigned int i=0;i<domain_edges.size();i++){
        if( domain_edges.at(i).v1.x > max_x && domain_edges.at(i).v2.x > max_x  ){
            if( between(domain_edges.at(i), foremost) != 0 ){
                doInlet(inlet, foremost, domain_edges.at(i), stl, variables, fore_domain);
                break;
            }
        } else {
            continue;
        }
    }

    initializeVertex(domain_edges.front().v1,aft_domain);
    for(unsigned int i=0;i<domain_edges.size();i++){
        if( domain_edges.at(i).v1.x < min_x && domain_edges.at(i).v2.x < min_x  ){
            if( between(domain_edges.at(i), foremost) != 0 ){
                doOutflow(outflow, foremost, domain_edges.at(i), stl, variables, aft_domain);
                break;
            }
        } else {
            continue;
        }
    }

    initializeVertex(hull_edges.front().v1,aft_hull);
    for(unsigned int i=0;i<hull_edges.size();i++){
        if( hull_edges.at(i).v1.x < max_x && hull_edges.at(i).v2.x < max_x  ){
            if( between(hull_edges.at(i), foremost) != 0 ){
                doHull(hull_facets, foremost, hull_edges.at(i), stl, variables, aft_hull);
                break;
            }
            if( hull_edges.at(i).v1.y == foremost.y ){
                initializeVertex(hull_edges.at(i).v1,aft_hull);
                break;
            }
            if( hull_edges.at(i).v2.y == foremost.y ){
                initializeVertex(hull_edges.at(i).v2,aft_hull);
                break;
            }
        } else {
            continue;
        }
    }

    domain_edges.clear();
    hull_edges.clear();

    defineEdge(e1,foremost,fore_domain);
    defineEdge(e2,aft_domain,aft_hull);

    domain_edges.push_back(e1);
    domain_edges.push_back(e2);
    findWaterlineEdges(stl,hull_facets,domain_edges, variables);
    findWaterlineEdges(stl,port_facets,domain_edges, variables);
    findWaterlineEdges(stl,outflow,domain_edges, variables);
    findWaterlineEdges(stl,inlet,domain_edges, variables);
    findWaterlineEdges(stl,center,domain_edges, variables);

    vector<stl_edge> h1,h2;


        for( unsigned int i=0;i<domain_edges.size();i++ ){
            if( domain_edges.at(i).v1.y > foremost.y && domain_edges.at(i).v2.y > foremost.y){
                h1.push_back(domain_edges.at(i)); continue;
            }
            if( domain_edges.at(i).v1.y == foremost.y && domain_edges.at(i).v2.y > foremost.y){
                h1.push_back(domain_edges.at(i)); continue;
            }
            if( domain_edges.at(i).v1.y > foremost.y && domain_edges.at(i).v2.y == foremost.y){
                h1.push_back(domain_edges.at(i)); continue;
            }
            if( domain_edges.at(i).v1.y < foremost.y && domain_edges.at(i).v2.y < foremost.y){
                h2.push_back(domain_edges.at(i)); continue;
            }
            if( domain_edges.at(i).v1.y == foremost.y && domain_edges.at(i).v2.y < foremost.y){
                h2.push_back(domain_edges.at(i)); continue;
            }
            if( domain_edges.at(i).v1.y < foremost.y && domain_edges.at(i).v2.y == foremost.y){
                h2.push_back(domain_edges.at(i)); continue;
            }
            if( domain_edges.at(i).v1.y == foremost.y && domain_edges.at(i).v2.y == foremost.y){
                h1.push_back(domain_edges.at(i));
                h2.push_back(domain_edges.at(i)); swapEdge(h2.back());
                continue;
            }
        }

    domain_edges.clear();

    FILE *f1, *f2;
    vector<stl_vertex> h1_points, h2_points;
    vector<stl_edge> h1_final, h2_final;

    sortEdges(h1, h1_final, variables); h1.clear();
    sortEdges(h2, h2_final, variables); h2.clear();


    EdgesToPoints(h1_final,h1_points); h1_final.clear();
    EdgesToPoints(h2_final,h2_points); h2_final.clear();

 //   reverseVector(h1_points);
    reverseVector(h2_points);

    f1 = fopen("./i.1", "w"); writePointsToFile(f1, h1_points); fclose(f1); h1_points.clear();
    f2 = fopen("./i.2", "w"); writePointsToFile(f2, h2_points); fclose(f2); h2_points.clear();

    system("./tri <i.1> o.1");
    system("./tri <i.2> o.2");

    f1 = fopen("./o.1", "r");
    f2 = fopen("./o.2", "r");

    readFile2(f1, waterline_facets,stl, variables);
    readFile2(f2, waterline_facets,stl, variables);
}

void catamaran_1(stl_file &stl, user_variables &variables){
    printf("==== Starting procedure sequences for catamaran ==== \n \n");

    vector<stl_facet> facets;
    readFile(stl,facets,variables);

    calculateDomainDimensions(stl,variables);

    vector<stl_facet> center_facets;
    generateCenter(stl,variables,center_facets);

    vector<stl_facet> bottom_facets;
    generateBottom(stl,variables,bottom_facets);

    vector<stl_facet> inlet_facets;
    generateInlet(stl,variables,inlet_facets);

    vector<stl_facet> port_facets;
    generatePort(stl,variables,port_facets);

    vector<stl_facet> outflow_facets;
    generateOutflow(stl,variables,outflow_facets);

    vector<stl_facet> hull_facets, waterline_facets;
    vector<stl_edge> waterline_edges_hull;
    vector<stl_edge> waterline_edges_domain;

    //deleting of waterline and centreline intersected facets
    deleteUpperFacets(stl,variables,facets,hull_facets);

    //finding edges lieing on waterline and centerline
    findWaterlineEdges(stl,hull_facets,waterline_edges_hull, variables); // from curved hull
    findWaterlineEdges(stl,inlet_facets,waterline_edges_domain, variables); // from inlet
    findWaterlineEdges(stl,outflow_facets,waterline_edges_domain, variables); // from outflow
    findWaterlineEdges(stl,center_facets,waterline_edges_domain, variables); // from centerline
    findWaterlineEdges(stl,port_facets,waterline_edges_domain, variables); // from port

    connectWaterline(waterline_edges_hull, waterline_edges_domain, variables, waterline_facets, stl,
                     inlet_facets, outflow_facets, hull_facets, port_facets, center_facets);

    //writing new files
    writeFile(stl, variables , center_facets, bottom_facets, port_facets, inlet_facets, outflow_facets, waterline_facets, hull_facets );
    writePropFile(stl, variables, center_facets, bottom_facets, port_facets, inlet_facets, outflow_facets, waterline_facets, hull_facets);

    system("./admesh -nfi --tolerance=0.000001 --iterations=5 model-v2.stl -a model-v3.stl");
    remove("o.2");
    remove("o.1");
    remove("i.1");
    remove("i.2");
}

void triangulateWaterline(vector<stl_edge> &domain_edges, vector<stl_edge> &hull_edges, vector<stl_facet> &waterline_facets, user_variables variables,stl_file stl,
                            vector<stl_facet> inlet, vector<stl_facet> outflow, vector<stl_facet> hull, vector<stl_facet> port){
    double max_x = -9999999;
    double min_x = 9999999;
    stl_vertex foremost, aftmost, fbound, abound;

    printf("--------> Trinagulating Waterline ");
    initializeVertex(hull_edges.front().v1, foremost);
    initializeVertex(hull_edges.front().v1, aftmost);
    initializeVertex(domain_edges.front().v1, fbound);
    initializeVertex(domain_edges.front().v1, abound);

    for( unsigned int i=0; i< hull_edges.size(); i++ ) {
        if( hull_edges.at(i).v1.x > max_x && fabs(hull_edges.at(i).v1.y - variables.center) < 0.000001){
            max_x = hull_edges.at(i).v1.x;
            initializeVertex(hull_edges.at(i).v1, foremost);
        }
        if( hull_edges.at(i).v2.x > max_x && fabs(hull_edges.at(i).v2.y - variables.center) < 0.000001){
            max_x = hull_edges.at(i).v2.x;
            initializeVertex(hull_edges.at(i).v2, foremost);
        }
        if( hull_edges.at(i).v1.x < min_x && fabs(hull_edges.at(i).v1.y - variables.center) < 0.000001){
            min_x = hull_edges.at(i).v1.x;
            initializeVertex(hull_edges.at(i).v1, aftmost);
        }
        if( hull_edges.at(i).v2.x < min_x && fabs(hull_edges.at(i).v1.y - variables.center) < 0.000001){
            min_x = hull_edges.at(i).v2.x;
            initializeVertex(hull_edges.at(i).v2, aftmost);
        }
    }

    for( unsigned int i=0; i< domain_edges.size(); i++ ) {
        if( domain_edges.at(i).v1.x < min_x && domain_edges.at(i).v1.y == variables.center ){
            initializeVertex(domain_edges.at(i).v1, abound);
        }
        if( domain_edges.at(i).v2.x < min_x && domain_edges.at(i).v2.y == variables.center ){
            initializeVertex(domain_edges.at(i).v2, abound);
        }
        if( domain_edges.at(i).v1.x > max_x && domain_edges.at(i).v1.y == variables.center ){
            initializeVertex(domain_edges.at(i).v1, fbound);
        }
        if( domain_edges.at(i).v2.x > max_x && domain_edges.at(i).v2.y == variables.center ){
            initializeVertex(domain_edges.at(i).v2, fbound);
        }
    }

    stl_edge fedge, aedge;

    defineEdge(fedge,foremost,fbound);
    defineEdge(aedge,abound,aftmost);

    domain_edges.clear();
    hull_edges.clear();

    findWaterlineEdges(stl, port, domain_edges, variables);
    findWaterlineEdges(stl, inlet, domain_edges, variables);
    findWaterlineEdges(stl, outflow, domain_edges, variables);
    findWaterlineEdges(stl, hull, domain_edges, variables);
    domain_edges.push_back(fedge);
    domain_edges.push_back(aedge);

    vector<stl_vertex> points;
    vector<stl_edge> edges;
    FILE *f1;

    sortEdges(domain_edges,edges,variables); domain_edges.clear();
    EdgesToPoints(edges, points); edges.clear();

    f1 = fopen("./i.1", "w"); writePointsToFile(f1, points); fclose(f1); points.clear();

    system("./tri <i.1> o.1");

    f1 = fopen("./o.1", "r");

    readFile2(f1, waterline_facets,stl, variables);
    printf("--------> OK \n");
}

void triangulateCenterline(vector<stl_edge> &domain_edges, vector<stl_edge> &hull_edges, vector<stl_facet> &waterline_facets, user_variables variables,stl_file stl){
    double max_x = -9999999;
    double min_x = 9999999;
    stl_vertex foremost, aftmost, fbound, abound;

    initializeVertex(hull_edges.front().v1, foremost);
    initializeVertex(hull_edges.front().v1, aftmost);
    initializeVertex(domain_edges.front().v1, abound);
    initializeVertex(domain_edges.front().v1, abound);

    printf("--------> Trinagulating Centerline ");

    for( unsigned int i=0; i< hull_edges.size(); i++ ) {
        if( hull_edges.at(i).v1.z == stl.stats.waterline_height && hull_edges.at(i).v1.x > max_x ){
            max_x = hull_edges.at(i).v1.x;
            initializeVertex(hull_edges.at(i).v1, foremost);
        }
        if( hull_edges.at(i).v2.z == stl.stats.waterline_height && hull_edges.at(i).v2.x > max_x ){
            max_x = hull_edges.at(i).v2.x;
            initializeVertex(hull_edges.at(i).v2, foremost);
        }
        if( hull_edges.at(i).v1.z == stl.stats.waterline_height && hull_edges.at(i).v1.x < min_x ){
            min_x = hull_edges.at(i).v1.x;
            initializeVertex(hull_edges.at(i).v1, aftmost);
        }
        if( hull_edges.at(i).v2.z == stl.stats.waterline_height && hull_edges.at(i).v2.x < min_x ){
            min_x = hull_edges.at(i).v2.x;
            initializeVertex(hull_edges.at(i).v2, aftmost);
        }
    }

    for( unsigned int i=0; i< domain_edges.size(); i++ ) {
        if( domain_edges.at(i).v1.x < min_x && domain_edges.at(i).v1.z == stl.stats.waterline_height ){
            initializeVertex(domain_edges.at(i).v1, abound);
        }
        if( domain_edges.at(i).v2.x < min_x && domain_edges.at(i).v2.z == stl.stats.waterline_height ){
            initializeVertex(domain_edges.at(i).v2, abound);
        }
        if( domain_edges.at(i).v1.x > max_x && domain_edges.at(i).v1.z == stl.stats.waterline_height ){
            initializeVertex(domain_edges.at(i).v1, fbound);
        }
        if( domain_edges.at(i).v2.x > max_x && domain_edges.at(i).v2.z == stl.stats.waterline_height ){
            initializeVertex(domain_edges.at(i).v2, fbound);
        }
    }

    stl_edge fedge, aedge;

    defineEdge(fedge,foremost,fbound);
    defineEdge(aedge,aftmost,abound);

    domain_edges.push_back(fedge);
    domain_edges.push_back(aedge);

    for( unsigned int i=0; i<hull_edges.size(); i++ ){
        domain_edges.push_back(hull_edges.at(i));
    }

    hull_edges.clear();
    vector<stl_edge> edges;
    vector<stl_vertex> points;
    FILE *f1;

    sortEdges(domain_edges,edges,variables); domain_edges.clear();
    EdgesToPoints(edges, points); edges.clear();

    f1 = fopen("./i.2", "w"); writePointsToFile1(f1, points); fclose(f1); points.clear();

    system("./tri <i.2> o.2");

    f1 = fopen("./o.2", "r");

    readFile2a(f1, waterline_facets,stl, variables);
    printf("--------> OK \n");
}

void connectCenterline(vector<stl_edge> &hull_edges, vector<stl_edge> &domain_edges, user_variables variables, vector<stl_facet> &centerline_facets, stl_file stl,
                        vector<stl_facet> &inlet, vector<stl_facet> &outflow, vector<stl_facet> &hull_facets, vector<stl_facet> &waterline_facets,
                            vector<stl_facet> bottom){

    printf("--------> Triangulating Centerline \n");domain_edges.clear();

    double max_x = -9999999;
    double max_zf = -9999999;
    stl_vertex foremost,aftmost, fore_domain, aft_domain;
    stl_edge e1,e2;
    vector<stl_edge> h1,h2;
    initializeVertex(hull_edges.front().v1, foremost);
    initializeVertex(hull_edges.front().v2, aftmost);
    initializeVertex(hull_edges.front().v1, fore_domain);
    initializeVertex(hull_edges.front().v1, aft_domain);

    int foremost_ID, aftmost_ID;

    double min_z = 999999;

    for( unsigned int i=0;i<hull_edges.size();i++ ){
            if( min_z > hull_edges.at(i).v1.z ){
                min_z = hull_edges.at(i).v1.z;
            }
            if( min_z > hull_edges.at(i).v2.z ){
                min_z = hull_edges.at(i).v2.z;
            }
    }

    for(unsigned int i=0;i<hull_edges.size();i++){
        if(hull_edges.at(i).v1.x > max_x && hull_edges.at(i).v1.y == variables.center && hull_edges.at(i).v1.z > max_zf ){
            max_x = hull_edges.at(i).v1.x;
            max_zf = hull_edges.at(i).v1.z;
            initializeVertex(hull_edges.at(i).v1, foremost);
            foremost_ID = i;
        }
        if(hull_edges.at(i).v2.x > max_x && hull_edges.at(i).v2.y == variables.center && hull_edges.at(i).v1.z > max_zf ){
            max_x = hull_edges.at(i).v2.x;
            max_zf = hull_edges.at(i).v2.z;
            initializeVertex(hull_edges.at(i).v2, foremost);
            foremost_ID = i;
        }
    }

    double min_x = 9999999;
    double max_za = -9999999;

    for(unsigned int i=0;i<hull_edges.size();i++){
        if(hull_edges.at(i).v1.x < min_x && hull_edges.at(i).v1.y == variables.center && hull_edges.at(i).v1.z > max_za ){
            min_x = hull_edges.at(i).v1.x;
            max_za = hull_edges.at(i).v1.z;
            initializeVertex(hull_edges.at(i).v1, aftmost);
            aftmost_ID = i;
        }
        if(hull_edges.at(i).v2.x < min_x && hull_edges.at(i).v2.y == variables.center && hull_edges.at(i).v2.z > max_za ){
            min_x = hull_edges.at(i).v2.x;
            max_za = hull_edges.at(i).v2.z;
            initializeVertex(hull_edges.at(i).v2, aftmost);
            aftmost_ID = i;
        }
    }

    min_x = 9999999;
    max_x = -9999999;

    h1.clear();
    findCenterlineEdges(variables,waterline_facets,h1);
    for(unsigned int i=0;i<h1.size();i++){
        if( h1.at(i).v1.x > max_x ){
            max_x = h1.at(i).v1.x;
            initializeVertex(h1.at(i).v1, fore_domain);
        }
        if( h1.at(i).v2.x > max_x ){
            max_x = h1.at(i).v2.x;
            initializeVertex(h1.at(i).v2, fore_domain);
        }
        if( h1.at(i).v1.x < min_x ){
            min_x = h1.at(i).v1.x;
            initializeVertex(h1.at(i).v1, aft_domain);
        }
        if( h1.at(i).v2.x < min_x ){
            min_x = h1.at(i).v2.x;
            initializeVertex(h1.at(i).v2, aft_domain);
        }
    }

    defineEdge(e1,foremost,fore_domain);
    defineEdge(e2,aft_domain,aftmost);

    h2.clear();
    h1.push_back(e1);h1.push_back(e2);
    h2.push_back(e1);h2.push_back(e2);

    vector<stl_edge> hull_1, hull_2;

    stl_vertex start, end;

    if( ifEqualVertex1(foremost, hull_edges.at(foremost_ID).v1, variables)){
        initializeVertex(hull_edges.at(foremost_ID).v1, start);
        initializeVertex(hull_edges.at(foremost_ID).v2, end);
    } else {
        initializeVertex(hull_edges.at(foremost_ID).v2, start);
        initializeVertex(hull_edges.at(foremost_ID).v1, end);
    }

    int ok = 0;

    hull_1.push_back(hull_edges.at(foremost_ID));
    hull_edges.erase(hull_edges.begin()+foremost_ID);

    do{
        for(unsigned int i=0;i<hull_edges.size();i++){
            if( ifEqualVertex1( hull_edges.at(i).v1, end, variables ) == 1){
                initializeVertex(hull_edges.at(i).v2, end);
                hull_1.push_back( hull_edges.at(i) );
                hull_edges.erase(hull_edges.begin()+i);i--;

                if( ifEqualVertex1(end, aftmost, variables) ){
                    ok = 1;
                    break;
                } else {
                    ok = 0;
                    continue;
                }

            }
            if( ifEqualVertex1( hull_edges.at(i).v2, end, variables ) == 1 ){
                initializeVertex(hull_edges.at(i).v1, end);
                hull_1.push_back( hull_edges.at(i) );
                hull_edges.erase(hull_edges.begin()+i);i--;

                if( ifEqualVertex1(end, aftmost, variables) ){
                    printVertex(end);
                    printVertex(aftmost);
                    ok = 1;
                    break;
                } else {
                    ok = 0;
                    continue;
                }
            }
        }
    }while(ok == 0);

    for(unsigned int i=0;i<hull_edges.size();i++){
        hull_2.push_back( hull_edges.at(i) );
    }
    hull_edges.clear();
    ok = 0;

    for( unsigned int i=0;i<hull_1.size();i++ ){
        if ( hull_1.at(i).v1.z == min_z || hull_1.at(i).v2.z == min_z ){
            ok = 1;
        }
    }


    findCenterlineEdges(variables,bottom,h2);
    findCenterlineEdges(variables,outflow,h2);
    findCenterlineEdges(variables,inlet,h2);

    if( ok == 1 ){
        copyVectorOfEdges(hull_1,h2);
        copyVectorOfEdges(hull_2,h1);
    } else {
        copyVectorOfEdges(hull_1,h1);
        copyVectorOfEdges(hull_2,h2);
    }

    FILE *f1, *f2;
    vector<stl_vertex> h1_points, h2_points;
    vector<stl_edge> h1_final, h2_final;

    sortEdges(h1, h1_final, variables); h1.clear();
    sortEdges(h2, h2_final, variables); h2.clear();


    EdgesToPoints(h1_final,h1_points); h1_final.clear();
    EdgesToPoints(h2_final,h2_points); h2_final.clear();

    reverseVector(h1_points);
    reverseVector(h2_points);

    f1 = fopen("./i.1", "w"); writePointsToFile1(f1, h1_points); fclose(f1); h1_points.clear();
    f2 = fopen("./i.2", "w"); writePointsToFile1(f2, h2_points); fclose(f2); h2_points.clear();

    system("./tri <i.1> o.1");
    system("./tri <i.2> o.2");

    f1 = fopen("./o.1", "r");
    f2 = fopen("./o.2", "r");

    readFile2a(f1, centerline_facets,stl, variables);
    readFile2a(f2, centerline_facets,stl, variables);
}

void monohull_1(stl_file &stl, user_variables &variables){
    printf("==== Starting procedure sequences for monohull  ==== \n \n");

    vector<stl_facet> facets;
    readFile(stl,facets,variables);

    calculateDomainDimensions(stl,variables);

    if( variables.waterline > stl.stats.size_z ){

        vector<stl_facet> center_facets;

        vector<stl_facet> waterline_facets;
        generateWaterline(stl,variables,waterline_facets);

        vector<stl_facet> bottom_facets;
        generateBottom(stl,variables,bottom_facets);

        vector<stl_facet> inlet_facets;
        generateInlet(stl,variables,inlet_facets);

        vector<stl_facet> port_facets;
        generatePort(stl,variables,port_facets);

        vector<stl_facet> outflow_facets;
        generateOutflow(stl,variables,outflow_facets);

        vector<stl_facet> hull_facets;
        vector<stl_edge> centerline_domain_edges, centerline_hull_edges;

        //deleting of waterline and centreline intersected facets
        deleteCenterFacets(stl,variables,facets,hull_facets); facets.clear();

        //finding edges lieing on waterline

        findCenterlineEdges(variables,hull_facets,centerline_hull_edges);
        findCenterlineEdges(variables,inlet_facets,centerline_domain_edges);
        findCenterlineEdges(variables,outflow_facets,centerline_domain_edges);
        findCenterlineEdges(variables,bottom_facets,centerline_domain_edges);
        findCenterlineEdges(variables,waterline_facets,centerline_domain_edges);

        connectCenterline(centerline_hull_edges,centerline_domain_edges,variables,center_facets,stl,inlet_facets,
                            outflow_facets, hull_facets,waterline_facets, bottom_facets);

        recalculateNormals(hull_facets);

        //writing new files
        writeFile(stl,variables , center_facets, bottom_facets, port_facets, inlet_facets, outflow_facets, waterline_facets, hull_facets );
        writePropFile(stl,variables , center_facets, bottom_facets, port_facets, inlet_facets, outflow_facets, waterline_facets, hull_facets);
    } else {

        vector<stl_facet> center_facets;
        vector<stl_facet> waterline_facets;

        vector<stl_facet> bottom_facets;
        generateBottom(stl,variables,bottom_facets);

        vector<stl_facet> inlet_facets;
        generateInlet(stl,variables,inlet_facets);

        vector<stl_facet> port_facets;
        generatePort(stl,variables,port_facets);

        vector<stl_facet> outflow_facets;
        generateOutflow(stl,variables,outflow_facets);

        vector<stl_facet> hull_facets, temp_facets;
        vector<stl_edge> waterline_domain_edges, waterline_hull_edges;
        vector<stl_edge> centerline_domain_edges, centerline_hull_edges;

        //deleting of waterline and centreline intersected facets
        deleteUpperFacets(stl,variables,facets,temp_facets); facets.clear();
        deleteCenterFacets(stl,variables,temp_facets,hull_facets); temp_facets.clear();

        //finding edges lieing on waterline
        findWaterlineEdges(stl,hull_facets,waterline_hull_edges, variables);
        findWaterlineEdges(stl,inlet_facets,waterline_domain_edges, variables);
        findWaterlineEdges(stl,outflow_facets,waterline_domain_edges, variables);
        findWaterlineEdges(stl,port_facets,waterline_domain_edges, variables);

        triangulateWaterline(waterline_domain_edges, waterline_hull_edges, waterline_facets, variables, stl, inlet_facets, outflow_facets, hull_facets, port_facets);

        findCenterlineEdges(variables,hull_facets,centerline_hull_edges);
        findCenterlineEdges(variables,inlet_facets,centerline_domain_edges);
        findCenterlineEdges(variables,outflow_facets,centerline_domain_edges);
        findCenterlineEdges(variables,bottom_facets,centerline_domain_edges);

        triangulateCenterline(centerline_domain_edges, centerline_hull_edges, center_facets, variables, stl);

        recalculateNormals(hull_facets);

        //writing new files
        writeFile(stl,variables , center_facets, bottom_facets, port_facets, inlet_facets, outflow_facets, waterline_facets, hull_facets );
        writePropFile(stl,variables , center_facets, bottom_facets, port_facets, inlet_facets, outflow_facets, waterline_facets, hull_facets);
    }
    system("./admesh -nfi --tolerance=0.000001 --iterations=5 model-v2.stl -a model-v3.stl");
    remove("o.2");
    remove("o.1");
    remove("i.1");
    remove("i.2");
}

void copyVectorOfEdges(vector<stl_edge> vec1, vector<stl_edge> &vec2 ){
    for(unsigned int i=0;i<vec1.size();i++){
        vec2.push_back(vec1.at(i));
    }
}

void copyVectorOfFacets(vector<stl_facet> vec1, vector<stl_facet> &vec2 ){
    for(unsigned int i=0;i<vec1.size();i++){
        vec2.push_back(vec1.at(i));
    }
}

void copyVectorOfVerticies(vector<stl_vertex> vec1, vector<stl_vertex> &vec2 ){
    for(unsigned int i=0;i<vec1.size();i++){
        vec2.push_back(vec1.at(i));
    }
}

void checkFacets(stl_file stl, vector<stl_facet> facets){
    for(unsigned int i=0;i<facets.size();i++){
        if(facets.at(i).vertex[0].z > stl.stats.waterline_height) { cout << "PROBLEM"; }
        if(facets.at(i).vertex[1].z > stl.stats.waterline_height) { cout << "PROBLEM"; }
        if(facets.at(i).vertex[2].z > stl.stats.waterline_height) { cout << "PROBLEM"; }
    }
}

void generateCenter(stl_file stl, user_variables variables, vector<stl_facet> &center_facets){
    stl_facet domain_facet;

    printf("--------> Generating Centerline plane ");

    if( variables.bodies == 1 ){
        for(unsigned int i=0;i<variables.N;i++){
            for(unsigned int k=variables.N;k>0;k--){
                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[2].y = variables.center;
                domain_facet.vertex[2].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[1].y = variables.center;
                domain_facet.vertex[1].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[0].y = variables.center;
                domain_facet.vertex[0].z = stl.stats.waterline_height - (k-1)*variables.dz;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                center_facets.push_back(domain_facet);

                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[2].y = variables.center;
                domain_facet.vertex[2].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[1].y = variables.center;
                domain_facet.vertex[1].z = stl.stats.waterline_height - (k-1)*variables.dz;
                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[0].y = variables.center;
                domain_facet.vertex[0].z = stl.stats.waterline_height - (k-1)*variables.dz;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                center_facets.push_back(domain_facet);
            }
        }
    } else if ( variables.bodies >= 2 ){
        for(unsigned int i=0;i<variables.N;i++){
            for(unsigned int k=variables.N;k>0;k--){
                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[2].y = stl.stats.min_y - variables.starboard;
                domain_facet.vertex[2].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[1].y = stl.stats.min_y - variables.starboard;
                domain_facet.vertex[1].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[0].y = stl.stats.min_y - variables.starboard;
                domain_facet.vertex[0].z = stl.stats.waterline_height - (k-1)*variables.dz;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                center_facets.push_back(domain_facet);

                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[2].y = stl.stats.min_y - variables.starboard;
                domain_facet.vertex[2].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[1].y = stl.stats.min_y - variables.starboard;
                domain_facet.vertex[1].z = stl.stats.waterline_height - (k-1)*variables.dz;
                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[0].y = stl.stats.min_y - variables.starboard;
                domain_facet.vertex[0].z = stl.stats.waterline_height - (k-1)*variables.dz;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                center_facets.push_back(domain_facet);
            }
        }
    } else {
        abort();
    }


	printf("--------> OK \n");
}

void generateWaterline(stl_file stl, user_variables variables, vector<stl_facet> &waterline_facets){
    stl_facet domain_facet;

    printf("--------> Generating Waterline plane ");

    if( variables.bodies == 1 ){
        for(unsigned int i=0;i<variables.N;i++){
            for(unsigned int j=variables.N;j>0;j--){
                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[2].y = variables.center + j*variables.dy;
                domain_facet.vertex[2].z = stl.stats.waterline_height;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[1].y = variables.center + j*variables.dy;
                domain_facet.vertex[1].z = stl.stats.waterline_height;
                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[0].y = variables.center + (j-1)*variables.dy;
                domain_facet.vertex[0].z = stl.stats.waterline_height;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                waterline_facets.push_back(domain_facet);

                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[2].y = variables.center + j*variables.dy;
                domain_facet.vertex[2].z = stl.stats.waterline_height;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[1].y = variables.center + (j-1)*variables.dy;
                domain_facet.vertex[1].z = stl.stats.waterline_height;
                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[0].y = variables.center + (j-1)*variables.dy;
                domain_facet.vertex[0].z = stl.stats.waterline_height;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                waterline_facets.push_back(domain_facet);
            }
        }
    } else if ( variables.bodies >= 2 ){
        for(unsigned int i=0;i<variables.N;i++){
            for(unsigned int j=variables.N;j>0;j--){
                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[2].y = stl.stats.min_y - variables.starboard + j*variables.dy;
                domain_facet.vertex[2].z = stl.stats.waterline_height;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[1].y = stl.stats.min_y - variables.starboard + j*variables.dy;
                domain_facet.vertex[1].z = stl.stats.waterline_height;
                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[0].y = stl.stats.min_y - variables.starboard + (j-1)*variables.dy;
                domain_facet.vertex[0].z = stl.stats.waterline_height;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                waterline_facets.push_back(domain_facet);

                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[2].y = stl.stats.min_y - variables.starboard + j*variables.dy;
                domain_facet.vertex[2].z = stl.stats.waterline_height;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[1].y = stl.stats.min_y - variables.starboard + (j-1)*variables.dy;
                domain_facet.vertex[1].z = stl.stats.waterline_height;
                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[0].y = stl.stats.min_y - variables.starboard + (j-1)*variables.dy;
                domain_facet.vertex[0].z = stl.stats.waterline_height;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                waterline_facets.push_back(domain_facet);
            }
        }
    } else {
        abort();
    }
	printf("--------> OK \n");
}

void generateBottom(stl_file stl, user_variables variables, vector<stl_facet> &bottom_facets){
    stl_facet domain_facet;

    printf("--------> Generating Bottom plane");

    if( variables.bodies == 1 ){
        for(unsigned int i=0;i<variables.N;i++){ //generation bottom
            for(unsigned int j=variables.N;j>0;j--){
                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[2].y = variables.center + j*variables.dy;
                domain_facet.vertex[2].z = stl.stats.bottom_height;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[1].y = variables.center + j*variables.dy;
                domain_facet.vertex[1].z = stl.stats.bottom_height;
                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[0].y = variables.center + (j-1)*variables.dy;
                domain_facet.vertex[0].z = stl.stats.bottom_height;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                bottom_facets.push_back(domain_facet);

                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[2].y = variables.center + j*variables.dy;
                domain_facet.vertex[2].z = stl.stats.bottom_height;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[1].y = variables.center + (j-1)*variables.dy;
                domain_facet.vertex[1].z = stl.stats.bottom_height;
                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[0].y = variables.center + (j-1)*variables.dy;
                domain_facet.vertex[0].z = stl.stats.bottom_height;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                bottom_facets.push_back(domain_facet);
            }
        }
    } else if ( variables.bodies >= 2 ){
        for(unsigned int i=0;i<variables.N;i++){ //generation bottom
            for(unsigned int j=variables.N;j>0;j--){
                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[2].y = stl.stats.min_y - variables.starboard + j*variables.dy;
                domain_facet.vertex[2].z = stl.stats.bottom_height;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[1].y = stl.stats.min_y - variables.starboard + j*variables.dy;
                domain_facet.vertex[1].z = stl.stats.bottom_height;
                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[0].y = stl.stats.min_y - variables.starboard + (j-1)*variables.dy;
                domain_facet.vertex[0].z = stl.stats.bottom_height;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                bottom_facets.push_back(domain_facet);

                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[2].y = stl.stats.min_y - variables.starboard + j*variables.dy;
                domain_facet.vertex[2].z = stl.stats.bottom_height;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[1].y = stl.stats.min_y - variables.starboard + (j-1)*variables.dy;
                domain_facet.vertex[1].z = stl.stats.bottom_height;
                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[0].y = stl.stats.min_y - variables.starboard + (j-1)*variables.dy;
                domain_facet.vertex[0].z = stl.stats.bottom_height;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                bottom_facets.push_back(domain_facet);
            }
        }
    } else {
        abort();
    }

	printf("--------> OK \n");
}

void generateInlet(stl_file stl, user_variables variables, vector<stl_facet> &inlet_facets){
    stl_facet domain_facet;

    printf("--------> Generating Inlet plane ");

    if( variables.bodies == 1 ){
        for(unsigned int j=0;j<variables.N;j++){
            for(unsigned int k=variables.N;k>0;k--){
                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft + variables.N*variables.dx;
                domain_facet.vertex[2].y = variables.center + j*variables.dy;
                domain_facet.vertex[2].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft + variables.N*variables.dx;
                domain_facet.vertex[1].y = variables.center + (j+1)*variables.dy;
                domain_facet.vertex[1].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft + variables.N*variables.dx;
                domain_facet.vertex[0].y = variables.center + j*variables.dy;
                domain_facet.vertex[0].z = stl.stats.waterline_height - (k-1)*variables.dz;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                inlet_facets.push_back(domain_facet);

                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft + variables.N*variables.dx;
                domain_facet.vertex[2].y = variables.center + (j+1)*variables.dy;
                domain_facet.vertex[2].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft + variables.N*variables.dx;
                domain_facet.vertex[1].y = variables.center + (j+1)*variables.dy;
                domain_facet.vertex[1].z = stl.stats.waterline_height - (k-1)*variables.dz;
                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft + variables.N*variables.dx;
                domain_facet.vertex[0].y = variables.center + j*variables.dy;
                domain_facet.vertex[0].z = stl.stats.waterline_height - (k-1)*variables.dz;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                inlet_facets.push_back(domain_facet);
            }
        }
    } else if( variables.bodies >= 2 ){
        for(unsigned int j=0;j<variables.N;j++){
            for(unsigned int k=variables.N;k>0;k--){
                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft + variables.N*variables.dx;
                domain_facet.vertex[2].y = stl.stats.min_y - variables.starboard + j*variables.dy;
                domain_facet.vertex[2].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft + variables.N*variables.dx;
                domain_facet.vertex[1].y = stl.stats.min_y - variables.starboard + (j+1)*variables.dy;
                domain_facet.vertex[1].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft + variables.N*variables.dx;
                domain_facet.vertex[0].y = stl.stats.min_y - variables.starboard + j*variables.dy;
                domain_facet.vertex[0].z = stl.stats.waterline_height - (k-1)*variables.dz;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                inlet_facets.push_back(domain_facet);

                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft + variables.N*variables.dx;
                domain_facet.vertex[2].y = stl.stats.min_y - variables.starboard + (j+1)*variables.dy;
                domain_facet.vertex[2].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft + variables.N*variables.dx;
                domain_facet.vertex[1].y = stl.stats.min_y - variables.starboard + (j+1)*variables.dy;
                domain_facet.vertex[1].z = stl.stats.waterline_height - (k-1)*variables.dz;
                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft + variables.N*variables.dx;
                domain_facet.vertex[0].y = stl.stats.min_y - variables.starboard + j*variables.dy;
                domain_facet.vertex[0].z = stl.stats.waterline_height - (k-1)*variables.dz;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                inlet_facets.push_back(domain_facet);
            }
        }
    } else {
        abort();
    }
	printf("--------> OK \n");
}

void generatePort(stl_file stl, user_variables variables, vector<stl_facet> &port_facets){
    stl_facet domain_facet;

    printf("--------> Generating Side plane ");

    if( variables.bodies == 1 ){
        for(unsigned int i=0;i<variables.N;i++){
            for(unsigned int k=variables.N;k>0;k--){
                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[0].y = variables.center + stl.stats.max_y + variables.port;
                domain_facet.vertex[0].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[1].y = variables.center + stl.stats.max_y + variables.port;
                domain_facet.vertex[1].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[2].y = variables.center + stl.stats.max_y + variables.port;
                domain_facet.vertex[2].z = stl.stats.waterline_height - (k-1)*variables.dz;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                port_facets.push_back(domain_facet);

                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[0].y = variables.center + stl.stats.max_y + variables.port;
                domain_facet.vertex[0].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[1].y = variables.center + stl.stats.max_y + variables.port;
                domain_facet.vertex[1].z = stl.stats.waterline_height - (k-1)*variables.dz;
                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[2].y = variables.center + stl.stats.max_y + variables.port;
                domain_facet.vertex[2].z = stl.stats.waterline_height - (k-1)*variables.dz;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                port_facets.push_back(domain_facet);
            }
        }
    } else if ( variables.bodies >= 2 ){
        for(unsigned int i=0;i<variables.N;i++){
            for(unsigned int k=variables.N;k>0;k--){
                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[0].y = stl.stats.min_y - variables.starboard + variables.N*variables.dy;
                domain_facet.vertex[0].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[1].y = stl.stats.min_y - variables.starboard + variables.N*variables.dy;
                domain_facet.vertex[1].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[2].y = stl.stats.min_y - variables.starboard + variables.N*variables.dy;
                domain_facet.vertex[2].z = stl.stats.waterline_height - (k-1)*variables.dz;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                port_facets.push_back(domain_facet);

                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[0].y = stl.stats.min_y - variables.starboard + variables.N*variables.dy;
                domain_facet.vertex[0].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft + (i+1)*variables.dx;
                domain_facet.vertex[1].y = stl.stats.min_y - variables.starboard + variables.N*variables.dy;
                domain_facet.vertex[1].z = stl.stats.waterline_height - (k-1)*variables.dz;
                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft + i*variables.dx;
                domain_facet.vertex[2].y = stl.stats.min_y - variables.starboard + variables.N*variables.dy;
                domain_facet.vertex[2].z = stl.stats.waterline_height - (k-1)*variables.dz;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                port_facets.push_back(domain_facet);
            }
        }
    } else {
        abort();
    }
	printf("--------> OK \n");
}

void generateOutflow(stl_file stl, user_variables variables, vector<stl_facet> &outflow_facets){
    stl_facet domain_facet;

    printf("--------> Generating Outflow plane ");

    if( variables.bodies == 1 ){
        for(unsigned int j=0;j<variables.N;j++){
            for(unsigned int k=variables.N;k>0;k--){
                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft;
                domain_facet.vertex[0].y = variables.center + j*variables.dy;
                domain_facet.vertex[0].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft;
                domain_facet.vertex[1].y = variables.center + (j+1)*variables.dy;
                domain_facet.vertex[1].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft;
                domain_facet.vertex[2].y = variables.center + j*variables.dy;
                domain_facet.vertex[2].z = stl.stats.waterline_height - (k-1)*variables.dz;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                outflow_facets.push_back(domain_facet);

                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft;
                domain_facet.vertex[0].y = variables.center + (j+1)*variables.dy;
                domain_facet.vertex[0].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft;
                domain_facet.vertex[1].y = variables.center + (j+1)*variables.dy;
                domain_facet.vertex[1].z = stl.stats.waterline_height - (k-1)*variables.dz;
                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft;
                domain_facet.vertex[2].y = variables.center + j*variables.dy;
                domain_facet.vertex[2].z = stl.stats.waterline_height - (k-1)*variables.dz;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                outflow_facets.push_back(domain_facet);
            }
        }
    } else if( variables.bodies >= 2 ){
        for(unsigned int j=0;j<variables.N;j++){
            for(unsigned int k=variables.N;k>0;k--){
                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft;
                domain_facet.vertex[0].y = stl.stats.min_y - variables.starboard + j*variables.dy;
                domain_facet.vertex[0].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft;
                domain_facet.vertex[1].y = stl.stats.min_y - variables.starboard + (j+1)*variables.dy;
                domain_facet.vertex[1].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft;
                domain_facet.vertex[2].y = stl.stats.min_y - variables.starboard + j*variables.dy;
                domain_facet.vertex[2].z = stl.stats.waterline_height - (k-1)*variables.dz;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                outflow_facets.push_back(domain_facet);

                domain_facet.vertex[0].x = stl.stats.min_x - variables.aft;
                domain_facet.vertex[0].y = stl.stats.min_y - variables.starboard + (j+1)*variables.dy;
                domain_facet.vertex[0].z = stl.stats.waterline_height - k*variables.dz;
                domain_facet.vertex[1].x = stl.stats.min_x - variables.aft;
                domain_facet.vertex[1].y = stl.stats.min_y - variables.starboard + (j+1)*variables.dy;
                domain_facet.vertex[1].z = stl.stats.waterline_height - (k-1)*variables.dz;
                domain_facet.vertex[2].x = stl.stats.min_x - variables.aft;
                domain_facet.vertex[2].y = stl.stats.min_y - variables.starboard + j*variables.dy;
                domain_facet.vertex[2].z = stl.stats.waterline_height - (k-1)*variables.dz;
                calculateNormal(&domain_facet);
                initializeFacet(stl.stats.waterline_height,&domain_facet,variables.factor);
                outflow_facets.push_back(domain_facet);
            }
        }
    } else {
        abort();
    }
	printf("--------> OK \n");
}

void writeVectorToFile(FILE *fpp, vector<stl_facet> vector){
    if(!vector.empty()){
        for(unsigned long i = 0; i < vector.size(); i++){
              fprintf(fpp, "  facet normal %.8lf %.8lf %.8lf \n", vector[i].normal.x, vector[i].normal.y, vector[i].normal.z);
              fprintf(fpp, "    outer loop\n");
              fprintf(fpp, "      vertex %.8lf %.8lf %.8lf \n", vector[i].vertex[0].x, vector[i].vertex[0].y, vector[i].vertex[0].z);
              fprintf(fpp, "      vertex %.8lf %.8lf %.8lf \n", vector[i].vertex[1].x, vector[i].vertex[1].y, vector[i].vertex[1].z);
              fprintf(fpp, "      vertex %.8lf %.8lf %.8lf \n", vector[i].vertex[2].x, vector[i].vertex[2].y, vector[i].vertex[2].z);
              fprintf(fpp, "    endloop\n");
              fprintf(fpp, "  endfacet\n");
        }
    } else {
        cout << "You were trying to write an empty array!" << endl;
    }
}

void writeFile(stl_file stl, user_variables variables, vector<stl_facet> v1, vector<stl_facet> v2, vector<stl_facet> v3, vector<stl_facet> v4, vector<stl_facet> v5, vector<stl_facet> v6, vector<stl_facet> v7 ){
    vector<stl_facet> fs;
    vector<stl_facet> fb;
    vector<stl_facet> tr;
    unsigned int num = topologyDetection(v7,fs,fb,tr, variables);

    stl.out_file = fopen("./model-v2.stl", "w");

    fprintf(stl.out_file, "solid one \n");
    writeVectorToFile(stl.out_file,v1); //center
    writeVectorToFile(stl.out_file,v2); //bottom
    writeVectorToFile(stl.out_file,v3); //port
    writeVectorToFile(stl.out_file,v4); //inlet
    writeVectorToFile(stl.out_file,v5); //outflow
    writeVectorToFile(stl.out_file,v6); //waterline
    writeVectorToFile(stl.out_file,v7); //curved hull
    switch(num)
    {
        case 1000: writeVectorToFile(stl.out_file,fs);  break;
        case 100:  writeVectorToFile(stl.out_file,fb);  break;
        case 10:   writeVectorToFile(stl.out_file,tr);  break;
        case 1110: writeVectorToFile(stl.out_file,tr);
                   writeVectorToFile(stl.out_file,fb);
                   writeVectorToFile(stl.out_file,fs);  break;
        case 1100: writeVectorToFile(stl.out_file,fb);
                   writeVectorToFile(stl.out_file,fs);  break;
        case 110:  writeVectorToFile(stl.out_file,tr);
                   writeVectorToFile(stl.out_file,fb);  break;
        case 1010: writeVectorToFile(stl.out_file,tr);
                   writeVectorToFile(stl.out_file,fs);  break;
        default  : break;
    }
    fprintf(stl.out_file, "endsolid one \n");
	fclose(stl.out_file);

	cout << "New STL file has been generated successfully! \n";
}

void writeVectorDomain(FILE *fpp, vector<stl_facet> vector, int domain){
    if(!vector.empty()){
        for(unsigned long i = 0; i < vector.size(); i++){
            fprintf(fpp, "%u\n", domain);
        }
    } else {
        cout << "You are trying to write an empty domain!" << endl;
    }
}

int topologyDetection(vector<stl_facet> &hull_facets, vector<stl_facet> &fs, vector<stl_facet> &fb, vector<stl_facet> &tr, user_variables variables){
    unsigned int num =0;
    for(unsigned int i=0;i<hull_facets.size();i++){
        if( variables.transom == 1 ){
            if( (fabs(hull_facets.at(i).normal.x - 0) > 1e-20 && fabs(hull_facets.at(i).normal.y - 0) < 1e-5 && fabs(hull_facets.at(i).normal.z - 0) < 1e-5)
              || fabs(hull_facets.at(i).normal.x - 1) < 1e-12 ){
                tr.push_back(hull_facets.at(i));
                hull_facets.erase(hull_facets.begin()+i);
                --i; continue;
            }
        }
        if( variables.sides == 1 ){
            if( (fabs(hull_facets.at(i).normal.x - 0) < 1e-5 && fabs(hull_facets.at(i).normal.y - 0) > 1e-20 && fabs(hull_facets.at(i).normal.z - 0) < 1e-5)
            || fabs(hull_facets.at(i).normal.y - 1) < 1e-12 ){
                fs.push_back(hull_facets.at(i));
                hull_facets.erase(hull_facets.begin()+i);
                --i; continue;
            }
        }
        if( variables.bottom ==1 ){
            if( (fabs(hull_facets.at(i).normal.x - 0) < 1e-5 && fabs(hull_facets.at(i).normal.y - 0) < 1e-5 && fabs(hull_facets.at(i).normal.z - 0) > 1e-20)
            || fabs(hull_facets.at(i).normal.z - 1) < 1e-12 ){
                fb.push_back(hull_facets.at(i));
                hull_facets.erase(hull_facets.begin()+i);
                --i; continue;
            }
        }
    }
    if(!fs.empty()){ num += 1000; }
    if(!fb.empty()){ num += 100; }
    if(!tr.empty()){ num += 10; }

    return num;
}

void writePropFile(stl_file stl, user_variables variables, vector<stl_facet> v1, vector<stl_facet> v2, vector<stl_facet> v3, vector<stl_facet> v4, vector<stl_facet> v5, vector<stl_facet> v6, vector<stl_facet> v7 ){
    vector<stl_facet> fs;
    vector<stl_facet> fb;
    vector<stl_facet> tr;
    unsigned int num = topologyDetection(v7,fs,fb,tr, variables);
    unsigned int domains = calculateDomains(num);

    stl.prop_file = fopen("./model-v3.stl.prop", "w");
    fprintf(stl.prop_file, "%u\n", domains);
    writeVectorDomain(stl.prop_file,v1,0);
    writeVectorDomain(stl.prop_file,v2,1);
    writeVectorDomain(stl.prop_file,v3,2);
    writeVectorDomain(stl.prop_file,v4,3);
    writeVectorDomain(stl.prop_file,v5,4);
    writeVectorDomain(stl.prop_file,v6,5);
    writeVectorDomain(stl.prop_file,v7,6);
    switch(num)
    {
        case 1000: writeVectorDomain(stl.prop_file,fs,7);  break;
        case 100:  writeVectorDomain(stl.prop_file,fb,7);  break;
        case 10:   writeVectorDomain(stl.prop_file,tr,7);  break;
        case 1110: writeVectorDomain(stl.prop_file,tr,7);
                   writeVectorDomain(stl.prop_file,fb,8);
                   writeVectorDomain(stl.prop_file,fs,9);  break;
        case 1100: writeVectorDomain(stl.prop_file,fb,7);
                   writeVectorDomain(stl.prop_file,fs,8);  break;
        case 110:  writeVectorDomain(stl.prop_file,tr,7);
                   writeVectorDomain(stl.prop_file,fb,8);  break;
        case 1010: writeVectorDomain(stl.prop_file,tr,7);
                   writeVectorDomain(stl.prop_file,fs,8);  break;
        default:   break;
    }
    fclose(stl.prop_file);

    cout << "New stl.prop file has been generated successfully! \n";
}

void deleteCenterFacets(stl_file stl, user_variables variables, vector<stl_facet> entry, vector<stl_facet> &result){
    stl_vertex vertex1,vertex2;
    stl_facet  facet1,facet2;

    printf("--------> Cutting Ship hull at centreline");

	for(unsigned long i=0;i<entry.size();i++){   //this is working
	    if(entry.at(i).vertex[0].y == variables.center &&
			entry.at(i).vertex[1].y == variables.center &&
			 entry.at(i).vertex[2].y > variables.center ){
			result.push_back(entry.at(i));
			continue;
		}
		if(entry.at(i).vertex[0].y > variables.center &&
			entry.at(i).vertex[1].y == variables.center &&
			 entry.at(i).vertex[2].y == variables.center ){
			result.push_back(entry.at(i));
			continue;
		}
		if(entry.at(i).vertex[0].y == variables.center &&
			entry.at(i).vertex[1].y > variables.center &&
			 entry.at(i).vertex[2].y == variables.center ){
			result.push_back(entry.at(i));
			continue;
		}
		if(entry.at(i).vertex[0].y > variables.center &&
			entry.at(i).vertex[1].y > variables.center &&
			 entry.at(i).vertex[2].y > variables.center ){
			result.push_back(entry.at(i));
			continue;
		}
		if(entry.at(i).vertex[0].y < variables.center &&
			entry.at(i).vertex[1].y > variables.center &&
			 entry.at(i).vertex[2].y > variables.center ){  //this is fixed AGAIN
				linearInterpolation3D_y(&vertex1,&entry.at(i).vertex[0],&entry.at(i).vertex[1],variables.center);
				linearInterpolation3D_y(&vertex2,&entry.at(i).vertex[0],&entry.at(i).vertex[2],variables.center);
				facet1.vertex[0] = vertex1;
				facet1.vertex[1] = entry.at(i).vertex[1];
				facet1.vertex[2] = entry.at(i).vertex[2];
				facet2.vertex[0] = facet1.vertex[2];
				facet2.vertex[1] = vertex2;
				facet2.vertex[2] = facet1.vertex[0];
				calculateNormal(&facet1);
				calculateNormal(&facet2);
                initializeFacet(stl.stats.waterline_height,&facet1,variables.factor);
                initializeFacet(stl.stats.waterline_height,&facet2,variables.factor);
				result.push_back(facet1);
				result.push_back(facet2);
				continue;
		}
		if(entry.at(i).vertex[0].y > variables.center &&
			entry.at(i).vertex[1].y < variables.center &&
			 entry.at(i).vertex[2].y > variables.center ){ //this is fixed AGAIN
				linearInterpolation3D_y(&vertex1,&entry.at(i).vertex[1],&entry.at(i).vertex[0],variables.center);
				linearInterpolation3D_y(&vertex2,&entry.at(i).vertex[1],&entry.at(i).vertex[2],variables.center);
				facet1.vertex[0] = entry.at(i).vertex[0];
				facet1.vertex[1] = vertex1;
				facet1.vertex[2] = entry.at(i).vertex[2];
				facet2.vertex[0] = facet1.vertex[2];
				facet2.vertex[1] = facet1.vertex[1];
				facet2.vertex[2] = vertex2;
				calculateNormal(&facet1);
				calculateNormal(&facet2);
				initializeFacet(stl.stats.waterline_height,&facet1,variables.factor);
				initializeFacet(stl.stats.waterline_height,&facet2,variables.factor);
				result.push_back(facet1);
				result.push_back(facet2);
				continue;
		}
		if(entry.at(i).vertex[0].y > variables.center &&
			entry.at(i).vertex[1].y > variables.center &&
			 entry.at(i).vertex[2].y < variables.center ){ //this is fixed AGAIN
			linearInterpolation3D_y(&vertex1,&entry.at(i).vertex[2],&entry.at(i).vertex[0],variables.center);
			linearInterpolation3D_y(&vertex2,&entry.at(i).vertex[2],&entry.at(i).vertex[1],variables.center);
			facet1.vertex[0] = entry.at(i).vertex[0];
			facet1.vertex[1] = entry.at(i).vertex[1];
			facet1.vertex[2] = vertex1;
			facet2.vertex[0] = facet1.vertex[2];
			facet2.vertex[1] = facet1.vertex[1];
			facet2.vertex[2] = vertex2;
			calculateNormal(&facet1);
			calculateNormal(&facet2);
			initializeFacet(stl.stats.waterline_height,&facet1,variables.factor);
			initializeFacet(stl.stats.waterline_height,&facet2,variables.factor);
			result.push_back(facet1);
			result.push_back(facet2);
			continue;
		}
		if(entry.at(i).vertex[0].y > variables.center &&
			entry.at(i).vertex[1].y < variables.center &&
			 entry.at(i).vertex[2].y < variables.center ){ //this is fixed AGAIN
			linearInterpolation3D_y(&vertex1,&entry.at(i).vertex[0],&entry.at(i).vertex[1],variables.center);
			linearInterpolation3D_y(&vertex2,&entry.at(i).vertex[0],&entry.at(i).vertex[2],variables.center);
			facet1.vertex[0] = entry.at(i).vertex[0];
			facet1.vertex[1] = vertex1;
			facet1.vertex[2] = vertex2;
			calculateNormal(&facet1);
			initializeFacet(stl.stats.waterline_height,&facet1,variables.factor);
			result.push_back(facet1);
			continue;
		}
		if(entry.at(i).vertex[0].y < variables.center &&
			entry.at(i).vertex[1].y < variables.center &&
			 entry.at(i).vertex[2].y > variables.center ){ //this is fixed AGAIN
			linearInterpolation3D_y(&vertex1,&entry.at(i).vertex[2],&entry.at(i).vertex[1],variables.center);
			linearInterpolation3D_y(&vertex2,&entry.at(i).vertex[2],&entry.at(i).vertex[0],variables.center);
			facet1.vertex[0] = vertex2;
			facet1.vertex[1] = vertex1;
			facet1.vertex[2] = entry.at(i).vertex[2];
			calculateNormal(&facet1);
			initializeFacet(stl.stats.waterline_height,&facet1,variables.factor);
			result.push_back(facet1);
			continue;
		}
		if(entry.at(i).vertex[0].y < variables.center &&
			entry.at(i).vertex[1].y > variables.center &&
			 entry.at(i).vertex[2].y < variables.center ){ //this is fixed
			linearInterpolation3D_y(&vertex1,&entry.at(i).vertex[1],&entry.at(i).vertex[2],variables.center);
			linearInterpolation3D_y(&vertex2,&entry.at(i).vertex[1],&entry.at(i).vertex[0],variables.center);
			facet1.vertex[0] = vertex2;
			facet1.vertex[1] = entry.at(i).vertex[1];
			facet1.vertex[2] = vertex1;
			calculateNormal(&facet1);
			initializeFacet(stl.stats.waterline_height,&facet1,variables.factor);
			result.push_back(facet1);
			continue;
		}
	}
	entry.clear();
	printf("--------> OK \n");
}

void deleteUpperFacets(stl_file stl, user_variables variables, vector<stl_facet> entry, vector<stl_facet> &result){
    stl_vertex vertex1,vertex2;
    stl_facet  facet1,facet2;

    printf("--------> Cutting Ship hull at waterline ");

	for(unsigned long i=0;i<entry.size();i++){   //this is working
	    if(entry.at(i).vertex[0].z == stl.stats.waterline_height &&
			entry.at(i).vertex[1].z == stl.stats.waterline_height &&
			 entry.at(i).vertex[2].z < stl.stats.waterline_height){
            result.push_back(entry.at(i));
            continue;
		}
		if(entry.at(i).vertex[0].z < stl.stats.waterline_height &&
			entry.at(i).vertex[1].z == stl.stats.waterline_height &&
			 entry.at(i).vertex[2].z == stl.stats.waterline_height){
            result.push_back(entry.at(i));
            continue;
		}
		if(entry.at(i).vertex[0].z == stl.stats.waterline_height &&
			entry.at(i).vertex[1].z < stl.stats.waterline_height &&
			 entry.at(i).vertex[2].z == stl.stats.waterline_height){
            result.push_back(entry.at(i));
            continue;
		}
		if(entry.at(i).vertex[0].z < stl.stats.waterline_height &&
			entry.at(i).vertex[1].z < stl.stats.waterline_height &&
			 entry.at(i).vertex[2].z < stl.stats.waterline_height){
			result.push_back(entry.at(i));
			continue;
		}
		if(entry.at(i).vertex[0].z > stl.stats.waterline_height &&
			entry.at(i).vertex[1].z < stl.stats.waterline_height &&
			 entry.at(i).vertex[2].z < stl.stats.waterline_height){  //this is fixed AGAIN
				linearInterpolation3D_z(&vertex1,&entry.at(i).vertex[0],&entry.at(i).vertex[1],stl.stats.waterline_height);
				linearInterpolation3D_z(&vertex2,&entry.at(i).vertex[0],&entry.at(i).vertex[2],stl.stats.waterline_height);
				facet1.vertex[0] = vertex1;
				facet1.vertex[1] = entry.at(i).vertex[1];
				facet1.vertex[2] = entry.at(i).vertex[2];
				facet2.vertex[0] = facet1.vertex[2];
				facet2.vertex[1] = vertex2;
				facet2.vertex[2] = facet1.vertex[0];
				calculateNormal(&facet1);
				calculateNormal(&facet2);
                initializeFacet(stl.stats.waterline_height,&facet1,variables.factor);
                initializeFacet(stl.stats.waterline_height,&facet2,variables.factor);
				result.push_back(facet1);
				result.push_back(facet2);
				continue;
		}
		if(entry.at(i).vertex[0].z < stl.stats.waterline_height &&
			entry.at(i).vertex[1].z > stl.stats.waterline_height &&
			 entry.at(i).vertex[2].z < stl.stats.waterline_height){ //this is fixed AGAIN
				linearInterpolation3D_z(&vertex1,&entry.at(i).vertex[1],&entry.at(i).vertex[0],stl.stats.waterline_height);
				linearInterpolation3D_z(&vertex2,&entry.at(i).vertex[1],&entry.at(i).vertex[2],stl.stats.waterline_height);
				facet1.vertex[0] = entry.at(i).vertex[0];
				facet1.vertex[1] = vertex1;
				facet1.vertex[2] = entry.at(i).vertex[2];
				facet2.vertex[0] = facet1.vertex[2];
				facet2.vertex[1] = facet1.vertex[1];
				facet2.vertex[2] = vertex2;
				calculateNormal(&facet1);
				calculateNormal(&facet2);
				initializeFacet(stl.stats.waterline_height,&facet1,variables.factor);
				initializeFacet(stl.stats.waterline_height,&facet2,variables.factor);
				result.push_back(facet1);
				result.push_back(facet2);
				continue;
		}
		if(entry.at(i).vertex[0].z < stl.stats.waterline_height &&
			entry.at(i).vertex[1].z < stl.stats.waterline_height &&
			 entry.at(i).vertex[2].z > stl.stats.waterline_height){ //this is fixed AGAIN
			linearInterpolation3D_z(&vertex1,&entry.at(i).vertex[2],&entry.at(i).vertex[0],stl.stats.waterline_height);
			linearInterpolation3D_z(&vertex2,&entry.at(i).vertex[2],&entry.at(i).vertex[1],stl.stats.waterline_height);
			facet1.vertex[0] = entry.at(i).vertex[0];
			facet1.vertex[1] = entry.at(i).vertex[1];
			facet1.vertex[2] = vertex1;
			facet2.vertex[0] = facet1.vertex[2];
			facet2.vertex[1] = facet1.vertex[1];
			facet2.vertex[2] = vertex2;
			calculateNormal(&facet1);
			calculateNormal(&facet2);
			initializeFacet(stl.stats.waterline_height,&facet1,variables.factor);
			initializeFacet(stl.stats.waterline_height,&facet2,variables.factor);
			result.push_back(facet1);
			result.push_back(facet2);
			continue;
		}
		if(entry.at(i).vertex[0].z < stl.stats.waterline_height &&
			entry.at(i).vertex[1].z > stl.stats.waterline_height &&
			 entry.at(i).vertex[2].z > stl.stats.waterline_height){ //this is fixed AGAIN
			linearInterpolation3D_z(&vertex1,&entry.at(i).vertex[0],&entry.at(i).vertex[1],stl.stats.waterline_height);
			linearInterpolation3D_z(&vertex2,&entry.at(i).vertex[0],&entry.at(i).vertex[2],stl.stats.waterline_height);
			facet1.vertex[0] = entry.at(i).vertex[0];
			facet1.vertex[1] = vertex1;
			facet1.vertex[2] = vertex2;
			calculateNormal(&facet1);
			initializeFacet(stl.stats.waterline_height,&facet1,variables.factor);
			result.push_back(facet1);
			continue;
		}
		if(entry.at(i).vertex[0].z > stl.stats.waterline_height &&
			entry.at(i).vertex[1].z > stl.stats.waterline_height &&
			 entry.at(i).vertex[2].z < stl.stats.waterline_height){ //this is fixed AGAIN
			linearInterpolation3D_z(&vertex1,&entry.at(i).vertex[2],&entry.at(i).vertex[1],stl.stats.waterline_height);
			linearInterpolation3D_z(&vertex2,&entry.at(i).vertex[2],&entry.at(i).vertex[0],stl.stats.waterline_height);
			facet1.vertex[0] = vertex2;
			facet1.vertex[1] = vertex1;
			facet1.vertex[2] = entry.at(i).vertex[2];
			calculateNormal(&facet1);
			initializeFacet(stl.stats.waterline_height,&facet1,variables.factor);
			result.push_back(facet1);
			continue;
		}
		if(entry.at(i).vertex[0].z > stl.stats.waterline_height &&
			entry.at(i).vertex[1].z < stl.stats.waterline_height &&
			 entry.at(i).vertex[2].z > stl.stats.waterline_height){ //this is fixed
			linearInterpolation3D_z(&vertex1,&entry.at(i).vertex[1],&entry.at(i).vertex[2],stl.stats.waterline_height);
			linearInterpolation3D_z(&vertex2,&entry.at(i).vertex[1],&entry.at(i).vertex[0],stl.stats.waterline_height);
			facet1.vertex[0] = vertex2;
			facet1.vertex[1] = entry.at(i).vertex[1];
			facet1.vertex[2] = vertex1;
			calculateNormal(&facet1);
			initializeFacet(stl.stats.waterline_height,&facet1,variables.factor);
			result.push_back(facet1);
			continue;
		}
	}
	entry.clear();
	printf("--------> OK \n");
}

void cleanFile(){

	vector<string> lines;
	string sline;
	string str1 = "solid";
	string str2 = "endsolid";
	size_t f1,f2;
	ifstream myfile("model.stl");
	while(!myfile.eof()){
		getline(myfile,sline);
		f1 = sline.find(str1);
		f2 = sline.find(str2);
		if(f1!=string::npos || f2!=string::npos){
			continue;
		} else {
			lines.push_back(sline);
		}
	}
	myfile.close();
	remove("model.stl");

	ofstream myfilew("model.stl");
	myfilew << "solid one\n";
	for(unsigned long k = 0; k<lines.size();k++){
		myfilew << lines[k];
	}
	myfilew << "endsolid one\n";
	myfilew.close();
}

void readFile(stl_file &stl, vector<stl_facet> &facets, user_variables variables){
    stl_facet facet;

  //  cleanFile();

	stl.in_file = fopen("model.stl","r");
	if(stl.in_file != NULL){
		printf("--------> File has been opened successfully! \n");
		fseek(stl.in_file, 0, SEEK_END);
	stl.fileSize = ftell(stl.in_file);

	// Get number of facets
	rewind(stl.in_file);
    unsigned long j = 0;
    unsigned long num_lines = 0;
    for(unsigned long i = 0; i < stl.fileSize ; i++)
	{
	  j++;
	  if(getc(stl.in_file) == '\n') {
	      if(j > 4){
	    	  num_lines++;
	      }
	      j = 0;
	    }
	}
    stl.stats.number_lines  = num_lines;
    stl.stats.number_facets = (stl.stats.number_lines-2)/ASCII_LINES_PER_FACET;

    stl.stats.max_x = -999999;
    stl.stats.max_y = -999999;
    stl.stats.max_z = -999999;
    stl.stats.min_x = 999999;
    stl.stats.min_y = 999999;
    stl.stats.min_z = 999999;

    // Get file contents
    rewind(stl.in_file);
    while(getc(stl.in_file) != '\n');
    for(unsigned long i=0;i<=stl.stats.number_facets;i++){
    	fscanf(stl.in_file, "%*s %*s %lf %lf %lf \n", &facet.normal.x, &facet.normal.y, &facet.normal.z);
    	fscanf(stl.in_file, "%*s %*s");
    	fscanf(stl.in_file, "%*s %lf %lf %lf \n", &facet.vertex[0].x, &facet.vertex[0].y,  &facet.vertex[0].z);
    	fscanf(stl.in_file, "%*s %lf %lf %lf \n", &facet.vertex[1].x, &facet.vertex[1].y,  &facet.vertex[1].z);
    	fscanf(stl.in_file, "%*s %lf %lf %lf \n", &facet.vertex[2].x, &facet.vertex[2].y,  &facet.vertex[2].z);
    	fscanf(stl.in_file, "%*s");
    	fscanf(stl.in_file, "%*s");

    	initializeFacet(stl.stats.waterline_height, &facet, variables.factor);
    	facets.push_back(facet);

    	stl.stats.max_x = STL_MAX(stl.stats.max_x, facet.vertex[0].x);
    	stl.stats.min_x = STL_MIN(stl.stats.min_x, facet.vertex[0].x);
    	stl.stats.max_y = STL_MAX(stl.stats.max_y, facet.vertex[0].y);
    	stl.stats.min_y = STL_MIN(stl.stats.min_y, facet.vertex[0].y);
    	stl.stats.max_z = STL_MAX(stl.stats.max_z, facet.vertex[0].z);
    	stl.stats.min_z = STL_MIN(stl.stats.min_z, facet.vertex[0].z);

    	stl.stats.max_x = STL_MAX(stl.stats.max_x, facet.vertex[1].x);
    	stl.stats.min_x = STL_MIN(stl.stats.min_x, facet.vertex[1].x);
    	stl.stats.max_y = STL_MAX(stl.stats.max_y, facet.vertex[1].y);
    	stl.stats.min_y = STL_MIN(stl.stats.min_y, facet.vertex[1].y);
    	stl.stats.max_z = STL_MAX(stl.stats.max_z, facet.vertex[1].z);
    	stl.stats.min_z = STL_MIN(stl.stats.min_z, facet.vertex[1].z);

    	stl.stats.max_x = STL_MAX(stl.stats.max_x, facet.vertex[2].x);
    	stl.stats.min_x = STL_MIN(stl.stats.min_x, facet.vertex[2].x);
    	stl.stats.max_y = STL_MAX(stl.stats.max_y, facet.vertex[2].y);
    	stl.stats.min_y = STL_MIN(stl.stats.min_y, facet.vertex[2].y);
    	stl.stats.max_z = STL_MAX(stl.stats.max_z, facet.vertex[2].z);
    	stl.stats.min_z = STL_MIN(stl.stats.min_z, facet.vertex[2].z);
    }
    printf("--------> STL file read successfully \n \n");
    fclose(stl.in_file);
	} else {
		printf("--------> Error in opening the file \n");
		abort();
	}
}

void findCenterlineEdges(user_variables variables, vector<stl_facet> hull_facets, vector<stl_edge> &edges){

    for(unsigned int i=0;i<hull_facets.size();i++){
        if( (hull_facets.at(i).edge[0].v1.y == variables.center ) && (hull_facets.at(i).edge[0].v2.y == variables.center ) ){
            edges.push_back(hull_facets.at(i).edge[0]);
        }
        if( (hull_facets.at(i).edge[1].v1.y == variables.center ) && (hull_facets.at(i).edge[1].v2.y == variables.center ) ){
            edges.push_back(hull_facets.at(i).edge[1]);
        }
        if( (hull_facets.at(i).edge[2].v1.y == variables.center ) && (hull_facets.at(i).edge[2].v2.y == variables.center ) ){
            edges.push_back(hull_facets.at(i).edge[2]);
        }
    }
    if( edges.empty() ){
        abort();
    }
}

void findWaterlineEdges(stl_file stl, vector<stl_facet> hull_facets, vector<stl_edge> &edges, user_variables variables){

    for(unsigned int i=0;i<hull_facets.size();i++){
        if(  hull_facets.at(i).edge[0].v1.z == stl.stats.waterline_height && hull_facets.at(i).edge[0].v2.z == stl.stats.waterline_height ){
            edges.push_back(hull_facets.at(i).edge[0]);
        }
        if(  hull_facets.at(i).edge[1].v1.z == stl.stats.waterline_height && hull_facets.at(i).edge[1].v2.z == stl.stats.waterline_height ){
            edges.push_back(hull_facets.at(i).edge[1]);
        }
        if(  hull_facets.at(i).edge[2].v1.z == stl.stats.waterline_height && hull_facets.at(i).edge[2].v2.z == stl.stats.waterline_height ){
            edges.push_back(hull_facets.at(i).edge[2]);
        }
    }
    if( edges.empty() ){
        abort();
    }
}

stl_edge swapEdge(stl_edge edge){
    stl_vertex t1,t2;
    stl_edge e1;

    initializeVertex(edge.v1,t1);
    initializeVertex(edge.v2,t2);

    defineEdge(e1,t2,t1);

    return e1;
}

void sortEdges(vector<stl_edge> start, vector<stl_edge> &final, user_variables variables){
    stl_vertex begin,end;
    stl_edge temp;

    final.push_back(start.front());
    start.erase(start.begin());

    initializeVertex(final.front().v2,end);
    initializeVertex(final.front().v1,begin);

    while(!start.empty()){
        for(unsigned int i=0;i<start.size();i++){
            if( ifEqualVertex1(end, start.at(i).v1,variables) ){
                final.push_back(start.at(i));
                initializeVertex(final.back().v2,end);
                start.erase(start.begin()+i);i--;
                continue;
            }
            if( ifEqualVertex1(end, start.at(i).v2,variables) ){
                temp = swapEdge(start.at(i));
                final.push_back(temp);
                initializeVertex(final.back().v2,end);
                start.erase(start.begin()+i);i--;
                continue;
            }
            if( ifEqualVertex1(begin, start.at(i).v1,variables) ){
                temp = swapEdge(start.at(i));
                final.insert(final.begin(), temp);
                initializeVertex(final.front().v1,begin);
                start.erase(start.begin()+i);i--;
                continue;
            }
            if( ifEqualVertex1(begin, start.at(i).v2,variables)  ){
                final.insert(final.begin(), start.at(i));
                initializeVertex(final.front().v1,begin);
                start.erase(start.begin()+i);i--;
                continue;
            }
        }
    }
}


void recalculateNormals( vector<stl_facet> &facets ){
    for(unsigned int i=0;i<facets.size();i++){
        calculateNormal(&facets.at(i));
    }
}
