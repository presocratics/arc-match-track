// =====================================================================================
//
//       Filename:  main.cpp
//
//    Description:  Create new file to read in image and gyro data.
//
//        Version:  1.0
//        Created:  07/03/2013 02:11:35 PM
//       Revision:  none
//       Compiler:  gcc
//
//         Author:  Martin Miller (), miller7@illinois.edu
//   Organization:  Aerospace Robotics and Control Lab
//
// =====================================================================================

#include <iostream>
#include <istream>
#include <string>
#include <vector>
#include <sstream>
#include <cstdlib>
#include <fstream>
#define spc " "
using namespace std;
typedef struct {
    string frame_name;
    float x, y, z;
} Unit;

typedef struct {
    long gtime;
    float x, y, z;
} Gyro;

typedef struct {
    long ftime;
    string frame;
} Frame;

int main()
{
    vector<Unit> U;
    vector<Gyro> G;
    vector<Frame> F;

    string    gfname = "gyro2";                 // input  file name 
    ifstream  gifs;                                // create ifstream object 

    gifs.open ( gfname.c_str() );           // open ifstream 
    if (!gifs) {
        cerr << "\nERROR : failed to open input  file " << gfname << endl;
        exit (EXIT_FAILURE);
    }
    string line;
    getline( gifs, line );                             // Ignore header line
    while( getline( gifs, line ) )
    {
        stringstream l(line);
        Gyro g;
        l >> g.gtime >> g.x >> g.y >> g.z ;
        G.push_back( g );
        //cout << g.gtime << spc << g.x << spc << g.y << spc << g.z << endl;
    }
    gifs.close ();                                 // close ifstream 

    string    ffname = "framedata2";                 // input  file name 
    ifstream  fifs;                                // create ifstream object 
    fifs.open ( ffname.c_str() );           // open ifstream 
    if (!fifs) {
        cerr << "\nERROR : failed to open input  file " << ffname << endl;
        exit (EXIT_FAILURE);
    }

    getline( fifs, line );
    while( getline( fifs, line ) )
    {
        stringstream l(line);
        Frame f;
        l >> f.ftime >> f.frame;
        F.push_back( f );
    }
    fifs.close ();                                 // close ifstream 

    size_t i, j;
    for( i=0, j=0; i<F.size(); ++i )
    {
        Unit u;
        while( G[j].gtime<F[i].ftime )
            ++j;
        //cout << F[i].ftime << " " << G[j].gtime << endl;
        cout << F[i].frame << " "
             << G[j].x << " "
             << G[j].y << " "
             << G[j].z << " " 
             << F[i].ftime << " "
             << G[j].gtime << endl;
        u.frame_name = F[i].frame;
        u.x = G[j].x;
        u.y = G[j].y;
        u.z = G[j].z;
        U.push_back( u );
    }
    /* 
    for( size_t i=0; i<U.size(); ++i )
    {
        cout << U[i].frame_name << " " 
             << U[i].x << " "
             << U[i].y << " "
             << U[i].z << " "
             << endl;
    }
    */
	return 0;
}

