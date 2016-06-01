#include <iostream>
#include <string>

#include "three_d_grid.h"

using namespace std;

string filename = "input.pcd";
int prec = 5;
string method = "sparse";

void parseCommandLine(int argc, char* argv[]);


int main(int argc, char* argv[]) {

    parseCommandLine(argc,argv);

    BaseGrid* grid;

    if(strcmp(method.c_str(),"dense") == 0) {
        DenseGrid dense(filename,prec);
        grid = &dense;
        grid->computeDistanceMap();
        grid->writeDataToFile();
    }
    else {
        AdaptiveGrid sparse(filename,prec);
        grid = &sparse;
        grid->computeDistanceMap();
        grid->writeDataToFile();
    }

    return 0;
}

void parseCommandLine(int argc, char *argv[])
{
    std::cout << "\n";
    std::cout << BOLD(FRED("Reconstruct implicit surface from a 3D data set.\n"));
    std::cout << "\n";

    if(argc == 1)
    {
        std::cout << FBLU("Syntax is: signed_distance_function input.pcd <options>\n");
        std::cout << "  where options are:\n";
        std::cout << "\t-p n" << "\t= grid precision (default: " << FCYN("10") << ")\n";
        std::cout << "\t-method X" << "\t= grid type: sparse/dense (default: " << FCYN("sparse") << ")\n";
        exit(0);
    }

    if(argc > 1)
    {
        filename = argv[1];

        for (int i = 2; i < argc; i++) {
            if(strcmp(argv[i],"-p") == 0)
                prec = atoi(argv[i+1]);
            if(strcmp(argv[i],"-method") == 0)
                method = argv[i+1];
        }
    }
}
