#include <iostream>

#include "three_d_grid.h"

using namespace std;

std::string filename = "input.pcd";
int prec = 5;

void parseCommandLine(int argc, char* argv[]);


int main(int argc, char* argv[]) {

    parseCommandLine(argc,argv);

    AdaptiveGrid grid(filename,prec);
    grid.computeDistanceMap();
    grid.writeDataToFile();

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
        exit(0);
    }

    if(argc > 1)
    {
        filename = argv[1];

        for (int i = 2; i < argc; i++) {
            if(strcmp(argv[i],"-p") == 0)
                prec = atoi(argv[i+1]);
        }
    }
}
