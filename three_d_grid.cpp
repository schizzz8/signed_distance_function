#include "three_d_grid.h"

using namespace std;

BaseGrid::BaseGrid(string filename, int prec) : _cloud(new pcl::PointCloud<PointType>){
    pcl::io::loadPCDFile(filename.c_str(),*_cloud);
    double res = computeCloudResolution();
    PointType min_pt,max_pt;
    pcl::getMinMax3D(*_cloud,min_pt,max_pt);

    cout << "\n";
    cout << BOLD(FBLU("Loading the Data-Set:\n"));
    cout << "\t>> Filename: " << filename << "\n";
    cout << "\t>> Points: " << _cloud->size() << "\n";
    cout << "\t>> Min: (" << min_pt.x << "," << min_pt.y << "," << min_pt.z << ")\n";
    cout << "\t>> Max: (" << max_pt.x << "," << max_pt.y << "," << max_pt.z << ")\n";
    cout << "\t>> Average distance: " << res << "m\n";
    cout << "--------------------------------------------------------------------------------\n";
    cout << "\n";

    bool found = false;
    int i = 2;
    while(found == false)
    {
        i--;
        if(getdigit(res,i) != 0)
            found = true;
    }
    _resolution = (100/prec)*pow(10,i);
    _inverse_resolution = 1./_resolution;

    _origin.x() = min_pt.x - 5*_resolution;
    _origin.y() = min_pt.y - 5*_resolution;
    _origin.z() = min_pt.z - 5*_resolution;

    _size.x() = ((max_pt.x+5*_resolution)-_origin.x())*_inverse_resolution;
    _size.y() = ((max_pt.y+5*_resolution)-_origin.y())*_inverse_resolution;
    _size.z() = ((max_pt.z+5*_resolution)-_origin.z())*_inverse_resolution;

    _num_cells = _size.x()*_size.y()*_size.z();

    size_t lastindex = filename.find_last_of(".");
    std::string directory = filename.substr(0,lastindex);
    manageDirectories(directory);

    pcl::PCLPointCloud2 out;
    pcl::toPCLPointCloud2(*_cloud,out);
    pcl::io::saveVTKFile("data_set.vtk",out);

}

void BaseGrid::toIdx(float x, float y, float z, Eigen::Vector3i &idx) {
    idx.x() = floor((x - _origin.x())*_inverse_resolution);
    idx.y() = floor((y - _origin.y())*_inverse_resolution);
    idx.z() = floor((z - _origin.z())*_inverse_resolution);
}

void BaseGrid::toWorld(int i, int j, int k, Eigen::Vector3f &point) {
    point.x() = _origin.x() + i*_resolution;
    point.y() = _origin.y() + j*_resolution;
    point.z() = _origin.z() + k*_resolution;
}

int BaseGrid::toInt(Eigen::Vector3i idx) {
    return (idx.z() + idx.y()*_size.z() + idx.x()*_size.y()*_size.z());
}

void BaseGrid::toIJK(int in, Eigen::Vector3i &idx) {
    div_t divresult = div(in,_size.x());
    idx.x() = divresult.rem;
    divresult = div(divresult.quot,_size.y());
    idx.y() = divresult.rem;
    divresult = div(divresult.quot,_size.z());
    idx.z() = divresult.rem;
}

double BaseGrid::computeCloudResolution() {
    double res = 0.0;
    int n_points = 0;

    std::vector<int> indices (2);
    std::vector<float> sqr_distances (2);
    pcl::KdTreeFLANN<PointType> tree;
    tree.setInputCloud (_cloud);

    for (size_t i = 0; i < _cloud->size (); ++i)
    {
        if (! pcl_isfinite ((*_cloud)[i].x))
        {
            continue;
        }
        tree.nearestKSearch (i, 2, indices, sqr_distances);
        res += sqrt (sqr_distances[1]);
        ++n_points;

    }
    if (n_points != 0)
    {
        res /= n_points;
    }
    return res;
}

int BaseGrid::getdigit(double number, int digit) {
    div_t divresult = div(number/pow(10, digit),10);
    return  divresult.rem;
}

void BaseGrid::manageDirectories(std::string directory)
{
    struct stat info;
    if( stat( directory.c_str(), &info ) != 0 )
    {
        mkdir(directory.c_str(),0700);
    }
    else if( info.st_mode & S_IFDIR )
    {
        if(isDirectoryEmpty(directory.c_str()) == 0)
        {
            DIR *theFolder = opendir(directory.c_str());
            struct dirent *next_file;
            char filepath[256];

            while ( (next_file = readdir(theFolder)) != NULL )
            {
                if (0==strcmp(next_file->d_name, ".") || 0==strcmp(next_file->d_name, "..")) { continue; }
                char cwd[1024];
                getcwd(cwd, sizeof(cwd));
                sprintf(filepath, "%s/%s", directory.c_str(), next_file->d_name);
                remove(filepath);
            }
            closedir(theFolder);
        }
    }
    else
        printf( "%s is no directory\n", directory.c_str() );

    chdir(directory.c_str());

}

int BaseGrid::isDirectoryEmpty(const char *dirname)
{
    int n = 0;
    struct dirent *d;
    DIR *dir = opendir(dirname);
    if (dir == NULL)
        return 1;
    while ((d = readdir(dir)) != NULL) {
        if(++n > 2)
            break;
    }
    closedir(dir);
    if (n <= 2)
        return 1;
    else
        return 0;
}

DenseGrid::DenseGrid(string filename, int prec): BaseGrid(filename,prec) {

    cout << BOLD(FBLU("Building the 3D Grid (Dense):\n"));
    cout << "\t>> Delta: " << _resolution << "m\n";
    cout << "\t>> Grid dimensions: (" << _size.x() << "," << _size.y() << "," << _size.z() << ")\t total: " << _num_cells << "\n";
    cout << "\t>> Origin: (" << _origin.x() << "," << _origin.y() << "," << _origin.z() << ")\n";
    cout << "--------------------------------------------------------------------------------\n";
    cout << "\n";

    _data = new Cell[_num_cells];
    _ptrs = new Cell**[_size.x()];
    for(size_t x=0; x < _size.x(); x++) {
        _ptrs[x] = new Cell*[_size.y()];
        for(size_t y=0; y < _size.y(); y++) {
            _ptrs[x][y] = _data + y*_size.z() + x*_size.y()*_size.z();
            for(size_t z=0; z < _size.z(); z++)
                _data[z + y*_size.z() + x*_size.y()*_size.z()]._idx = Eigen::Vector3i (x,y,z);
        }
    }

    _index_image = new int**[_size.x()];
    for(size_t x=0; x < _size.x(); x++) {
        _index_image[x] = new int*[_size.y()];
        for(size_t y=0; y < _size.y(); y++) {
            _index_image[x][y] = new int[_size.z()];
            for(size_t z=0; z < _size.z(); z++)
                _index_image[x][y][z] = -1;
        }
    }

    for(size_t ii=0; ii < _cloud->size(); ii++) {
        Eigen::Vector3i idx;
        toIdx(_cloud->at(ii).x,_cloud->at(ii).y,_cloud->at(ii).z,idx);
        if(hasCell(idx)) {
            int linear_idx = toInt(idx);
            _index_image[idx.x()][idx.y()][idx.z()] = linear_idx;
        }
        else {
            cout << "Error\n";
            cout << "Point (" << _cloud->at(ii).x << "," << _cloud->at(ii).y << "," << _cloud->at(ii).z << ")\t";
            cout << "Cell (" << idx.x() << "," << idx.y() << "," << idx.z() << "\n";
        }
    }

    _imap = new int**[_size.x()];
    for(size_t x=0; x < _size.x(); x++) {
        _imap[x] = new int*[_size.y()];
        for(size_t y=0; y < _size.y(); y++) {
            _imap[x][y] = new int[_size.z()];
            for(size_t z=0; z < _size.z(); z++)
                _imap[x][y][z] = -1;
        }
    }

}

DenseGrid::~DenseGrid() {
    for(size_t x=0; x < _size.x(); x++) {
        delete [] _ptrs[x];
    }
    delete [] _ptrs;
    delete [] _data;

    for (size_t x=0; x < _size.x(); x++) {
        for (size_t y=0; y < _size.y(); y++)
            delete[] _index_image[x][y];
        delete[] _index_image[x];
    }
    delete[] _index_image;

    for (size_t x=0; x < _size.x(); x++) {
        for (size_t y=0; y < _size.y(); y++)
            delete[] _imap[x][y];
        delete[] _imap[x];
    }
    delete[] _imap;

}

bool DenseGrid::hasCell(const Eigen::Vector3i &idx_)  {
    (idx_.x() >= 0 && idx_.x() <= _size.x()
            && idx_.y() >= 0 && idx_.y() <= _size.y()
            && idx_.z() >= 0 && idx_.z() <= _size.z()) ?  true : false;

}

int DenseGrid::findNeighbors(Cell **neighbors, Cell *c) {
    int x = c->_idx.x();
    int y = c->_idx.y();
    int z = c->_idx.z();
    int xmin = (x-1<0) ? 0 : x-1;
    int xmax = (x+1>_size.x()-1) ? _size.x()-1 : x+1;
    int ymin = (y-1<0) ? 0 : y-1;
    int ymax = (y+1>_size.y()-1) ? _size.y()-1 : y+1;
    int zmin = (z-1<0) ? 0 : z-1;
    int zmax = (z+1>_size.z()-1) ? _size.z()-1 : z+1;
    int k=0;
    for(size_t xx=xmin; xx <= xmax; xx++)
        for(size_t yy=ymin; yy <= ymax; yy++)
            for(size_t zz=zmin; zz <= zmax; zz++)
                if(xx != x || yy != y || zz != z) {
                    neighbors[k] = &_ptrs[xx][yy][zz];
                    k++;
                }
    return k;
}

void DenseGrid::computeDistanceMap(float maxDistance) {
    cout << BOLD(FBLU("Computing Distance Function:\n"));
    cerr << "\t>> Time elapsed: ";
    std::clock_t t0 = clock();
    CellQueue q;
    for(size_t x=0; x < _size.x(); x++)
        for(size_t y=0; y < _size.y(); y++)
            for(size_t z=0; z < _size.z(); z++) {
                Cell& cell = _ptrs[x][y][z];
                int idx = _index_image[x][y][z];
                if(idx > -1) {
                    cell._parent = &cell;
                    cell._distance = 0;
                    q.push(&cell);
                    _imap[x][y][z] = idx;
                }
            }
    Cell* neighbors[26];
    size_t maxQSize = q.size();

    while(!q.empty()) {
        Cell* current = q.top();
        Cell* parent = current->_parent;
        int parentIndex = _imap[parent->_idx.x()][parent->_idx.y()][parent->_idx.z()];
        q.pop();
        int k = findNeighbors(neighbors,current);
        for(int ii=0; ii<k; ii++) {
            Cell* child = neighbors[ii];
            int i = child->_idx.x();
            int j = child->_idx.y();
            int k = child->_idx.z();
            int di = i - parent->_idx.x();
            int dj = j - parent->_idx.y();
            int dk = k - parent->_idx.z();
            int d = (di*di + dj*dj + dk*dk);
            if(d<maxDistance && child->_distance>d) {
                child->_parent = parent;
                _imap[i][j][k] = parentIndex;
                child->_distance = d;
                q.push(child);
            }
        }
        maxQSize = maxQSize < q.size() ? q.size() : maxQSize;
    }

    std::clock_t t1 = clock();
    double elapsed_time1 = double(t1 - t0)/CLOCKS_PER_SEC;
    cout << elapsed_time1 << "s\n";
    cout << "--------------------------------------------------------------------------------\n";
    cout << "\n";
}

void DenseGrid::writeDataToFile() {
    vtkSmartPointer<vtkImageData> imageData = vtkSmartPointer<vtkImageData>::New();
    imageData->SetDimensions(_size.x(),_size.y(),_size.z());
    imageData->SetOrigin(_origin.x(),_origin.y(),_origin.z());
    imageData->SetSpacing(_resolution,_resolution,_resolution);
    imageData->SetExtent(0,_size.x()-1,0,_size.y()-1,0,_size.z()-1);
#if VTK_MAJOR_VERSION <= 5
    imageData->SetNumberOfScalarComponents(1);
    imageData->SetScalarTypeToFloat();
#else
    imageData->AllocateScalars(VTK_FLOAT, 1);
#endif

    vtkSmartPointer<vtkFloatArray> dist = vtkSmartPointer<vtkFloatArray>::New();
    dist->SetName("distance_function");

    for(int z = 0; z < _size.z(); z++)
        for(int y = 0; y < _size.y(); y++)
            for(int x = 0; x < _size.x(); x++)
            {
                Cell& cell = _ptrs[x][y][z];
                dist->InsertNextValue(std::sqrt(cell._distance));
            }

    imageData->GetPointData()->AddArray(dist);
    imageData->Update();

    vtkSmartPointer<vtkIntArray> image = vtkSmartPointer<vtkIntArray>::New();
    image->SetName("index_image");

    for(size_t z=0; z < _size.z(); z++)
        for(size_t y=0; y < _size.y(); y++)
            for(size_t x=0; x < _size.x(); x++)
                image->InsertNextValue(_index_image[x][y][z]);

    imageData->GetPointData()->AddArray(image);
    imageData->Update();

    vtkSmartPointer<vtkXMLImageDataWriter> writer = vtkSmartPointer<vtkXMLImageDataWriter>::New();
    writer->SetFileName("solution.vti");
#if VTK_MAJOR_VERSION <= 5
    writer->SetInputConnection(imageData->GetProducerPort());
#else
    writer->SetInputData(imageData);
#endif
    writer->Write();

}

AdaptiveGrid::AdaptiveGrid(string filename, int prec) : BaseGrid(filename,prec) {

    for(size_t ii=0; ii < _cloud->size(); ii++) {

        float x=_cloud->at(ii).x,y=_cloud->at(ii).y,z=_cloud->at(ii).z;

        Eigen::Vector3i idx;
        toIdx(x,y,z,idx);

        set<Cell>::iterator it = _cells.find(idx);
        if(it != _cells.end()) {
            Cell* cell = const_cast<Cell*>(&(*it));
            cell->_points.push_back(Eigen::Vector3f(x,y,z));
        }
        else {
            Cell c(idx);
            c._points.push_back(Eigen::Vector3f(x,y,z));
            c._distance = 0.0;
            _cells.insert(c);
        }
    }

    cout << BOLD(FBLU("Building the 3D Grid (Sparse):\n"));
    cout << "\t>> Delta: " << _resolution << "m\n";
    cout << "\t>> Grid dimensions: (" << _size.x() << "," << _size.y() << "," << _size.z() << ")\t total: " << _num_cells << "\n";
    cout << "\t>> Origin: (" << _origin.x() << "," << _origin.y() << "," << _origin.z() << ")\n";
    cout << "\t>> Active cells: " << _cells.size() << "\n";
    cout << "--------------------------------------------------------------------------------\n";
    cout << "\n";
}

int AdaptiveGrid::findNeighbors(Cell **neighbors, Cell *c) {
    int x = c->_idx.x();
    int y = c->_idx.y();
    int z = c->_idx.z();
    int xmin = (x-1<0) ? 0 : x-1;
    int xmax = (x+1>_size.x()-1) ? _size.x()-1 : x+1;
    int ymin = (y-1<0) ? 0 : y-1;
    int ymax = (y+1>_size.y()-1) ? _size.y()-1 : y+1;
    int zmin = (z-1<0) ? 0 : z-1;
    int zmax = (z+1>_size.z()-1) ? _size.z()-1 : z+1;
    int k=0;
    for(size_t xx=xmin; xx <= xmax; xx++)
        for(size_t yy=ymin; yy <= ymax; yy++)
            for(size_t zz=zmin; zz <= zmax; zz++)
                if(xx != x || yy != y || zz != z) {
                    Eigen::Vector3i idx(xx,yy,zz);
                    set<Cell>::iterator it = _cells.find(idx);
                    if (it != _cells.end()) {
                        neighbors[k] = const_cast<Cell*>(&(*it));
                        k++;
                    }
                    else {
                        pair<set<Cell>::iterator,bool> ret;
                        ret = _cells.insert(Cell (idx));
                        if (ret.second == true)
                            neighbors[k] = const_cast<Cell*>(&(*(ret.first)));
                        k++;
                    }
                }
    return k;
}

void AdaptiveGrid::computeDistanceMap(float maxDistance) {
    cout << BOLD(FBLU("Computing Distance Function:\n"));
    cerr << "\t>> Time elapsed: ";
    std::clock_t t0 = clock();
    CellQueue q;
    for(set<Cell>::iterator it = _cells.begin(); it != _cells.end(); ++it) {
        Cell* cell = const_cast<Cell*>(&(*it));
        cell->_parent = cell;
        cell->_distance = 0;
        q.push(cell);
    }

//    int loop = 3;
//    Cell* last = 0;
//    CellQueue dummy(q);
//    while (!dummy.empty()) {
//        last = dummy.top();
//        dummy.pop();
//    }

    bool stop = false;

    Cell* neighbors[26];

    while(stop == false) {
        Cell* current = q.top();
        Cell* parent = current->_parent;
        q.pop();
        int k = findNeighbors(neighbors,current);
        for(int ii=0; ii<k; ii++) {
            Cell* child = neighbors[ii];
            int i = child->_idx.x();
            int j = child->_idx.y();
            int k = child->_idx.z();
            int di = i - parent->_idx.x();
            int dj = j - parent->_idx.y();
            int dk = k - parent->_idx.z();
            int d = (di*di + dj*dj + dk*dk);
            if(d<maxDistance && child->_distance>d) {
                child->_parent = parent;
                child->_distance = d;
                q.push(child);
            }
            if(d > 5)
                stop = true;

        }
//        if(*last == *current) {
//            dummy = q;
//            while (!dummy.empty()) {
//                last = dummy.top();
//                dummy.pop();
//            }
//            loop--;
//        }

    }
    std::clock_t t1 = clock();
    double elapsed_time1 = double(t1 - t0)/CLOCKS_PER_SEC;
    cout << elapsed_time1 << "s\n";
    cout << "--------------------------------------------------------------------------------\n";
    cout << "\n";
}

void AdaptiveGrid::writeDataToFile() {
    vtkSmartPointer<vtkImageData> imageData = vtkSmartPointer<vtkImageData>::New();
    imageData->SetDimensions(_size.x(),_size.y(),_size.z());
    imageData->SetOrigin(_origin.x(),_origin.y(),_origin.z());
    imageData->SetSpacing(_resolution,_resolution,_resolution);
    imageData->SetExtent(0,_size.x()-1,0,_size.y()-1,0,_size.z()-1);
#if VTK_MAJOR_VERSION <= 5
    imageData->SetNumberOfScalarComponents(1);
    imageData->SetScalarTypeToFloat();
#else
    imageData->AllocateScalars(VTK_FLOAT, 1);
#endif

    vtkSmartPointer<vtkFloatArray> dist = vtkSmartPointer<vtkFloatArray>::New();
    dist->SetName("distance_function");
    dist->SetNumberOfComponents(1);
    dist->SetNumberOfValues(_num_cells);

    for(vtkIdType i=0; i < dist->GetNumberOfTuples(); i++) {
        Eigen::Vector3i idx;
        toIJK(i,idx);
        set<Cell>::iterator it = _cells.find(idx);
        if (it != _cells.end())
            dist->SetValue(i,it->_distance);
        else
            dist->SetValue(i,100.0);
    }
    imageData->GetPointData()->AddArray(dist);
    imageData->Update();

    vtkSmartPointer<vtkIntArray> image = vtkSmartPointer<vtkIntArray>::New();
    image->SetName("index_image");
    image->SetNumberOfComponents(1);
    image->SetNumberOfValues(_num_cells);

    for(vtkIdType i=0; i < image->GetNumberOfTuples(); i++) {
        Eigen::Vector3i idx;
        toIJK(i,idx);
        set<Cell>::iterator it = _cells.find(idx);
        if (it != _cells.end())
            image->SetValue(i,i);
        else
            image->SetValue(i,-1);
    }
    imageData->GetPointData()->AddArray(image);
    imageData->Update();

    vtkSmartPointer<vtkXMLImageDataWriter> writer = vtkSmartPointer<vtkXMLImageDataWriter>::New();
    writer->SetFileName("solution.vti");
#if VTK_MAJOR_VERSION <= 5
    writer->SetInputConnection(imageData->GetProducerPort());
#else
    writer->SetInputData(imageData);
#endif
    writer->Write();

}
