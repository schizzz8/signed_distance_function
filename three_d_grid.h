#pragma once

#include <iostream>
#include <math.h>
#include <limits>
#include <queue>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <dirent.h>
#include <utility>
#include <set>

#include <Eigen/Core>

#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <vtkSmartPointer.h>
#include <vtkVersion.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkPolyData.h>
#include <vtkImageData.h>
#include <vtkFloatArray.h>
#include <vtkIntArray.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkXMLImageDataWriter.h>

#include "colors.h"


typedef pcl::PointXYZ PointType;


class Cell {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Cell(const Eigen::Vector3i& idx=Eigen::Vector3i::Zero()):
        _idx(idx){
        _parent = 0;
        _distance = std::numeric_limits<float>::max();
    }

    inline bool operator < (const Cell& c) const {
        for (int i=0; i<3; i++){
            if (_idx[i]<c._idx[i])
                return true;
            if (_idx[i]>c._idx[i])
                return false;
        }
        return false;
    }

    inline bool operator == (const Cell& c) const {
        for (int i=0; i<3; i++)
            if(_idx[i] != c._idx[i])
                return false;
        return true;
    }

    inline const Eigen::Vector3i idx() const {return _idx;}
    inline const Eigen::Vector3f center() const {return _center;}
    inline const std::vector<Eigen::Vector3f> points() const {return _points;}
    inline const float distance() const {return _distance;}

    Eigen::Vector3i _idx;
    Eigen::Vector3f _center;
    std::vector<Eigen::Vector3f> _points;
    Cell* _parent;
    size_t _closest_point;
    float _distance;
    int _tag;
};

struct QEntry{
    QEntry(Cell* c=0, float d=std::numeric_limits<float>::max()) {
        _cell = c;
        _distance = d;
    }

    inline bool operator < (const QEntry& e) const {
        return e._distance > _distance ;
    }

    float _distance;
    Cell* _cell;
};

struct CellQueue : public std::priority_queue<QEntry> {
    typedef typename std::priority_queue<QEntry>::size_type size_type;
    CellQueue(size_type capacity = 0) { reserve(capacity); }
    inline void reserve(size_type capacity) { this->c.reserve(capacity); }
    inline size_type capacity() const { return this->c.capacity(); }
    inline Cell* top() { return std::priority_queue<QEntry>::top()._cell;}
    inline void push(Cell* c) { return std::priority_queue<QEntry>::push(QEntry(c, c->_distance));}
};

class BaseGrid {
public:
    BaseGrid(std::string filename="input.pcd", int prec = 5);
    virtual ~BaseGrid(){}
    inline float resolution(){ return _resolution;}
    inline const Eigen::Vector3i size(){ return _size;}
    inline const Eigen::Vector3f origin(){ return _origin;}
    inline int numCells(){ return _num_cells;}

    void toIdx(float x, float y, float z, Eigen::Vector3i& idx);
    void toWorld(int i, int j, int k, Eigen::Vector3f& point);
    int toInt(Eigen::Vector3i idx);
    void toIJK(int in, Eigen::Vector3i& idx);

    virtual bool hasCell(const Eigen::Vector3i& _idx){ return 0;}
    virtual int findNeighbors(Cell** neighbors, Cell* c){ return 0;}
    virtual void computeDistanceMap(float maxDistance=std::numeric_limits<float>::max()){}

    virtual void writeDataToFile(){}

protected:
    pcl::PointCloud<PointType>::Ptr _cloud;
    float _resolution;
    float _inverse_resolution;
    Eigen::Vector3f _origin;
    Eigen::Vector3i _size;
    int _num_cells;

private:
    double computeCloudResolution ();
    int getdigit(double number, int digit);
    void manageDirectories(std::string directory);
    int isDirectoryEmpty(const char *dirname);
};

class DenseGrid : public BaseGrid {
public:
    DenseGrid (std::string filename="input.pcd", int prec = 5);
    ~DenseGrid();
    void computeDistanceMap(float maxDistance=std::numeric_limits<float>::max());

    bool hasCell(const Eigen::Vector3i& idx_);
    int findNeighbors(Cell** neighbors, Cell* c);

    void writeDataToFile();

protected:
    Cell* _data;
    Cell*** _ptrs;
    int*** _index_image;
    int*** _imap;
};

class AdaptiveGrid : public BaseGrid {
public:
    AdaptiveGrid (std::string filename="input.pcd", int prec = 5);
    void computeDistanceMap(float maxDistance=std::numeric_limits<float>::max());

    int findNeighbors(Cell **neighbors, Cell *c);

    void writeDataToFile();

protected:
    std::set<Cell> _cells;
};
