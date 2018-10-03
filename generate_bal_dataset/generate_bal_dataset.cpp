#include <iostream>
#include <fstream>
#include <vector>
#include <stdlib.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define PI 3.14159265
#define G  9.80665

class camera{
public:
    camera(int &camID,
           Eigen::Vector3d &Rotation,
           Eigen::Vector3d &Position,
           Eigen::Vector3d &Distortion);
    int camID;
    Eigen::Vector3d R;
    Eigen::Vector3d t;
    Eigen::Vector3d distortion;
};

camera::camera(int &camID,
               Eigen::Vector3d &Rotation,
               Eigen::Vector3d &Position,
               Eigen::Vector3d &Distortion){
    this->camID = camID;
    this->R = Rotation;
    this->t = Position;
    this->distortion = Distortion;
}

typedef struct observation{
    int camID;
    Eigen::Vector2d coordinates;
} observation;

class point{
public:
    point(int id, Eigen::Vector3d &coordinates);
    int computeObservations(std::vector<camera> &cameraList, int &n_observations);
    int id_;
    Eigen::Vector3d coordinates_;
    std::vector<observation>  observations_;
};

point::point(int id, Eigen::Vector3d &coordinates){
    this->id_ = id;
    this->coordinates_ = coordinates;
}

int point::computeObservations(std::vector<camera> &cameraList, int &n_observations){
    if(!cameraList.empty()){
        for(int i=0; i<cameraList.size(); i++){
            double xp_cam  = this->coordinates_(0) - cameraList[i].t(0);
            double yp_cam  = this->coordinates_(1) - cameraList[i].t(1);
            double zp_cam  = this->coordinates_(2) - cameraList[i].t(2);
            Eigen::Vector2d pointObservation(-xp_cam/zp_cam, -yp_cam/zp_cam);
            observation temp;
            temp.camID = cameraList[i].camID;
            temp.coordinates = pointObservation;
            this->observations_.push_back(temp);
            n_observations++;
        }
    }
    else{
        std::cout << "Error: empty camera list.\n";
        return 0;
    }
}

int main(){
    std::vector<point> points;
    std::vector<camera> cameras;
    int n_observations = 0;


    // Generate Cameras
    std::cout << "GT Camera coordinates (expressed in W): \n";
    for(int id=0; id<4; id++){
        Eigen::Vector3d cameraRotation(0,0,0);
        Eigen::Vector3d cameraPosition(4+4*id,0,0);
        Eigen::Vector3d cameraDistortion(1,0,0);
        cameras.push_back(camera(id, cameraRotation, cameraPosition, cameraDistortion));
        std::cout << id << '\t'
                  << cameraPosition.transpose() << '\n';
    }
    std::cout << '\n';

    // Generate Points and compute observations
    std::cout << "GT Point coordinates (expressed in W): \n";
    int pointID = 0;
    // for(int x=0; x<=20; x+=4){
    //     for(int y=0; y<=20; y+=4){
    //         for(int z=4; z<=20; z+=4){
    for(int x=0; x<3; x++){
        for(int y=0; y<3; y++){
            for(int z=2; z<5; z++){
                Eigen::Vector3d pointCoordinates(x,y,z);
                std::cout << pointID << '\t'
                          << pointCoordinates.transpose() << '\n';
                points.push_back(point(pointID, pointCoordinates));
                points.back().computeObservations(cameras, n_observations);
                pointID++;
            }
        }
    }
    std::cout << '\n';

    // Write BAL dataset to file
    std::ofstream bal ("../Data/bal.txt");
    std::ofstream bal_gt ("../Data/bal_gt.txt");
    if (bal.is_open()){
        // Header line
        bal_gt << cameras.size() << '\t'
        << points.size()  << '\t'
        << n_observations << '\n';
        bal << cameras.size() << '\t'
            << points.size()  << '\t'
            << n_observations << '\n';
        // Point observations
        double nu, nv;
        for(int i=0; i<points.size(); i++){
            for(int j=0; j<points[i].observations_.size(); j++){
                nu = ((double) rand() / (RAND_MAX)) * 0.001;
                nv = ((double) rand() / (RAND_MAX)) * 0.001;
                bal_gt << points[i].observations_[j].camID << '\t'
                       << points[i].id_ << '\t'
                       << points[i].observations_[j].coordinates(0) << '\t'
                       << points[i].observations_[j].coordinates(1) << '\n';
                bal << points[i].observations_[j].camID << '\t'
                    << points[i].id_ << '\t'
                    << points[i].observations_[j].coordinates(0) + nu << '\t'
                    << points[i].observations_[j].coordinates(1) + nv << '\n';
            }
        }
        // Camera parameters
        double n1, n2, n3, n4, n5, n6, n7, n8, n9;
        for(int i=0; i<cameras.size(); i++){
            n1 = ((double) rand() / (RAND_MAX)) * 0.01;
            n2 = ((double) rand() / (RAND_MAX)) * 0.01;
            n3 = ((double) rand() / (RAND_MAX)) * 0.01;
            n4 = ((double) rand() / (RAND_MAX)) * 0.01;
            n6 = ((double) rand() / (RAND_MAX)) * 0.01;
            n5 = ((double) rand() / (RAND_MAX)) * 0.01;
            n7 = ((double) rand() / (RAND_MAX)) * 0.01;
            n8 = ((double) rand() / (RAND_MAX)) * 0;
            n9 = ((double) rand() / (RAND_MAX)) * 0;
            bal_gt << cameras[i].R(0) << '\n'
                   << cameras[i].R(1) << '\n'
                   << cameras[i].R(2) << '\n'
                   << -cameras[i].t(0) << '\n'
                   << -cameras[i].t(1) << '\n'
                   << -cameras[i].t(2) << '\n'
                   << cameras[i].distortion(0) << '\n'
                   << cameras[i].distortion(1) << '\n'
                   << cameras[i].distortion(2) << '\n';
            bal << cameras[i].R(0) + n1 << '\n'
                << cameras[i].R(1) + n2 << '\n'
                << cameras[i].R(2) + n3 << '\n'
                << -cameras[i].t(0) + n4 << '\n'
                << -cameras[i].t(1) + n5 << '\n'
                << -cameras[i].t(2) + n6 << '\n'
                << cameras[i].distortion(0) + n7 << '\n'
                << cameras[i].distortion(1) + n8 << '\n'
                << cameras[i].distortion(2) + n9 << '\n';
        }

        // Point coordinates
        double nx, ny, nz;
        for(int i=0; i<points.size(); i++){
            nx = ((double) rand() / (RAND_MAX)) * 0.5;
            ny = ((double) rand() / (RAND_MAX)) * 0.5;
            nz = ((double) rand() / (RAND_MAX)) * 0.5;
            bal_gt << points[i].coordinates_(0) << '\n'
                   << points[i].coordinates_(1) << '\n'
                   << points[i].coordinates_(2) << '\n';
            bal << points[i].coordinates_(0) + nx << '\n'
                << points[i].coordinates_(1) + ny << '\n'
                << points[i].coordinates_(2) + nz << '\n';
        }

        bal.close();
    }
    else{
        std::cout << "Error opening file \n";
    }

    return 1;
}
