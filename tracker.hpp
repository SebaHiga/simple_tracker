#pragma once

#include "geometry.hpp"
#include <iostream>
#include <vector>
#include <stdlib.h>
#include "kalman.hpp" 
#include <deque>  

#define MAX_DISS 60
#define MIN_APP 5
#define MAX_DOTS 900
#define MAX_RANGE 120

//max range

using point_i = geom::Point<int>;

class Object {
public:
    int id;
    int class_id;
    float max_dist;
    bool updated;

    int b, g, r;

    int min_appearance;
    bool valid;

    point_i position;
    point_i position_old;

    std::deque<point_i> trackline;
    std::deque<point_i> trackline_nonkf;

    Kalman kf;

    Object();

    Object(int id, point_i pos) :   id(id), position(pos), updated(true), class_id(0),
                                        max_dist(MAX_DISS), min_appearance(MIN_APP), valid(false){
        trackline.push_back(pos);
        b = rand() % 256;
        g = rand() % 256;
        r = rand() % 256;

        kf.init(pos.top, pos.left, 0.1);
    }
    Object(int class_id, int id, point_i pos) :   id(id), position(pos), updated(true), class_id(0),
                                        max_dist(MAX_DISS), min_appearance(MIN_APP), valid(false){
        trackline.push_back(pos);
        b = rand() % 256;
        g = rand() % 256;
        r = rand() % 256;

    }

    ~Object(){};
    bool dissapeared(){
        if(valid){
            if(max_dist){
                max_dist--;
                position_old = position;

                kf.predict();
                auto [top, left] = kf.getPosition();
                position.top = (int) top;
                position.left = (int) left;

                trackline.push_back(position);
                
                if(trackline.size() > MAX_DOTS){
                    trackline.erase(trackline.begin());
                }

                return false;
            }
            else{
                return true;
            }
        }
        return true;
    }

    void update(point_i p){
        position_old = position;
        trackline_nonkf.push_back(p);

        kf.predict();
        kf.update(p.top, p.left);

        auto [top, left] = kf.getPosition();
        p.top = (int) top;
        p.left = (int) left;

        position = p;

        updated = true;

        max_dist = MAX_DISS;

        trackline.push_back(p);
        if(trackline.size() > MAX_DOTS){
            trackline.erase(trackline.begin());
            trackline_nonkf.erase(trackline_nonkf.begin());
        }

        if(min_appearance){
            min_appearance--;
            if(!min_appearance){
                valid = true;
            }
        }
    }

    void update(Object obj){
        valid = true;
        updated = true;
        max_dist = MAX_DISS;
        position = obj.position;

        trackline.push_back(obj.position);
        if(trackline.size() > MAX_DOTS){
            trackline.erase(trackline.begin());
        }
    }

    cv::Scalar getColor(){
        return cv::Scalar(b, g, r);
    }
};

class Tracker {
private:
    static const int maxRange = MAX_RANGE;

public:
    std::deque<Object> objects;
    Tracker(void){};
    ~Tracker(){};

    // returns the total difference of tracked objects after updating
    int update(std::vector<point_i> &points);
    void addObject(point_i p);
    void addObject(Object obj);
    int getNewID();

    bool isNew(int object_id);
};


int Tracker::update(std::vector<point_i> &points){
    for (auto &object : objects){
        object.updated = false;
    }

    for (auto &point : points){
        point.updated = false;
    }

    // valid priority analysis
    bool checking_valid = true;
    do{
        for(auto &point : points){
            if(point.updated){
                continue;
            }

            float min_dist = MAX_DISS;
            std::deque<Object>::iterator it;
            for(auto object = objects.begin(); object != objects.end(); object++){
                if(object->updated or !(checking_valid xor object->valid)){
                    continue;
                }

                if(auto dist = point.getDistance(object->position); dist < min_dist){
                    min_dist = dist;
                    it = object;
                }
            }
            
            if(min_dist < MAX_DISS){
                it->update(point);
                point.updated = true;
            }
            else if(!checking_valid){
                addObject(point);
                point.updated = true;
            }
            
        }

        checking_valid = !checking_valid;
    }while(!checking_valid);

    //check lost
    for(auto it = objects.begin(); it != objects.end();){
        if(!it->updated and it->dissapeared()){
            it = objects.erase(it);
        }
        else{
            it++;
        }
    }

    return 1;
}

bool Tracker::isNew(int object_id){
    for (int i = 0; i < objects.size(); i++){
        if(objects[i].id == object_id){
            return false;
        }
    }

    return true;
}

void Tracker::addObject(point_i p){
    objects.push_back(Object(getNewID(), p));
}

void Tracker::addObject(Object obj){
    objects.push_back(obj);
}

int Tracker::getNewID(){
    std::vector<int> vct(10);
    int id;
    bool used;

    for (const auto &object : objects){
        vct.push_back(object.id);
    }

    for (id = 0; id < 100; id++){
        used = false;

        for(const auto &it : vct){
            if(id == it){
                used = true;
            }
        }

        if(!used){
            break;
        }
    }

    return id;
}