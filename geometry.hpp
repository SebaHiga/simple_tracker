#pragma once

#include <math.h>

namespace geom {

    template<typename T>
    class Vector{
    public:
        T angle;
        T module;

        // Vector() : angle(0), module(0){};

        void update(Vector v){
            module = (module + v.module)/2;
            angle = (angle + v.angle)/2;  
        }

        void operator=(Vector v){
            module = v.module;
            angle = v.angle; 
        }

        template<typename F>
        void operator=(F num){
            module = (T) num;
            angle = (T) num; 
        }

        template<typename F>
        void operator/=(F div){
            module /= (T) div;
            angle /= (T) div; 
        }

        void operator+=(Vector v){
            module += v.module;
            angle += v.angle; 
        }
    };

    template<typename T>
    class Point {
    private:
        using vector_f = Vector<float>;
    public:
        T top, left;
        bool updated;

        Point() : top(0), left(0), updated(true){

        }

        Point(T _top, T _left) : top(_top), left(_left), updated(true){

        }

        float getDistance (Point p){
            return hypot(p.top-top, p.left-left);
        }

        vector_f getVelocity (Point p){
            // if new vector then return 0 vel
            if(!top || !left){
                vector_f v;
                return v;
            }

            float module = hypot(p.top - top, p.left - left);
            float angle = atan2f32(p.top - top, p.left - left);
            
            return {module, angle};
        }

        float originDistance (void){
            return hypot(top, left);
        }

        void update (Point point){
            top = point.top;
            left = point.left;
        }

        void update (vector_f speed){
            top = top + speed.module * sin(speed.angle);
            left = left + speed.module * cos(speed.angle);
        }

        void operator=(Point p){
            top = p.top;
            left = p.left;
        }

        // opencv format

        cv::Point cv_getPoint(){
            return cv::Point(top, left);
        }
    };



}