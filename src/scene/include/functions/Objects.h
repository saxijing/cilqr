#ifndef OBJECTS_H_
#define OBJECTS_H_

#include "Object.h"

using namespace std;

class Objects
{
    public:
        Objects();
        ~Objects();
        void addObject(const Object& obj);
        int findObjectByID(const int id);
        int findObjectByName(const string name);
        void removeObjectByID(const int id);
        void removeObjectByIndex(const int obj_index);
        void removeObjectByName(const string name);
        int getObjectsNum();
        const Object& getObjectByIndex(const int obj_index);
        const Object& getObjectByID(const int id);
        const Object& getObjectByName(const string name);
        const vector<Object>& getObjects();
    protected:
        vector<Object> objects;
        int object_num;
        int object_index;
};

#endif