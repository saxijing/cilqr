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
        int findObjectByID(const int id) const;
        int findObjectByName(const string name) const;
        void removeObjectByID(const int id);
        void removeObjectByIndex(const int obj_index);
        void removeObjectByName(const string name);
        int getObjectsNum() const;
        const Object& getObjectByIndex(const int obj_index) const;
        const Object& getObjectByID(const int id) const;
        const Object& getObjectByName(const string name) const;
        const vector<Object>& getObjects() const;

        Object& getObjectByIndexForModify(const int obj_index);
        Object& getObjectByIDForModify(const int id);
        Object& getObjectByNameForModify(const string name);
        vector<Object>& getObjectsForModify();

    protected:
        vector<Object> objects;
        int object_num;
        int object_index;
};

#endif