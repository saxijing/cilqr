#include "Objects.h"

using namespace std;

Objects::Objects(): object_num(0), object_index(-1)
{
    cout<<"Objects constructed!"<<endl;
}

Objects::~Objects()
{
    for(int i=0; i<objects.size(); i++)
    {
        objects[i].~Object();
    }
    objects.clear();
    cout<<"Objects destructed!"<<endl;
}

void Objects::addObject(const Object& obj)
{
    objects.push_back(obj);
    object_num++;
}

int Objects::findObjectByID(const int id)
{
    for(int i=0; i<objects.size(); i++)
    {
        if(objects[i].getID() == id)
        {
            return i;
        }
    }
    return -1;
}

int Objects::findObjectByName(const string name)
{
    for(int i=0; i<objects.size(); i++)
    {
        if(objects[i].getName() == name)
        {
            return i;
        }
    }
    return -1;
}

void Objects::removeObjectByID(const int id)
{
    object_index = findObjectByID(id);
    if(object_index != -1)
    {
        objects.erase(objects.begin()+object_index);
        object_num--;
        cout<<"Object(ID) "<<id<<" removed!"<<endl;
    }
    else
        cout<<"Object(ID) "<<id<<" not found!"<<endl;
}

void Objects::removeObjectByIndex(const int obj_index)
{
    if(obj_index >= 0 && object_index < objects.size())
    {
        objects.erase(objects.begin()+object_index);
        object_num--;
        cout<<"Object(Index) "<<obj_index<<" removed!"<<endl;
    }
    else
        cout<<"Object index out of range!"<<endl;
}

void Objects::removeObjectByName(const string name)
{
    object_index = findObjectByName(name);
    if(object_index != -1)
    {
        objects.erase(objects.begin()+object_index);
        object_num--;
        cout<<"Object(Name) "<<name<<" removed!"<<endl;
    }
    else
        cout<<"Object(Name) "<<name<<" not found!"<<endl;
}

int Objects::getObjectsNum()
{
    return object_num;
}

const Object& Objects::getObjectByIndex(const int obj_index)
{
    if(obj_index >= 0 && obj_index < objects.size())
    {
        return objects[obj_index];
    }
    else
    {
        cout<<"Object index out of range!"<<endl;
        return objects[0];
    }
}

const Object& Objects::getObjectByID(const int id)
{
    object_index = findObjectByID(id);
    if(object_index != -1)
    {
        return objects[object_index];
    }
    else
    {
        cout<<"Object(ID) "<<id<<" not found!"<<endl;
        return objects[0];
    }
}

const Object& Objects::getObjectByName(const string name)
{
    object_index = findObjectByName(name);
    if(object_index != -1)
    {
        return objects[object_index];
    }
    else
    {
        cout<<"Object(Name) "<<name<<" not found!"<<endl;
        return objects[0];
    }
}

const vector<Object>& Objects::getObjects()
{
    return objects;
}