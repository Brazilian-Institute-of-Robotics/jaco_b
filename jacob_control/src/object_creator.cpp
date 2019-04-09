#include <jacob_control/object_creator.h>

ObjectCreator::ObjectCreator(Geometry type, std::string name) : PlanningGroup(_name){
    _name = "objects_collision";
    createObject(type,name);

}

void ObjectCreator::setPrimitive(moveit_msgs::CollisionObject object, Geometry type){
    shape_msgs::SolidPrimitive object_primitive;

    switch (type){
        case BOX:
            object_primitive.type = object_primitive.BOX;
            object_primitive.dimensions.resize(3);
            object_primitive.dimensions[0] = 3.0;
            object_primitive.dimensions[1] = 3.0;
            object_primitive.dimensions[2] = 0.0;
            break;
    }
    object.primitives.push_back(object_primitive);
}

void ObjectCreator::createObject(Geometry type, std::string name){
    moveit_msgs::CollisionObject object;
    object.id = name; //ex: "plane"
    object.header.frame_id = getPlanningFrame();

    setPrimitive(object,type);

    switch (type){
        case BOX:
            break;
    }

}

Geometry type;
PlanningGroup pgroup;


int main (int argc, char** argv){
    ObjectCreator object(BOX, "plane");
}