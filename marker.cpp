#include "marker.h"

int ID;
Vec3d position, angle;
quat quaternion;


//getters-setters
int Marker::get_id()
{
	return ID;
}
void Marker::set_id(int id)
{
	ID = id;
}
Vec3d Marker::get_position()
{
	return position;
}
void Marker::set_position(Vec3d pos)
{
	position = pos;
}
Vec3d Marker::get_angle()
{
	return angle;
}
void Marker::set_angle(Vec3d orientation)
{
	angle = orientation;
}
quat Marker::get_quaternion()
{
	return quaternion;
}
void Marker::set_quaternion(quat q)
{
	quaternion = q;
}
void Marker::set_quaternion(Vec3d angle)
{
	vec3 eulerAngles = Vec3dtoVec3(angle); 
	quaternion = quat(eulerAngles);
}
vec3 Marker::Vec3dtoVec3 (Vec3d source)
{
	vec3 dest;
	for (int i=0; i<3; i++)
	{
		dest[i] = source(i);
	}
	return dest;
}
bool Marker::is_in_list(vector<Marker> marker_list, int markerID)
{
	for(int i =0; i<marker_list.size(); i++)
	{
		if (markerID == marker_list[i].get_id())
			return true;
	}
	return false;
}
void Marker::print()
{
	cout << "Marker ID = " << ID <<endl
		<< "DistanceVector = " << position << endl
		<< "AngleVector=" << angle << endl;
}