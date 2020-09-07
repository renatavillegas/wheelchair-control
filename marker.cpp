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
void fprintf()
{
	cout << "Marker ID = " << ID;
}