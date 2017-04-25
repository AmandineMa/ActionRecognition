#ifndef VECTOR3D_HPP
#define	VECTOR3D_HPP

#include <vector>

#define VECTOR3D_SIZE 3

class Vector3D{

private:
  std::vector<float> vector3D_;

public:
  /* Constructor for an emtpy 3D vector*/
  Vector3D();
  /*Constructor for a 3D vector*/
  Vector3D(float x, float y, float z);
  Vector3D(std::vector<float> vector3D);

  void set_new_values(float x, float y, float z);

  float get_x(void);
  float get_y(void);
  float get_z(void);

  void set_x(float x);
  void set_y(float y);
  void set_z(float z);

  Vector3D normalize(void);
};


#endif
