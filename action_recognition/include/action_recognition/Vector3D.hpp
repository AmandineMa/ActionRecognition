#ifndef VECTOR3D_H
#define	VECTOR3D_H

#include <vector>

#define VECTOR3D_SIZE 3

class Vector3D{

private:
  std::vector<double> vector3D_;

public:
  /* Constructor for an emtpy 3D vector*/
  Vector3D();
  /*Constructor for a 3D vector*/
  Vector3D(double x, double y, double z);
  Vector3D(std::vector<double> vector3D);

  void set_new_values(double x, double y, double z);

  double get_x(void);
  double get_y(void);
  double get_z(void);

  void set_x(double x);
  void set_y(double y);
  void set_z(double z);

  Vector3D normalize(void);
};


#endif
