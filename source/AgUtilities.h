#ifndef AGUTILITIES
#define AGUTILITIES

#include <UnigineMathLib.h>

void printMatrix(const char *name, Unigine::Math::mat4 mat)
{
  int len = strlen(name);

  printf("%s: %.2f %.2f %.2f %.2f\n", name, mat[0], mat[1], mat[2], mat[3]);
  int i = len;
  for (i = 0; i < len + 2; ++i) printf(" ");
  printf("%.2f %.2f %.2f %.2f\n", mat[4], mat[5], mat[6], mat[7]);
  for (i = 0; i < len + 2; ++i) printf(" ");
  printf("%.2f %.2f %.2f %.2f\n", mat[8], mat[9], mat[10], mat[11]);
  for (i = 0; i < len + 2; ++i) printf(" ");
  printf("%.2f %.2f %.2f %.2f\n", mat[12], mat[13], mat[14], mat[15]);
}

inline void wrapAngle(float &a)
{
  while (a >= 180.f) a -= 360.f;
  while (a < -180.f) a += 360.f;
}

inline Unigine::Math::Vec2 getVector2FromAngle(float degrees)
{
  degrees *= M_PI / 180.f;
  return Unigine::Math::Vec2(0 * Unigine::Math::cos(degrees) - 1.f * Unigine::Math::sin(degrees),
                             0 * Unigine::Math::sin(degrees) + 1.f * Unigine::Math::cos(degrees));
}

#endif /* AGUTILITIES */
