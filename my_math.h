#ifndef MY_MATH
#define MY_MATH
#include <math.h>

struct Vector3 {
    float x, y, z;
};

struct Matrix3 {
    float m[3][3];

    Matrix3 operator*(const Matrix3& other) const {
        Matrix3 result;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.m[i][j] = 0.0f;
                for (int k = 0; k < 3; ++k) {
                    result.m[i][j] += m[i][k] * other.m[k][j];
                }
            }
        }
        return result;
    }

    Vector3 operator*(const Vector3& v) const {
        Vector3 result;
        result.x = m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z;
        result.y = m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z;
        result.z = m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z;
        return result;
    }
};

Matrix3 createRotationMatrix(float angleX, float angleY, float angleZ) {
    // Преобразование углов в радианы
    float radX = angleX * M_PI / 180.0f;
    float radY = angleY * M_PI / 180.0f;
    float radZ = angleZ * M_PI / 180.0f;

    // Создание матрицы поворота
    Matrix3 rotationMatrix;

    rotationMatrix.m[0][0] = cos(radY) * cos(radZ);
    rotationMatrix.m[0][1] = -cos(radY) * sin(radZ);
    rotationMatrix.m[0][2] = sin(radY);
    
    rotationMatrix.m[1][0] = cos(radX) * sin(radZ) + sin(radX) * sin(radY) * cos(radZ);
    rotationMatrix.m[1][1] = cos(radX) * cos(radZ) - sin(radX) * sin(radY) * sin(radZ);
    rotationMatrix.m[1][2] = -sin(radX) * cos(radY);
    
    rotationMatrix.m[2][0] = sin(radX) * sin(radZ) - cos(radX) * sin(radY) * cos(radZ);
    rotationMatrix.m[2][1] = sin(radX) * cos(radZ) + cos(radX) * sin(radY) * sin(radZ);
    rotationMatrix.m[2][2] = cos(radX) * cos(radY);
    
    return rotationMatrix;
}

#endif
