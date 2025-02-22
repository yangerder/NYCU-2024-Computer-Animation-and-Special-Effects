#include "glut.h"

bool mouseLeftDown;
bool mouseRightDown;
bool mouseMiddleDown;
float mouseX, mouseY;
float cameraDistanceX;
float cameraDistanceY;
float cameraAngleX;
float cameraAngleY;
float times = 1;

void mouseCB(int button, int state, int x, int y) {
    mouseX = x;
    mouseY = y;
    times = 1;

    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            mouseLeftDown = true;
        } else if (state == GLUT_UP)
            mouseLeftDown = false;
    }

    else if (button == GLUT_RIGHT_BUTTON) {
        if (state == GLUT_DOWN) {
            mouseRightDown = true;
        } else if (state == GLUT_UP)
            mouseRightDown = false;
    }
    /* else if (button == GLUT_MIDDLE_BUTTON)
    {
        if (state == GLUT_DOWN)
        {
            mouseMiddleDown = true;
        }
        else if (state == GLUT_UP)
            mouseMiddleDown = false;
    }*/

    /* else if (state == GLUT_UP && button == GLUT_WHEEL_UP) {
        times = 0.008f + 1;
        glutPostRedisplay();
    } else if (state == GLUT_UP && button == GLUT_WHEEL_DOWN) {
        times = -0.008f + 1;
        glutPostRedisplay();
    }*/
}