#include <stdlib.h>
#include <GL/glut.h>
#include <stdlib.h>

void changeSize( int w , int h);
void renderScene();
 
//a[][]={ 0.36    0.032
//        0.98544 0.095744
//        1.80588 0.191082
//        2.7689  0.317932
//        3.8355  0.476251
//        4.97676 0.666023
//        6.17121 0.887245
//        7.40299 1.13993
//        8.66039 1.42408
//        9.9348  1.73972
//        11.2199 2.08687
//        12.5112 2.46555
//        13.8053 2.87577
//        15.0999 3.31756
//        16.3933 3.79092
//        17.6844 4.29589
//        18.9724 4.83247
//        20.2569 5.40068
//        21.5375 6.00054
//        22.8141 6.63206
//        24.0867 7.29527
//        25.3554 7.99016
//        26.6201 8.71675
//        27.8812 9.47507
//        29.1386 10.2651
//        30.3925 11.0869
//        31.6432 11.9404
//        32.8908 12.8257
//        34.1355 13.7428
//        35.3774 14.6917
//        36.6167 15.6723
//        37.8535 16.6847
//        39.088  17.729
//        40.3204 18.8051
//        41.5507 19.913
//        42.7791 21.0527
//        44.0058 22.2243
//        45.2307 23.4277
//        46.4541 24.663
//        47.676  25.9301
//        48.8965 27.2291
//        50.1157 28.56
//        51.3337 29.9227
//        52.5506 31.3173
//        53.7664 32.7439
//        54.9811 34.2023
//        56.195  35.6926
//        57.408  37.2148
//        58.6201 38.7689
//        59.8315 40.3549
//        61.0422 41.9728
//        62.2522 43.6227
//        63.4616 45.3045
//        64.6703 47.0182
//        65.8785 48.7638
//        67.0862 50.5413
//        68.2934 52.3508
//        69.5002 54.1922
//        70.7065 56.0656
//        71.9124 57.9709
//        73.118  59.9082
//        74.3232 61.8773
//        75.528  63.8785
//        76.7326 65.9116
//        77.9368 67.9766
//        79.1408 70.0736
//        80.3446 72.2026
//        81.5481 74.3635
//        82.7514 76.5564
//        83.9545 78.7812
//        85.1573 81.038
//        86.3601 83.3268
//        87.5626 85.6475
//        88.7649 88.0002
//		89.9672 90.3849};


int main(int argc ,char** argv)
{
    glutInit( &argc , argv);
    glutInitDisplayMode( GLUT_DEPTH|GLUT_SINGLE|GLUT_RGBA);
    glutInitWindowPosition(0,0);
    glutInitWindowSize(1920,1080);
    glutCreateWindow("GLUT TUT");
    glutDisplayFunc(renderScene);
    //空闲时调用
    glutIdleFunc(renderScene);
    //
    glutReshapeFunc(changeSize);
    //开启深度测试，因为默认没有开启
    glEnable(GL_DEPTH_TEST);
    glutMainLoop();
}
 
//w:宽度 h:高度
void changeSize( int w , int h)
{
   //防止除数，即高度为0
    if( h == 0)
        h = 1;
    float ratio = 1.0 * w / h;
    //单位化投影矩阵
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //设置窗口大小为单个窗口大小
    glViewport(0, 0, w, h);
    //
    gluPerspective(45,ratio,1,1000);
    //
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    //
    gluLookAt(0.0,0.0,5.0, 0.0,0.0,-1.0,0.0f,1.0f,0.0f);
}
 
float angle = 0.0;
float rtx = 0.0;
float rty = 0.0;
void renderScene()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //
    glPushMatrix();
    //
    glRotatef(angle , 0.0, 0.0 , 1.0);
    //
	glTranslatef(rtx, rty, 0);

    glBegin(GL_TRIANGLES);
        
	glColor3f(0.96f, 0.96f, 0.96f);
	glVertex4f(0.0f, 10.0f, 0.0f,100.0f);
 
	//glColor3f(1.0f, 0.0f, 0.0f);
	glVertex4f(-3.0f, -10.0f, 0.0f,100.0f);
 
	//glColor3f(0.0f, 0.0f, 1.0f);
	glVertex4f(3.0f, -10.0f, 0.0f,100.0f);
	//第二个三角形
	glBegin(GL_TRIANGLES);
	glColor3f(0.96f, 0.96f, 0.96f);
	glVertex4f(0.0f, 3.0f, 0.0f,100.0f);
 
	//glColor3f(1.0f, 0.0f, 0.0f);
	glVertex4f(7.0f, -8.0f, 0.0f,100.0f);
 
	//glColor3f(0.0f, 0.0f, 1.0f);
	glVertex4f(-7.0f, -8.0f, 0.0f,100.0f);
     glEnd();
     //
     glPopMatrix();
     //
     glutSwapBuffers();
     glFlush();
     angle+=0.01;
	 if(rtx<0.55)rtx+=0.00001;
	 
	 if(rty<0.55)rty+=0.00001;
	 
	 
}