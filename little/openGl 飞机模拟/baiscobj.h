// baiscobj.h: interface for the baiscobj class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_BAISCOBJ_H__BC615F63_0D8C_41D7_87F5_4B60AB3D2EEA__INCLUDED_)
#define AFX_BAISCOBJ_H__BC615F63_0D8C_41D7_87F5_4B60AB3D2EEA__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
#define	MAX_PARTICLES	200		// 需要创建的粒子数目
#define MAX  500
struct vertex
{
	GLfloat x, y, z, light;
};

typedef struct						// 定义一个粒子的结构
{
	bool	active;					// 粒子是否处于激活状态
	GLfloat	life;					// 粒子的寿命
	GLfloat	fade;					// 粒子的淡化速度
	GLfloat	r;						// 粒子的颜色
	GLfloat	g;
	GLfloat	b;
	GLfloat	x;						// 粒子的位置坐标
	GLfloat	y;
	GLfloat	z;
	GLfloat	xi;						// 粒子的方向矢量
	GLfloat	yi;	
	GLfloat	zi;
	GLfloat	xg;						// 粒子加速度值
	GLfloat	yg;	
	GLfloat	zg;	
}
particles;
class baiscobj  
{
public:
	baiscobj();
	virtual ~baiscobj();
public:
	void DrawSmoke();
	bool DrawSun();
	bool InitSmoke(GLvoid);
	bool kongzhijian();
	bool DrawPlane(GLvoid);
	bool DrawSky(GLvoid);
	bool DrawTexture(GLvoid);
	GLfloat Hypot(GLfloat A, GLfloat B);
	GLfloat ABS(GLfloat A);
	void RestoreMyDefaultSettings();
	bool RenderScence();
	void Caculate(GLvoid);
	bool DrawTerrain(GLvoid);
	bool InitTettain(GLvoid);

vertex field[MAX+9][MAX+9];
particles particle[MAX_PARTICLES];
	
	UINT		g_cactus[16];
	BITMAPINFOHEADER  g_bit;   
	unsigned char    *g_imageData; 
	void		texture(UINT textur);
	void		light0();   
   	bool		LoadT8(char *filename, GLuint &texture);
	void		LoadT16(char *filename, GLuint &texture);
	unsigned char* LoadBit(char *filename, BITMAPINFOHEADER *bitmap);
};

#endif // !defined(AFX_BAISCOBJ_H__BC615F63_0D8C_41D7_87F5_4B60AB3D2EEA__INCLUDED_)
