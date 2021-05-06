// baiscobj.cpp: implementation of the baiscobj class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "baiscobj.h"
#include "bitmap.h"
#include "F16.h"
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
int F16;
GLuint texturem = 0;
GLfloat XPOS = -MAX/2;
GLfloat YPOS = 0;
GLfloat ZPOS = -MAX/2;
GLfloat XP=0;
GLfloat YP=0;
GLfloat ZP=0;

GLfloat xtrans = MAX/2;
GLfloat ytrans = 0;
GLfloat ztrans = MAX/2;

GLfloat visual_distance = 180;
GLfloat sun_height = 2000;
GLfloat sun_zdistance = -5000;

GLfloat xtexa; 
GLfloat ytexa; 
GLfloat xtexa2; 
GLfloat ytexa2; 
    
int xrange1 ; 
int xrange2 ;
int zrange1 ;
int zrange2 ;   

GLfloat	xrot=0;				// 绕 X 轴旋转
GLfloat	yrot=0;				// 绕 Y 轴旋转
GLfloat	zrot=0;				// 绕 Z 轴旋转
GLfloat Throttlei;
GLfloat Throttle = 5;
GLfloat _Throttle=Throttle;
GLfloat Speed = Throttle;
GLfloat Speedi;
GLfloat piover180 = 0.0174532925f;
GLfloat sceneroty;
GLfloat heading;
GLfloat pitch = 0;
GLfloat yaw = 0;

GLfloat zprot;

int quality = 3;

GLfloat H = 0;

GLfloat glow = .4f;
GLfloat glowp = 0;

bool  wireframe = FALSE;	// 线框绘制模式ON/OFF
bool  water = true;			// 是否绘制水 ON/OFF
bool  Afterburner = false;

GLUquadricObj *quadratic;
//GLuint	texture[8];

GLfloat V;
GLfloat Angle;
int loop;

baiscobj::baiscobj()
{
    
//////////////////////////////////////////////////////////////////////
	char	appdir[256];
	GetCurrentDirectory(256,appdir);
	CString dir=appdir;

	if(dir.Right(8)!="运行程序")
	SetCurrentDirectory("../运行程序");
//////////////////////////////////////////////////////////////////////
	g_imageData = LoadBit("data/images/Terrain1.bmp",&g_bit);
	LoadT8("data/images/SAND3.bmp",	 g_cactus[4]);
	LoadT8("data/images/ASPHALT.bmp", g_cactus[7]);
	LoadT8("data/images/sky.bmp", g_cactus[8]);
	LoadT8("data/images/4LEFT.bmp", g_cactus[1]);
	LoadT8("data/images/NEBULA.bmp", g_cactus[2]);
    glEnable(GL_TEXTURE_2D);
    F16=GL3DS_initialize_F16();
}	

baiscobj::~baiscobj()
{

}

void baiscobj::light0()
{	float fog_r = 50.f/255.f;
	float fog_g = 150.f/255.f;
	float fog_b = 254.f/255.f;
	glClearColor(fog_r, fog_g, fog_b, 1);			// 黑背景颜色
	glClearDepth(1.0f);		   							// 深度缓冲设置
  
	RestoreMyDefaultSettings();
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations
    // 设置光照效果
	GLfloat LightAmbient[]=		{ 0.5f, 0.5f, 0.5f, 1.0f };
	GLfloat LightDiffuse[]=		{ 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat LightSpecular[]=	{ 0.5f, 0.5f, 0.5f, 1.0f };
	GLfloat LightPosition[]=	{ 0.0f, 0.0f, 0.0f, 1.0f };
	
    glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);		
	glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);	
	glLightfv(GL_LIGHT1, GL_SPECULAR,LightSpecular);
	glLightfv(GL_LIGHT1, GL_POSITION,LightPosition);
	glEnable(GL_LIGHT1);							
	// 设置雾化效果
	GLuint	fogMode[]= { GL_EXP, GL_EXP2, GL_LINEAR };	
	GLuint	fogfilter = 0;								
	GLfloat	fogColor[4] = {fog_r, fog_g, fog_b, 1};		

	glFogi(GL_FOG_MODE, fogMode[2]);			        
	glFogfv(GL_FOG_COLOR, fogColor);					
	glFogf(GL_FOG_DENSITY, 0.294f);						
	glHint(GL_FOG_HINT, GL_NICEST);					    
	glFogf(GL_FOG_START, 10.0f);						
	glFogf(GL_FOG_END, visual_distance);				
	glEnable(GL_FOG);									

	quadratic=gluNewQuadric();						
	gluQuadricNormals(quadratic, GLU_SMOOTH);			
	gluQuadricTexture(quadratic, GL_TRUE);				
	InitSmoke();
	InitTettain();

}
void baiscobj::texture(UINT textur)
{	glBindTexture  (GL_TEXTURE_2D, textur);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
}
//==========================================================================
bool baiscobj::LoadT8(char *filename, GLuint &texture)
{	AUX_RGBImageRec *pImage = NULL;
	pImage = auxDIBImageLoad(filename);
	if(pImage == NULL)		return false;
	glGenTextures(1, &texture);	
	glBindTexture    (GL_TEXTURE_2D,texture);
	gluBuild2DMipmaps(GL_TEXTURE_2D,4, pImage->sizeX, 
					  pImage->sizeY,GL_RGB, GL_UNSIGNED_BYTE,pImage->data);
	free(pImage->data);
	free(pImage);	
	return true;
}
void baiscobj::LoadT16(char *filename, GLuint &texture)
{ glGenTextures(1, &texture);  
  glBindTexture(GL_TEXTURE_2D, texture);
  BITMAPINFOHEADER bitHeader;
  unsigned char *buffer;  
  buffer=LoadBitmapFileWithAlpha(filename,&bitHeader);
  gluBuild2DMipmaps	( GL_TEXTURE_2D,  
					  4,    
					  bitHeader.biWidth, 
					  bitHeader.biHeight,
					  GL_RGBA, 
					  GL_UNSIGNED_BYTE,
					  buffer  
					); 
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,GL_LINEAR); 
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
  free(buffer);  
}
unsigned char * baiscobj::LoadBit(char *filename, BITMAPINFOHEADER *bitmap)
{ FILE *filePtr;  
  BITMAPFILEHEADER  Header; 
  unsigned char    *Image; 
  unsigned int      imageIdx = 0; 
  unsigned char     tempRGB;  
  filePtr = fopen(filename, "rb");
  if (filePtr == NULL)    return NULL;
  fread(&Header, sizeof(BITMAPFILEHEADER), 1, filePtr);
  if (Header.bfType != BITMAP_ID)
  { fclose(filePtr);
    return NULL;
  }
  fread(bitmap, sizeof(BITMAPINFOHEADER), 1, filePtr);
  fseek(filePtr, Header.bfOffBits, SEEK_SET);
  Image = (unsigned char*)malloc(bitmap->biSizeImage);
  if (!Image)
  { free(Image);
    fclose(filePtr);
    return NULL;
  }
  fread(Image, 1, bitmap->biSizeImage, filePtr);
  if (Image == NULL)
  { fclose(filePtr);
    return NULL;
  }
  for (imageIdx = 0; imageIdx < bitmap->biSizeImage; imageIdx+=3)
  { tempRGB = Image[imageIdx];
    Image[imageIdx] = Image[imageIdx + 2];
    Image[imageIdx + 2] = tempRGB;
  }
  fclose(filePtr);
  return Image;
}
bool baiscobj::InitTettain(GLvoid)
{
    int i,i2;     
    
	field[0][0].y=(GLfloat(rand()%100)-50)/3;

	// 生成地形数据
	for (i = 0; i < MAX; i++)
	{  
		for (i2 = 0; i2 < MAX; i2++)
		{
			if (i<10 || i2<10 || i>MAX-10 || i2>MAX-10)
				field[i][i2].y=0;   
			else
				field[i][i2].y=(GLfloat(rand()%151)-75)/50+(field[i-1][i2-1].y+field[i-1][i2].y+field[i-1][i2+1].y+field[i-1][i2-2].y+field[i-1][i2+2].y)/5.05f; //Calculate the y coordinate on the same principle. 				
		}
	}
	// 地形光滑处理
   for (int cnt = 0; cnt < 3; cnt++)
   {
	   for (int t = 1; t < MAX-1; t++)
	   {
		   for (int t2 = 1; t2 < MAX-1; t2++)
		   {
			   field[t][t2].y = (field[t+1][t2].y+field[t][t2-1].y+field[t-1][t2].y+field[t][t2+1].y)/4;           
			   if (cnt == 0)
			   {
				   if (field[t][t2].y < -1 && field[t][t2].y > -1-.5) 
					   field[t][t2].y -= .45f, field[t][t2].y *= 2;
				   else if (field[t][t2].y > -1 && field[t][t2].y < -1+.5) 
					   field[t][t2].y += .5, field[t][t2].y /= 5;
			   }
		   }
	   }
   }
   return true;
}

bool baiscobj::DrawTerrain(GLvoid)
{
    int i;    
	int i2;  
	int t, t2;	
	glEnable(GL_CULL_FACE);
	glFrontFace(GL_CCW);
	glColor4f(1,1,1,1);
	
	
	for (t = xrange1; t < xrange2; t+=quality)
	{        
		for (t2 = zrange1; t2 < zrange2; t2+=quality)
		{                                     
			i = t;
			i2 = t2;
            
			while (i < 0) i += MAX;             
			while (i > MAX) i -= MAX;            
			while (i2 < 0) i2 += MAX;             
			while (i2 > MAX) i2 -= MAX;

   			xtexa = (GLfloat(i)/MAX)*57;
			xtexa2 = (GLfloat(i+quality)/MAX)*57;    
			ytexa = (GLfloat(i2)/MAX)*57;
			ytexa2 = (GLfloat(i2+quality)/MAX)*57;       
			int coord=t-MAX;
			int coord2=t2-MAX;
             texture(g_cactus[4]);
			glBegin(GL_TRIANGLE_STRIP);
			glTexCoord2f(xtexa2,ytexa2);  glVertex3f((float)(coord+quality),field[i+quality][i2+quality].y,(float)coord2+quality);
			glTexCoord2f(xtexa2,ytexa);   glVertex3f((float)(coord+quality),field[i+quality][i2].y,(float)coord2); 
			glTexCoord2f(xtexa,ytexa2);   glVertex3f((float)(coord),field[i][i2+quality].y,(float)coord2+quality); 
			glTexCoord2f(xtexa,ytexa);   glVertex3f((float)(coord),field[i][i2].y,(float)coord2); 
			glEnd();       
		}   
	}

	glEnable(GL_BLEND);
	glBlendFunc(GL_DST_COLOR, GL_ZERO);
	// 第二次绘制地形(多重纹理)

	glColor4f(1,1,1,.5f);
	for (t = xrange1; t < xrange2; t+=quality)
	{   
		for (t2 = zrange1; t2 < zrange2; t2+=quality)
		{               
			i = t;
			i2 = t2;
            
			while (i < 0) i += MAX;             
			while (i > MAX) i -= MAX;            
			while (i2 < 0) i2 += MAX;             
			while (i2 > MAX) i2 -= MAX;

			xtexa = (GLfloat(i)/MAX)*1;
			xtexa2 = (GLfloat(i+quality)/MAX)*1;
			ytexa = (GLfloat(i2)/MAX)*1;
			ytexa2 = (GLfloat(i2+quality)/MAX)*1;       
			int coord=t-MAX;
			int coord2=t2-MAX;
            
			glBegin(GL_TRIANGLE_STRIP);
			glTexCoord2f(xtexa2,ytexa2);  glVertex3f((float)(coord+quality),field[i+quality][i2+quality].y, (float)coord2+quality);
			glTexCoord2f(xtexa2,ytexa);   glVertex3f((float)(coord+quality),field[i+quality][i2].y,(float)coord2); 
			glTexCoord2f(xtexa,ytexa2);   glVertex3f((float)coord,field[i][i2+quality].y,(float)(coord2+quality)); 
			glTexCoord2f(xtexa,ytexa);   glVertex3f((float)coord,field[i][i2].y,(float)coord2); 
			glEnd();            
		}
	}   
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_BLEND);
       
	glFrontFace(GL_CW);
	glDisable(GL_CULL_FACE);
	return true;
}

void baiscobj::Caculate(GLvoid)
{
    zprot*=0.935f;
	heading += zprot/3;
	yaw += zprot/3;
	yaw*=0.95f; 

	Throttlei += (_Throttle-Throttle)/10;
	Throttlei *= 0.9f;
	Throttle += Throttlei/10;
				
	GLfloat MAX_Speed = GLfloat(sqrt(Throttle)) * 10; 
	Speedi += MAX_Speed-Speed;
	Speedi *= 0.9f;
	Speed += Speedi/1000;
	XP = -(GLfloat)sin(heading*piover180) * Speed;	                    
	YP = -(GLfloat)sin(pitch*piover180) * Speed;
	ZP = -(GLfloat)cos(heading*piover180) * Speed;
    GLfloat overallspeed = Hypot(Hypot(XP,YP),ZP) / (ABS(Speed)+1);  				

	YP *= overallspeed;
	XP *= overallspeed;
	ZP *= overallspeed;

	XPOS += XP/30;
	YPOS += YP/30;
	ZPOS += ZP/30;
}

bool baiscobj::RenderScence()
{
   	RestoreMyDefaultSettings();
	if (-XPOS < 0) XPOS -= MAX; 
	if (-XPOS > MAX) XPOS += MAX;
	if (-ZPOS < 0) ZPOS -= MAX; 
	if (-ZPOS > MAX) ZPOS += MAX;

	xtrans = -XPOS;
	ytrans = YPOS;   
	ztrans = -ZPOS;
	
	yrot = heading;
    
	sceneroty = 360.0f - yrot;
	H = sceneroty;
	if (H > 360) H = 0;
	else if (H < 0) H = 360;

	glLoadIdentity();
	glTranslatef(0,0,-10);
	glRotatef(sceneroty,0,1,0);
	glTranslatef(xtrans,ytrans-3.5f-ABS(Speed)/5,ztrans);    
   
	xrange1 = int(MAX-xtrans - visual_distance); 
	xrange2 = int(MAX-xtrans + visual_distance);
	zrange1 = int(MAX-ztrans - visual_distance);
	zrange2 = int(MAX-ztrans + visual_distance);   
  
	if (quality != 1)
	{
		xrange1 /= quality;
		xrange1 *= quality;
		xrange2 /= quality;
		xrange2 *= quality;

		zrange1 /= quality;
		zrange1 *= quality;
		zrange2 /= quality;
		zrange2 *= quality;
	}    
    DrawTexture();
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);
	Caculate();

	return TRUE;
}

void baiscobj::RestoreMyDefaultSettings()
{
    glEnable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_FOG);
}

GLfloat baiscobj::ABS(GLfloat A)
{
   if (A < 0)
		A = -A; 
	return A;
}

GLfloat baiscobj::Hypot(GLfloat A, GLfloat B)
{
   return (float)sqrt(A*A+B*B);
}

bool baiscobj::DrawTexture(GLvoid)
{
	
    DrawTerrain();		
    DrawSky();
    DrawPlane();
    glTranslatef(0.0f,0.0f,40.0f);

    DrawSmoke();
    glTranslatef(0.0f,10.0f,-80.0f);
    DrawSun();
    kongzhijian();
	return true;
}

bool baiscobj::DrawSky(GLvoid)
{
    glFogf(GL_FOG_START, MAX*2);					
	glFogf(GL_FOG_END, MAX*15);					
	glColor4f(1,1,1,1);
	glBindTexture(GL_TEXTURE_2D, g_cactus[1]);
	glTranslatef(-xtrans,-ytrans-MAX*48,-ztrans);
	glRotatef(90,1,0,1);
	gluSphere(quadratic,MAX*50,20,20); 
	glFogf(GL_FOG_START, 50.0f);				
	glFogf(GL_FOG_END, visual_distance);		
	return true;
}

bool baiscobj::DrawPlane(GLvoid)
{ 

    glColor4f(1,1,1,1);
	glLoadIdentity();
    glTranslatef(0,0,-108);
     
	glRotatef(yaw,0,1,0);
	glRotatef(zprot*15,0,0,1);
	glRotatef(pitch,1,0,0);
    glEnable(GL_LIGHTING);
	glScalef(0.8f,0.8f,0.8f);
	glCallList(F16);		// 绘制飞机模型
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);

	return true;
}

bool baiscobj::kongzhijian()
{
   if (KEY_DOWN(VK_LEFT))	
	   {
		zprot += 6/(ABS(Speed)+1);
		Throttle*=.99f;             
	}
   if (KEY_DOWN(VK_RIGHT))	
	   {
		zprot -= 6/(ABS(Speed)+1);
		Throttle*=.99f;             
	}
   if (KEY_DOWN(VK_UP))	
	   {
			pitch -= 2.0f/ (ABS(Speed)+1);
	}
   if (KEY_DOWN(VK_DOWN))	
	   {
			pitch +=2.0f / (ABS(Speed)+1);
	}
    return TRUE;
}

bool baiscobj::InitSmoke(GLvoid)
{
   	for (loop=0;loop<MAX_PARTICLES;loop++)				
	{
		particle[loop].active=true;					// 使所有的粒子激活
		particle[loop].life=1.0f;					// 给予新的生命
		particle[loop].fade=GLfloat(rand()%100)/7500 + 0.0075f;	// 随机淡化数值
		if (loop < MAX_PARTICLES/2) 
			particle[loop].x= .15f;
		else
		particle[loop].x= -.15f;
		particle[loop].y= -.15f;						// Center On Y Axis
		particle[loop].z=3;						// Center On Z Axis
		V = (GLfloat((rand()%5))+2)/5;
		Angle = GLfloat(rand()%360);
		particle[loop].zg =1.55f;
		particle[loop].xi =(float) sin(Angle) * V;
		particle[loop].yi =(float) cos(Angle) * V;
		particle[loop].zi =(float) ((rand()%10)-5)/5;
	}
	return true;
}

bool baiscobj::DrawSun()
{
    float sun_flare_size;
	sun_flare_size = 800;
	glColor4f(1,0.7f,0,0.5f);
	glBindTexture(GL_TEXTURE_2D, g_cactus[2]);//太阳贴图
	glBegin(GL_TRIANGLE_STRIP);						// Build Quad From A Triangle Strip
		glTexCoord2f(1,1); glVertex3f(MAX/2+sun_flare_size,sun_height+sun_flare_size,sun_zdistance); // Top Right
		glTexCoord2f(0,1); glVertex3f(MAX/2-sun_flare_size,sun_height+sun_flare_size,sun_zdistance); // Top Left
		glTexCoord2f(1,0); glVertex3f(MAX/2+sun_flare_size,sun_height-sun_flare_size,sun_zdistance); // Bottom Right
		glTexCoord2f(0,0); glVertex3f(MAX/2-sun_flare_size,sun_height-sun_flare_size,sun_zdistance); // Bottom Left
	glEnd();										// Done Building Triangle Strip
	
	sun_flare_size = 400;
	glColor4f(1,.5f,0,1);
	glBegin(GL_TRIANGLE_STRIP);						// Build Quad From A Triangle Strip
		glTexCoord2f(1,1); glVertex3f(MAX/2+sun_flare_size,sun_height+sun_flare_size,sun_zdistance); // Top Right
		glTexCoord2f(0,1); glVertex3f(MAX/2-sun_flare_size,sun_height+sun_flare_size,sun_zdistance); // Top Left
		glTexCoord2f(1,0); glVertex3f(MAX/2+sun_flare_size,sun_height-sun_flare_size,sun_zdistance); // Bottom Right
		glTexCoord2f(0,0); glVertex3f(MAX/2-sun_flare_size,sun_height-sun_flare_size,sun_zdistance); // Bottom Left
	glEnd();										// Done Building Triangle Strip
	return true;
}

void baiscobj::DrawSmoke()
{
  	glEnable(GL_ALPHA_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE);

	GLfloat exhaust_r, exhaust_g, exhaust_b;
	if (Afterburner)
	{
		exhaust_r = 1;
		exhaust_g = .2f;
		exhaust_b = 0;
	}
	else
	{
		exhaust_r = 1;
		exhaust_g = .2f;
		exhaust_b = 0.5f;
	}

	
	glBindTexture(GL_TEXTURE_2D, g_cactus[2]);
	glScalef(5.0f,5.0f,5.0f);
    
     glowp += .5f-glow;    
	glow += glowp*(ABS(Throttle)/500);
	if (glow > 1) glow = 1;
	else if (glow < .25f) glow = .25f;
	glColor4f(exhaust_r,exhaust_g,exhaust_b,glow);
	float glowsize = 0.6f;
	for (float glowpos = 3; glowpos <= 3.25f; glowpos+=.25f)
	{
		glowsize -= .175f;
		glBegin(GL_TRIANGLE_STRIP);						
			glTexCoord2f(1,1); glVertex3f(+glowsize,+glowsize,glowpos); 
			glTexCoord2f(0,1); glVertex3f(-glowsize,+glowsize,glowpos); 
			glTexCoord2f(1,0); glVertex3f(+glowsize,-glowsize,glowpos);
			glTexCoord2f(0,0); glVertex3f(-glowsize,-glowsize,glowpos); 
		glEnd();									

	} 
	for (loop=0;loop<MAX_PARTICLES;loop++)					
	{       	
		GLfloat x=particle[loop].x;						
		GLfloat y=particle[loop].y;					
		GLfloat z=particle[loop].z;	
		glColor4f(particle[loop].r,particle[loop].g,particle[loop].b,particle[loop].life/2);
		
     		glBegin(GL_TRIANGLE_STRIP);					
     		glTexCoord2f(1,1); glVertex3f(x+0.25f,y+0.275f,z); 
			glTexCoord2f(0,1); glVertex3f(x-0.2f,y+0.275f,z);
			glTexCoord2f(1,0); glVertex3f(x-0.2f,y+0.325f,z); 
			glTexCoord2f(0,0); glVertex3f(x+0.25f,y+0.325f,z);
		glEnd();
		glBegin(GL_TRIANGLE_STRIP);					
     		glTexCoord2f(1,1); glVertex3f(x+0.255f,y+0.325f,z); 
			glTexCoord2f(0,1); glVertex3f(x+0.255f,y+0.225f,z);
			glTexCoord2f(1,0); glVertex3f(x+0.205f,y+0.225f,z); 
			glTexCoord2f(0,0); glVertex3f(x+0.205f,y+0.325f,z);
		glEnd();
		glBegin(GL_TRIANGLE_STRIP);					
     		glTexCoord2f(1,1); glVertex3f(x-0.255f,y+0.325f,z); 
			glTexCoord2f(0,1); glVertex3f(x-0.255f,y+0.225f,z);
			glTexCoord2f(1,0); glVertex3f(x-0.205f,y+0.225f,z); 
			glTexCoord2f(0,0); glVertex3f(x-0.205f,y+0.325f,z);
		glEnd();
		glBegin(GL_TRIANGLE_STRIP);					
     		glTexCoord2f(1,1); glVertex3f(x-0.295f,y+0.325f,z); 
			glTexCoord2f(0,1); glVertex3f(x-0.295f,y+0.125f,z);
			glTexCoord2f(1,0); glVertex3f(x-0.115f,y+0.125f,z); 
			glTexCoord2f(0,0); glVertex3f(x-0.115f,y+0.325f,z);
		glEnd();
		glBegin(GL_TRIANGLE_STRIP);					
     		glTexCoord2f(1,1); glVertex3f(x+0.295f,y+0.325f,z); 
			glTexCoord2f(0,1); glVertex3f(x+0.295f,y+0.125f,z);
			glTexCoord2f(1,0); glVertex3f(x+0.115f,y+0.125f,z); 
			glTexCoord2f(0,0); glVertex3f(x+0.115f,y+0.325f,z);
		glEnd();
    	glBegin(GL_TRIANGLE_STRIP);					
     		glTexCoord2f(1,1); glVertex3f(x+0.295f,y+0.125f,z); 
			glTexCoord2f(0,1); glVertex3f(x+0.295f,y+0.005f,z);
			glTexCoord2f(1,0); glVertex3f(x+0.115f,y+0.005f,z); 
			glTexCoord2f(0,0); glVertex3f(x+0.115f,y+0.125f,z);
		glEnd();
		glBegin(GL_TRIANGLE_STRIP);					
     		glTexCoord2f(1,1); glVertex3f(x-0.295f,y+0.125f,z); 
			glTexCoord2f(0,1); glVertex3f(x-0.295f,y+0.005f,z);
			glTexCoord2f(1,0); glVertex3f(x-0.115f,y+0.005f,z); 
			glTexCoord2f(0,0); glVertex3f(x-0.115f,y+0.125f,z);
		glEnd();
		glBegin(GL_TRIANGLE_STRIP);					
     		glTexCoord2f(1,1); glVertex3f(x+0.295f,y-0.125f,z); 
			glTexCoord2f(0,1); glVertex3f(x+0.295f,y-0.005f,z);
			glTexCoord2f(1,0); glVertex3f(x+0.115f,y-0.005f,z); 
			glTexCoord2f(0,0); glVertex3f(x+0.115f,y-0.125f,z);
		glEnd();
		glBegin(GL_TRIANGLE_STRIP);					
     		glTexCoord2f(1,1); glVertex3f(x-0.295f,y-0.125f,z); 
			glTexCoord2f(0,1); glVertex3f(x-0.295f,y-0.005f,z);
			glTexCoord2f(1,0); glVertex3f(x-0.115f,y-0.005f,z); 
			glTexCoord2f(0,0); glVertex3f(x-0.115f,y-0.125f,z);
		glEnd();
		glBegin(GL_TRIANGLE_STRIP);					
     		glTexCoord2f(1,1); glVertex3f(x+0.295f,y-0.325f,z); 
			glTexCoord2f(0,1); glVertex3f(x+0.295f,y-0.125f,z);
			glTexCoord2f(1,0); glVertex3f(x+0.115f,y-0.125f,z); 
			glTexCoord2f(0,0); glVertex3f(x+0.115f,y-0.325f,z);
		glEnd();
		glBegin(GL_TRIANGLE_STRIP);					
     		glTexCoord2f(1,1); glVertex3f(x-0.295f,y-0.325f,z); 
			glTexCoord2f(0,1); glVertex3f(x-0.295f,y-0.125f,z);
			glTexCoord2f(1,0); glVertex3f(x-0.115f,y-0.125f,z); 
			glTexCoord2f(0,0); glVertex3f(x-0.115f,y-0.325f,z);
		glEnd();
		glBegin(GL_TRIANGLE_STRIP);					
     		glTexCoord2f(1,1); glVertex3f(x+0.19f,y-0.3f,z); 
			glTexCoord2f(0,1); glVertex3f(x-0.1f,y-0.3f,z);
			glTexCoord2f(1,0); glVertex3f(x-0.1f,y-0.4f,z); 
			glTexCoord2f(0,0); glVertex3f(x+0.19f,y-0.4f,z);
		glEnd();


	
   		particle[loop].x+=particle[loop].xi/200;
		particle[loop].y+=particle[loop].yi/200;
		particle[loop].z+=particle[loop].zi/200;
		particle[loop].xi*=0.975f;
		particle[loop].yi*=0.975f;
		particle[loop].zi*=0.975f;
		particle[loop].zi+=particle[loop].zg;			// Take Pull On Z Axis Into Account
		particle[loop].life-=particle[loop].fade*3;		// Reduce Particles Life By 'Fade'
		if (particle[loop].life < 0.5f) 
			particle[loop].life*=.975f;

		if (particle[loop].life<0.05f)					// If Particle Is Burned Out
		{ 			    
			particle[loop].r=exhaust_r;
			particle[loop].g=exhaust_g;
			particle[loop].b=exhaust_b;
			
			particle[loop].life=1.0f;					// Give It New Life
			particle[loop].fade=GLfloat(rand()%100)/2500 + 0.015f;	// Random Fade Value
         
			if (loop < MAX_PARTICLES/2) 
				particle[loop].x= 0.01f;						
			else  
			particle[loop].x= -0.01f;						
			particle[loop].y= -0.0f;						
			particle[loop].z= 3.0f;						
			V = (GLfloat((rand()%5))+2)/5;
			Angle = GLfloat(rand()%360);
              
			particle[loop].xi =(float) sin(Angle) * V;
			particle[loop].yi = (float) cos(Angle) * V;
			particle[loop].zi = ((rand()%10)-5)/5 + Throttle*3;
		}
	} 
	glDisable(GL_FOG);
	glLoadIdentity();
	glRotatef(sceneroty,0,1,0);

}
