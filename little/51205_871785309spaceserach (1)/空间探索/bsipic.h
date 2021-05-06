// bsipic.h: interface for the bsipic class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_BSIPIC_H__2B2795AA_4079_4FAA_B5BF_73506D18CCEF__INCLUDED_)
#define AFX_BSIPIC_H__2B2795AA_4079_4FAA_B5BF_73506D18CCEF__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class bsipic  
{	public:	bsipic();
	virtual ~bsipic();
	public:
	//UINT g_cactus[16];	
	GLUquadricObj *g_text; 
	void Box(float x,float y,float z);
	void picter(float x,float y,float z);
	void airplane(float x,float y,float z);
	//void light0(float x,float y,float z,float a);
	//bool LoadT8(char *filename, GLuint &texture);
};

#endif // !defined(AFX_BSIPIC_H__2B2795AA_4079_4FAA_B5BF_73506D18CCEF__INCLUDED_)
